// Winchy - glider winch rope force & advice system
// Copyright (C) 2026 Dominic Spreitz
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version. Distributed WITHOUT ANY WARRANTY.
// See the GNU General Public License for more details, and the LICENSE
// file or <https://www.gnu.org/licenses/> for the full text.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// winchy_fast: hardware-exact QMI8658 IMU sampling for the rope.
//
// The Python IMU loop tops out at ~36 Hz of its 50 Hz target: even with the
// burst read, the asyncio wake-up latency behind other tasks' non-yielding
// blocks (mag I2C, flash writes, console) stretches the cadence. This module
// moves the SAMPLING out of the interpreter entirely:
//
//   * an esp_timer periodic callback (runs in the high-priority esp_timer
//     FreeRTOS task, preempting the MicroPython task) does one 13-byte SPI
//     polling transaction per period and stores the raw sample + timestamp
//     into a static ring buffer;
//   * Python drains finished samples with imu_next() at whatever cadence the
//     asyncio loop manages - the SAMPLE SPACING is exact regardless.
//
// The register setup mirrors winchy/sensors/qmi8658.py exactly (CTRL1 0x60 =
// ADDR_AI + byte order as before, CTRL2 0x05, CTRL3 0x45, CTRL7 0x83,
// CTRL5 0x33); values returned are the same raw int16s the Python driver
// assembles, scaling stays in Python (imu_fast.py).

#include "py/runtime.h"
#include "py/obj.h"
#include "py/mphal.h"

#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define QMI_WHO_AM_I_REG 0x00
#define QMI_WHO_AM_I_VAL 0x05
#define QMI_REG_AX_L 0x35
#define RING_N 128            // 2.56 s of backlog at 50 Hz

typedef struct {
    int64_t t_us;             // esp_timer_get_time() at the sample
    int16_t v[6];             // ax, ay, az, gx, gy, gz (raw)
} sample_t;

static spi_device_handle_t s_dev = NULL;
static esp_timer_handle_t s_timer = NULL;
static sample_t s_ring[RING_N];
static volatile uint32_t s_head = 0;  // written by the timer task
static uint32_t s_tail = 0;           // read cursor, MicroPython task only
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static void qmi_write(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    spi_device_polling_transmit(s_dev, &t);
}

static uint8_t qmi_read1(uint8_t reg) {
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0 };
    uint8_t rx[2] = { 0, 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(s_dev, &t);
    return rx[1];
}

// Runs in the esp_timer FreeRTOS task (NOT an ISR, NOT the MicroPython task):
// plain C state only - no MicroPython objects, no allocation, no GIL.
static void sample_cb(void *arg) {
    uint8_t tx[13] = { QMI_REG_AX_L | 0x80 };
    uint8_t rx[13] = { 0 };
    spi_transaction_t t = { .length = 13 * 8, .tx_buffer = tx, .rx_buffer = rx };
    if (spi_device_polling_transmit(s_dev, &t) != ESP_OK) {
        return;               // drop this sample; the next tick retries
    }
    sample_t smp;
    smp.t_us = esp_timer_get_time();
    for (int i = 0; i < 6; i++) {
        // byte order identical to the Python driver: value = b[addr+1]<<8 | b[addr]
        smp.v[i] = (int16_t)((rx[2 + 2 * i] << 8) | rx[1 + 2 * i]);
    }
    taskENTER_CRITICAL(&s_mux);
    s_ring[s_head % RING_N] = smp;
    s_head++;
    taskEXIT_CRITICAL(&s_mux);
}

// imu_init(sck, mosi, miso, cs, hz=50) -> WHO_AM_I. Owns the SPI bus (the
// Python side must NOT create machine.SPI(2) when this module is used - see
// winchy/board.py). Idempotent across soft reboots: the bus/device and a
// running timer survive a MicroPython soft reset, so re-init just restarts.
static mp_obj_t winchy_imu_init(size_t n_args, const mp_obj_t *args) {
    int sck = mp_obj_get_int(args[0]);
    int mosi = mp_obj_get_int(args[1]);
    int miso = mp_obj_get_int(args[2]);
    int cs = mp_obj_get_int(args[3]);
    int hz = (n_args > 4) ? mp_obj_get_int(args[4]) : 50;
    if (hz < 1 || hz > 250) {
        mp_raise_ValueError(MP_ERROR_TEXT("hz must be 1..250"));
    }

    if (s_timer != NULL) {
        esp_timer_stop(s_timer);          // may not be running - fine
    }

    if (s_dev == NULL) {                  // first init since power-up
        spi_bus_config_t bus = {
            .mosi_io_num = mosi,
            .miso_io_num = miso,
            .sclk_io_num = sck,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 32,
        };
        if (spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_DISABLED) != ESP_OK) {
            mp_raise_msg(&mp_type_RuntimeError,
                         MP_ERROR_TEXT("SPI3 bus init failed"));
        }
        spi_device_interface_config_t dev = {
            .clock_speed_hz = 1000000,
            .mode = 3,                    // matches machine.SPI polarity=1 phase=1
            .spics_io_num = cs,
            .queue_size = 2,
        };
        if (spi_bus_add_device(SPI3_HOST, &dev, &s_dev) != ESP_OK) {
            mp_raise_msg(&mp_type_RuntimeError,
                         MP_ERROR_TEXT("SPI3 add device failed"));
        }
    }

    // The IMU can read 0x00 for the first ms after power-up (see the Python
    // driver) - poll WHO_AM_I before giving up.
    uint8_t who = 0;
    for (int i = 0; i < 20; i++) {
        who = qmi_read1(QMI_WHO_AM_I_REG);
        if (who == QMI_WHO_AM_I_VAL) {
            break;
        }
        mp_hal_delay_ms(10);
    }
    if (who != QMI_WHO_AM_I_VAL) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("QMI8658 not found"));
    }

    // Same configuration as winchy/sensors/qmi8658.py.
    qmi_write(0x02, 0x60);    // CTRL1: ADDR_AI + byte-order bit as before
    qmi_write(0x03, 0x05);    // CTRL2: accel +/-2g, 250 Hz ODR
    qmi_write(0x04, 0x45);    // CTRL3: gyro +/-256 dps, 250 Hz ODR
    qmi_write(0x08, 0x83);    // CTRL7: accel + gyro enabled
    qmi_write(0x06, 0x33);    // CTRL5: LPF on, ~9 Hz
    mp_hal_delay_ms(10);      // first conversions settle

    taskENTER_CRITICAL(&s_mux);
    s_head = 0;
    s_tail = 0;
    taskEXIT_CRITICAL(&s_mux);

    if (s_timer == NULL) {
        const esp_timer_create_args_t targs = {
            .callback = sample_cb,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "winchy_imu",
        };
        if (esp_timer_create(&targs, &s_timer) != ESP_OK) {
            mp_raise_msg(&mp_type_RuntimeError,
                         MP_ERROR_TEXT("esp_timer create failed"));
        }
    }
    esp_timer_start_periodic(s_timer, 1000000 / hz);
    return MP_OBJ_NEW_SMALL_INT(who);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(winchy_imu_init_obj, 4, 5,
                                           winchy_imu_init);

// imu_next() -> (t_ms, ax, ay, az, gx, gy, gz) raw int16s, or None when the
// ring is drained. t_ms is masked to MicroPython's 2^30 ticks period so
// time.ticks_diff works on it directly.
static mp_obj_t winchy_imu_next(void) {
    if (s_tail == s_head) {
        return mp_const_none;
    }
    taskENTER_CRITICAL(&s_mux);
    if ((uint32_t)(s_head - s_tail) > RING_N) {
        s_tail = s_head - RING_N;         // overran: drop the oldest
    }
    sample_t smp = s_ring[s_tail % RING_N];
    s_tail++;
    taskEXIT_CRITICAL(&s_mux);
    mp_obj_t items[7];
    items[0] = mp_obj_new_int_from_uint(
        (uint32_t)(smp.t_us / 1000) & 0x3FFFFFFF);
    for (int i = 0; i < 6; i++) {
        items[1 + i] = MP_OBJ_NEW_SMALL_INT(smp.v[i]);
    }
    return mp_obj_new_tuple(7, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(winchy_imu_next_obj, winchy_imu_next);

// imu_count() -> samples currently waiting in the ring (debug/telemetry).
static mp_obj_t winchy_imu_count(void) {
    uint32_t n = s_head - s_tail;
    return MP_OBJ_NEW_SMALL_INT(n > RING_N ? RING_N : n);
}
static MP_DEFINE_CONST_FUN_OBJ_0(winchy_imu_count_obj, winchy_imu_count);

// imu_stop() -> stop the sampler (deploys/tests).
static mp_obj_t winchy_imu_stop(void) {
    if (s_timer != NULL) {
        esp_timer_stop(s_timer);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(winchy_imu_stop_obj, winchy_imu_stop);

static const mp_rom_map_elem_t winchy_fast_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_winchy_fast) },
    { MP_ROM_QSTR(MP_QSTR_imu_init), MP_ROM_PTR(&winchy_imu_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_imu_next), MP_ROM_PTR(&winchy_imu_next_obj) },
    { MP_ROM_QSTR(MP_QSTR_imu_count), MP_ROM_PTR(&winchy_imu_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_imu_stop), MP_ROM_PTR(&winchy_imu_stop_obj) },
};
static MP_DEFINE_CONST_DICT(winchy_fast_globals, winchy_fast_globals_table);

const mp_obj_module_t winchy_fast_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&winchy_fast_globals,
};
MP_REGISTER_MODULE(MP_QSTR_winchy_fast, winchy_fast_user_cmodule);
