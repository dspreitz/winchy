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
// v2 - REDESIGNED after the 2026-07-05 stability bisect. v1 sampled from the
// esp_timer SERVICE task with spi_device_polling_transmit: a stuck polling
// transaction (priority 22, core 0) could busy-hold the very task the WiFi
// stack's timers depend on - hard VM freezes (hotspot-dance repro) and
// rst=WDT/PANIC in the field. v2 removes the whole failure class:
//
//   * a DEDICATED FreeRTOS task (priority 10, core 0, vTaskDelayUntil paces
//     the period) - no shared service is ever held hostage;
//   * INTERRUPT-driven SPI transactions with a bounded wait
//     (spi_device_queue_trans + get_trans_result, 15 ms timeout) - a stuck
//     transaction is reaped on a later cycle, never spun on forever;
//   * self-quarantine: errors only cost samples, and imu_stats() exposes
//     the counters so degradation is visible in Python.
//
// The register setup mirrors winchy/sensors/qmi8658.py exactly; values are
// the same raw int16s, scaling stays in Python (imu_fast.py).

#include "py/runtime.h"
#include "py/obj.h"
#include "py/mphal.h"

#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"   // xTaskCreatePinnedToCore (IDF >= 5)

#define QMI_WHO_AM_I_REG 0x00
#define QMI_WHO_AM_I_VAL 0x05
#define QMI_REG_AX_L 0x35
#define RING_N 128            // 2.56 s of backlog at 50 Hz
#define SPI_WAIT_TICKS pdMS_TO_TICKS(15)

typedef struct {
    int64_t t_us;             // esp_timer_get_time() at the sample
    int16_t v[6];             // ax, ay, az, gx, gy, gz (raw)
} sample_t;

static spi_device_handle_t s_dev = NULL;
static TaskHandle_t s_task = NULL;
static volatile bool s_run = false;
static volatile uint32_t s_period_ms = 20;

static sample_t s_ring[RING_N];
static volatile uint32_t s_head = 0;  // written by the sampler task
static uint32_t s_tail = 0;           // read cursor, MicroPython task only
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// diagnostics (imu_stats)
static volatile uint32_t s_spi_err = 0;    // queue/transmit errors
static volatile uint32_t s_spi_timeout = 0;  // bounded waits that expired
static volatile uint32_t s_reaped = 0;     // late transactions reaped OK

// The queued transaction + buffers must stay valid while in flight.
static spi_transaction_t s_trans;
static bool s_pending = false;
static uint8_t s_txbuf[13];
static uint8_t s_rxbuf[13];

// --- init-time register access (MicroPython task, sampler not running) -----

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

// --- the sampler task -------------------------------------------------------

static void sample_once(void) {
    // Reap a transaction that timed out on an earlier cycle: the driver
    // requires collecting it before queueing a new one. Never blocks.
    if (s_pending) {
        spi_transaction_t *done;
        if (spi_device_get_trans_result(s_dev, &done, 0) == ESP_OK) {
            s_pending = false;
            s_reaped++;
        } else {
            s_spi_timeout++;          // still stuck; try again next cycle
            return;
        }
    }
    s_txbuf[0] = QMI_REG_AX_L | 0x80;
    for (int i = 1; i < 13; i++) {
        s_txbuf[i] = 0;
    }
    s_trans = (spi_transaction_t){ .length = 13 * 8,
                                   .tx_buffer = s_txbuf,
                                   .rx_buffer = s_rxbuf };
    if (spi_device_queue_trans(s_dev, &s_trans, SPI_WAIT_TICKS) != ESP_OK) {
        s_spi_err++;
        return;
    }
    s_pending = true;
    spi_transaction_t *done;
    if (spi_device_get_trans_result(s_dev, &done, SPI_WAIT_TICKS) != ESP_OK) {
        s_spi_timeout++;              // reaped on a later cycle
        return;
    }
    s_pending = false;
    sample_t smp;
    smp.t_us = esp_timer_get_time();
    for (int i = 0; i < 6; i++) {
        // byte order identical to the Python driver: value = b[addr+1]<<8 | b[addr]
        smp.v[i] = (int16_t)((s_rxbuf[2 + 2 * i] << 8) | s_rxbuf[1 + 2 * i]);
    }
    taskENTER_CRITICAL(&s_mux);
    s_ring[s_head % RING_N] = smp;
    s_head++;
    taskEXIT_CRITICAL(&s_mux);
}

static void sampler_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(s_period_ms));
        if (s_run && s_dev != NULL) {
            sample_once();
        }
    }
}

// --- Python API --------------------------------------------------------------

// imu_init(sck, mosi, miso, cs, hz=50) -> WHO_AM_I. Owns the SPI bus (the
// Python side must NOT create machine.SPI(2) when this path is active - see
// winchy/board.py + config.IMU_FAST). Idempotent across soft reboots: bus,
// device and the sampler task survive; re-init pauses sampling, reconfigures
// and resumes.
static mp_obj_t winchy_imu_init(size_t n_args, const mp_obj_t *args) {
    int sck = mp_obj_get_int(args[0]);
    int mosi = mp_obj_get_int(args[1]);
    int miso = mp_obj_get_int(args[2]);
    int cs = mp_obj_get_int(args[3]);
    int hz = (n_args > 4) ? mp_obj_get_int(args[4]) : 50;
    if (hz < 1 || hz > 250) {
        mp_raise_ValueError(MP_ERROR_TEXT("hz must be 1..250"));
    }

    s_run = false;                    // pause a running sampler (soft reboot)
    mp_hal_delay_ms(40);              // let an in-flight cycle finish

    if (s_dev == NULL) {              // first init since power-up
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
            .mode = 3,                // matches machine.SPI polarity=1 phase=1
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
    qmi_write(0x06, 0x55);    // CTRL5: LPF mode 10 = 5.32% of ODR (~13 Hz)
    mp_hal_delay_ms(10);      // first conversions settle

    taskENTER_CRITICAL(&s_mux);
    s_head = 0;
    s_tail = 0;
    taskEXIT_CRITICAL(&s_mux);
    s_spi_err = 0;
    s_spi_timeout = 0;
    s_reaped = 0;
    s_period_ms = 1000 / hz;
    if (s_period_ms == 0) {
        s_period_ms = 1;
    }

    if (s_task == NULL) {
        // Priority 10: above idle, well below WiFi (23) and esp_timer (22) -
        // even a misbehaving sampler cannot starve the system services.
        if (xTaskCreatePinnedToCore(sampler_task, "winchy_imu", 3072, NULL,
                                    10, &s_task, 0) != pdPASS) {
            mp_raise_msg(&mp_type_RuntimeError,
                         MP_ERROR_TEXT("sampler task create failed"));
        }
    }
    s_run = true;
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

// imu_count() -> samples currently waiting in the ring.
static mp_obj_t winchy_imu_count(void) {
    uint32_t n = s_head - s_tail;
    return MP_OBJ_NEW_SMALL_INT(n > RING_N ? RING_N : n);
}
static MP_DEFINE_CONST_FUN_OBJ_0(winchy_imu_count_obj, winchy_imu_count);

// imu_stats() -> (total_samples, spi_errors, spi_timeouts, reaped_late) -
// visibility into the sampler's health from Python (dashboard/logs).
static mp_obj_t winchy_imu_stats(void) {
    mp_obj_t items[4] = {
        mp_obj_new_int_from_uint(s_head),
        mp_obj_new_int_from_uint(s_spi_err),
        mp_obj_new_int_from_uint(s_spi_timeout),
        mp_obj_new_int_from_uint(s_reaped),
    };
    return mp_obj_new_tuple(4, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(winchy_imu_stats_obj, winchy_imu_stats);

// imu_stop() -> pause sampling (deploys/tests). The task stays parked.
static mp_obj_t winchy_imu_stop(void) {
    s_run = false;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(winchy_imu_stop_obj, winchy_imu_stop);

static const mp_rom_map_elem_t winchy_fast_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_winchy_fast) },
    { MP_ROM_QSTR(MP_QSTR_imu_init), MP_ROM_PTR(&winchy_imu_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_imu_next), MP_ROM_PTR(&winchy_imu_next_obj) },
    { MP_ROM_QSTR(MP_QSTR_imu_count), MP_ROM_PTR(&winchy_imu_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_imu_stats), MP_ROM_PTR(&winchy_imu_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_imu_stop), MP_ROM_PTR(&winchy_imu_stop_obj) },
};
static MP_DEFINE_CONST_DICT(winchy_fast_globals, winchy_fast_globals_table);

const mp_obj_module_t winchy_fast_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&winchy_fast_globals,
};
MP_REGISTER_MODULE(MP_QSTR_winchy_fast, winchy_fast_user_cmodule);
