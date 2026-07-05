# winchy_fast: native helpers for the rope's hot paths (IMU sampler).
add_library(usermod_winchy_fast INTERFACE)

target_sources(usermod_winchy_fast INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/winchy_fast.c
)

target_include_directories(usermod_winchy_fast INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_winchy_fast)
