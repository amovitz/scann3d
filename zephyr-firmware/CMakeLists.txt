cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(scanner VERSION 0.1.0)

# VL53L8CX ULD sources - clone from:
#   https://github.com/STMicroelectronics/VL53L8CX_ULD_driver
# and place under lib/vl53l8cx_uld/
set(VL53L8CX_ULD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/vl53l8cx_uld)
set(VL53L8CX_ULD_SOURCES
    ${VL53L8CX_ULD_DIR}/vl53l8cx_api.c
    ${VL53L8CX_ULD_DIR}/platform.c
    ${VL53L8CX_ULD_DIR}/vl53l8cx_plugin_detection_thresholds.c
    ${VL53L8CX_ULD_DIR}/vl53l8cx_plugin_motion_indicator.c
    ${VL53L8CX_ULD_DIR}/vl53l8cx_plugin_xtalk.c
)

target_sources(app PRIVATE
    src/main.c
    src/work_queues.c
    src/protocol.c
    src/wifi_mgr.c
    src/uart_aux.c
    src/led/led_ctrl.c
    src/servo/servo_ctrl.c
    src/vl53l8cx/vl53l8cx_driver.c
    src/lsm6dsv/lsm6dsv_driver.c
    src/output/data_output.c
    src/output/udp_out.c
    src/output/usb_serial.c
    ${VL53L8CX_ULD_SOURCES}
)

target_include_directories(app PRIVATE
    src
    src/vl53l8cx
    src/lsm6dsv
    src/led
    src/servo
    src/output
    ${VL53L8CX_ULD_DIR}
)

# Let the ULD find our platform header
target_compile_definitions(app PRIVATE
    VL53L8CX_ULD_CUSTOM_PLATFORM=1
)
