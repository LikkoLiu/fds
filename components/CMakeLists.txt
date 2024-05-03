# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
# cmake_minimum_required(VERSION 3.16)
set(component_srcs      "power/src/power_control.c"
                        "imu/src/i2c_imu.c"
                        "imu/src/bmi160.c"
                        "imu/src/madgwickFilter.c"
                        "gps/src/uart_gps.c"
                        "tof/src/i2c_tof.c"
                        "tof/src/i2c_drv.c" 
                        "tof/src/i2cdev_esp32.c"
                        "tof/src/vl53l1x.c"
                        "tof/src/vl53l1_api_calibration.c"
                        "tof/src/vl53l1_api_core.c"
                        "tof/src/vl53l1_api_debug.c"
                        "tof/src/vl53l1_api_preset_modes.c"
                        "tof/src/vl53l1_api_strings.c"
                        "tof/src/vl53l1_api.c"
                        "tof/src/vl53l1_core_support.c"
                        "tof/src/vl53l1_core.c"
                        "tof/src/vl53l1_error_strings.c"
                        "tof/src/vl53l1_register_funcs.c"
                        "tof/src/vl53l1_silicon_core.c"
                        "tof/src/vl53l1_wait.c"
)

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "power/include"
                                    "imu/include"
                                    "tof/include"
                                    "gps/include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver"
                       PRIV_REQUIRES driver esp_timer
                       REQUIRES "")