# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
# cmake_minimum_required(VERSION 3.16)
set(component_srcs      "power/src/power_control.c"
                        "imu/src/i2c_imu.c"
                        "imu/src/bmi160.c"
                        "imu/src/madgwickFilter.c"
                        "tof/src/i2c_tof.c"
                        "gps/src/uart_gps.c"
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