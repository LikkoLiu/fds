# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
# cmake_minimum_required(VERSION 3.16)
set(component_srcs      "src/i2c_tof.c"
                        "src/i2c_imu.c"
                        "src/uart_gps.c"
                        "src/power_control.c"
                        "src/bmi160.c"
                        "src/MadgwickAHRS.cpp"
)

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver"
                       PRIV_REQUIRES driver esp_timer
                       REQUIRES "")