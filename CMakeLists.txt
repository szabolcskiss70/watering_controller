# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# (Not part of the boilerplate)
# This example uses an extra component for common functions such as Wi-Fi and Ethernet connection.

set(EXTRA_COMPONENT_DIRS "C:/ESP32/Projects/components/u8g2" "C:/ESP32/esp-idf_v4.0/examples/common_components/protocol_examples_common" "C:/ESP32/Projects/components/ADS71020" "C:/ESP32/Projects/components/lora")




set(PROJECT_VER "1.0.0.52")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(watering_controller)
