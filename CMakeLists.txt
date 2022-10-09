#

cmake_minimum_required(VERSION 3.5)

include_directories(components)
add_definitions(-D__MANUVR_ESP32=1 -DMANUVR_PLATFORM=ESP32 -DCONFIG_C3P_I2C -DCONFIG_C3P_SUPPORT_SPI -DCONFIG_C3P_CBOR)
set(EXTRA_COMPONENT_DIRS "components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lux-turpis)

#message(STATUS "****************  " ${EXTRA_COMPONENTS_DIRS})
