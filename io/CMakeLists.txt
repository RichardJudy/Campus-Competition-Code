cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_subdirectory(serial)

# 极简版 io：当前只需要 USB 相机 + serial，用于 usbcamera_detect_test
add_library(io STATIC
    usbcamera/usbcamera.cpp
)

target_link_libraries(io yaml-cpp serial usb-1.0)
