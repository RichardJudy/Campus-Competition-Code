cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
# OpenVINO removed - using traditional vision only
# set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
# find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
# 只保留传统视觉装甲板（auto_aim），关闭打符与全局感知模块
# add_subdirectory(tasks/auto_buff)
# add_subdirectory(tasks/omniperception)

####################src###################
# 只保留与传统 auto_aim 直接相关或不依赖 auto_buff/omniperception 的程序
# 下列程序依赖 auto_buff/OpenVINO 或 MPC，已关闭：
# add_executable(standard src/standard.cpp)
# add_executable(mt_standard src/mt_standard.cpp)
# add_executable(standard_mpc src/standard_mpc.cpp)
# add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
# add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
# add_executable(auto_buff_debug src/auto_buff_debug.cpp)
# add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
# add_executable(uav src/uav.cpp)
# uav_debug 依赖 YOLO/Tracker/Aimer/Shooter，已关闭
# add_executable(uav_debug src/uav_debug.cpp)

# target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
# target_link_libraries(mt_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
# target_link_libraries(standard_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
# target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
# target_link_libraries(mt_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
# target_link_libraries(auto_buff_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
# target_link_libraries(auto_buff_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
# target_link_libraries(uav ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
# target_link_libraries(uav_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

##################calibration################
# 标定相关程序
add_executable(capture calibration/capture.cpp)
target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)

##################tests##################
# 当前仅保留 detector + solver + usbcamera_detect_test，这里只编译这一个测试程序

add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
target_link_libraries(usbcamera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

# Copy configs and assets directories to build directory
file(COPY ${CMAKE_SOURCE_DIR}/configs DESTINATION ${CMAKE_BINARY_DIR})
file(COPY ${CMAKE_SOURCE_DIR}/assets DESTINATION ${CMAKE_BINARY_DIR})


# planner_test 依赖 Planner 模块，已关闭
# add_executable(planner_test tests/planner_test.cpp)
# target_link_libraries(planner_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# planner_test_offline 依赖 Planner 模块，已关闭
# add_executable(planner_test_offline tests/planner_test_offline.cpp)
# target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# 检查 ROS 环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)
# 只有在找到 ROS 环境时才会添加 sentry 和 publish_test
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
    # 编译 sentry 和 publish_test
    add_executable(sentry src/sentry.cpp)
    add_executable(sentry_bp src/sentry_bp.cpp)
    add_executable(sentry_debug src/sentry_debug.cpp)
    add_executable(sentry_multithread src/sentry_multithread.cpp)
    add_executable(publish_test tests/publish_test.cpp)
    add_executable(subscribe_test tests/subscribe_test.cpp)
    add_executable(topic_loop_test tests/topic_loop_test.cpp)

    # 全局感知使用 OpenVINO/omniperception，已关闭，仅保留传统 auto_aim
    # target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    # target_link_libraries(sentry_bp ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    # target_link_libraries(sentry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    # target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
    target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()