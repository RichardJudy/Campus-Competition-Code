cmake_minimum_required(VERSION 3.16)

# 最小 auto_aim：只保留传统视觉装甲板检测 + 弹道解算
# 不编译：classifier、YOLO、planner、tinympc、打符、跟踪、瞄准、射击、多线程等

add_library(auto_aim OBJECT
    armor.cpp
    detector.cpp
    solver.cpp
)

target_link_libraries(auto_aim io)