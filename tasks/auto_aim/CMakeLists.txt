cmake_minimum_required(VERSION 3.16)

add_library(auto_aim OBJECT
    armor.cpp
    detector.cpp
    solver.cpp
)

target_link_libraries(auto_aim io)