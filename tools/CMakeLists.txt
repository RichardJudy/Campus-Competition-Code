cmake_minimum_required(VERSION 3.16)

# 极简版 tools：仅保留当前 demo 依赖的工具模块
add_library(tools OBJECT
    exiter.cpp
    img_tools.cpp
    math_tools.cpp
    logger.cpp
)