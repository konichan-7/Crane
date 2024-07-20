cmake_minimum_required(VERSION 3.16)

add_library(tools OBJECT 
    exiter.cpp
    extended_kalman_filter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    logger.cpp
)