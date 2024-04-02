cmake_minimum_required(VERSION 3.15)
project(glad)

add_library(glad STATIC src/gl.c)
target_include_directories(glad PUBLIC include)
target_link_libraries(glad OpenGL::GL)
