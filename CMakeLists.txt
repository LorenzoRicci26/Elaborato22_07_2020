cmake_minimum_required(VERSION 3.16)
project(Elaborato22_07_2020)

set(CMAKE_CXX_STANDARD 14)

find_package(SFML 2.5 COMPONENTS system window graphics)

add_executable(Elaborato22_07_2020 main.cpp stlastar.h fsa.h Subject.cpp Subject.h Observer.cpp Observer.h)

target_link_libraries(Elaborato22_07_2020 sfml-system sfml-window sfml-graphics)