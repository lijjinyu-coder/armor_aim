cmake_minimum_required(VERSION 3.16)
project(armor_aim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 找 OpenCV
find_package(OpenCV REQUIRED)

add_executable(armor_aim
    src/main.cpp
    src/ConfigLoader.cpp
    src/VideoInput.cpp
    src/ArmorDetector.cpp
    src/TargetSelector.cpp
    src/PoseSolver.cpp
    src/DebugDrawer.cpp
    src/DataPacker.cpp
    src/DataParser.cpp
)

target_include_directories(armor_aim PRIVATE include)
target_link_libraries(armor_aim PRIVATE ${OpenCV_LIBS})
