cmake_minimum_required(VERSION 3.10)
project(gripper_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

set(GRIPPER_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/x86")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

foreach(demo position_loop_demo velocity_loop_demo torque_loop_demo zero_demo)
    add_executable(${demo} src/${demo}.cpp)
    target_link_libraries(${demo} PRIVATE
        "-Wl,--no-as-needed"
        "${GRIPPER_LIB_DIR}/libgripper_lib.so"
        "${GRIPPER_LIB_DIR}/libgrip_pawl_lib.so"
        "${GRIPPER_LIB_DIR}/libcan_motor_driver.so"
        Eigen3::Eigen
    )
endforeach()
