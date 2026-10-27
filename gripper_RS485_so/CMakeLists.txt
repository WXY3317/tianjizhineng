cmake_minimum_required(VERSION 3.10)
project(gripper_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    set(GRIPPER_LIB_SUBDIR "lib/arm")
else()
    set(GRIPPER_LIB_SUBDIR "lib/x86")
endif()
set(GRIPPER_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${GRIPPER_LIB_SUBDIR}")

# $ORIGIN 是程序所在目录，与启动时的工作目录无关。
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../${GRIPPER_LIB_SUBDIR}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(gripper_tools_lib STATIC src/gripper_tools.cpp)
target_link_libraries(gripper_tools_lib PUBLIC
    "${GRIPPER_LIB_DIR}/libgripper_lib.so"
    Eigen3::Eigen
)

foreach(demo position_loop_demo velocity_loop_demo torque_loop_demo zero_demo)
    add_executable(${demo} src/${demo}.cpp)
    target_link_libraries(${demo} PRIVATE
        "-Wl,--no-as-needed"
        gripper_tools_lib
        "${GRIPPER_LIB_DIR}/libgripper_lib.so"
        "${GRIPPER_LIB_DIR}/libgrip_pawl_lib.so"
        "${GRIPPER_LIB_DIR}/libcan_motor_driver.so"
        Eigen3::Eigen
    )
endforeach()
