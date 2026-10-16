cmake_minimum_required(VERSION 3.16)
project(SG100_SDK_Publish CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SDK_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(EXAMPLES_DIR ${SDK_ROOT_DIR}/examples)
set(SG100_EXAMPLE_CONFIG ${EXAMPLES_DIR}/serial_config.json)
set(SG100_JOINT_ANGLE_MAP ${SDK_ROOT_DIR}/joint_angle_map.json)
set(SG100_JOINT_ANGLE_MAP_V3 ${SDK_ROOT_DIR}/joint_angle_map_v3.0.json)

include_directories(
    ${SDK_ROOT_DIR}/include
    ${SDK_ROOT_DIR}/include/serial
    ${SDK_ROOT_DIR}/include/serial/impl
)

# ========== 平台检测 ==========
if(WIN32)
    if(MSVC)
        set(PLATFORM_DIR "win-msvc")
        set(SG100_DLL_NAME "sg100_sdk.dll")
    elseif(MINGW)
        set(PLATFORM_DIR "win-mingw32")
        set(SG100_DLL_NAME "libsg100_sdk.dll")
    else()
        message(FATAL_ERROR "Windows 平台仅支持 MSVC 或 MinGW")
    endif()
elseif(UNIX AND NOT APPLE)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(PLATFORM_DIR "linux-arm64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
        set(PLATFORM_DIR "linux-x86_64")
    else()
        set(PLATFORM_DIR "linux-${CMAKE_SYSTEM_PROCESSOR}")
    endif()
elseif(APPLE)
    if(CMAKE_OSX_ARCHITECTURES MATCHES "arm64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
        set(PLATFORM_DIR "macos-arm64")
    else()
        set(PLATFORM_DIR "macos-x86_64")
    endif()
else()
    message(FATAL_ERROR "不支持的平台")
endif()

set(LIB_DIR ${SDK_ROOT_DIR}/lib/${PLATFORM_DIR})

# ========== 查找库文件 ==========
if(MSVC)
    find_library(SG100_SDK_LIB
        NAMES sg100_sdk
        PATHS ${LIB_DIR}
        NO_DEFAULT_PATH
    )
    set(SG100_SDK_DLL ${LIB_DIR}/${SG100_DLL_NAME})
elseif(MINGW)
    find_library(SG100_SDK_LIB
        NAMES sg100_sdk libsg100_sdk.dll
        PATHS ${LIB_DIR}
        NO_DEFAULT_PATH
    )
    set(SG100_SDK_DLL ${LIB_DIR}/${SG100_DLL_NAME})
elseif(UNIX)
    find_library(SG100_SDK_LIB
        NAMES sg100_sdk
        PATHS ${LIB_DIR}
        NO_DEFAULT_PATH
    )
endif()

if(NOT SG100_SDK_LIB)
    message(FATAL_ERROR "找不到 SG100 SDK 库文件！\n"
                        "平台: ${PLATFORM_DIR}\n"
                        "搜索路径: ${LIB_DIR}\n"
                        "请确保库文件存在于: ${SDK_ROOT_DIR}/lib/${PLATFORM_DIR}/")
endif()

message(STATUS "========================================")
message(STATUS "SG100 SDK Publish 配置")
message(STATUS "========================================")
message(STATUS "平台目录: ${PLATFORM_DIR}")
message(STATUS "编译器: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "C++标准: ${CMAKE_CXX_STANDARD}")
message(STATUS "SDK根目录: ${SDK_ROOT_DIR}")
message(STATUS "示例目录: ${EXAMPLES_DIR}")
message(STATUS "库目录: ${LIB_DIR}")
message(STATUS "SDK库文件: ${SG100_SDK_LIB}")
if(WIN32)
    message(STATUS "DLL文件: ${SG100_SDK_DLL}")
endif()
message(STATUS "========================================")

# ========== 编译选项 ==========
if(MSVC)
    add_compile_options(/utf-8 /W3)
    add_compile_options(
        /wd4251
        /wd4275
        /wd4819
        /wd4459
    )
    add_definitions(-DUNICODE -D_UNICODE)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    add_compile_options(-finput-charset=UTF-8 -fexec-charset=UTF-8)
endif()

set(SG100_RUNTIME_DIR ${CMAKE_BINARY_DIR}/bin)
set(SG100_DEMO_TARGETS)

add_custom_target(sg100_runtime_assets ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory "${SG100_RUNTIME_DIR}"
    COMMENT "准备运行时资源目录"
    VERBATIM
)

if(EXISTS ${SG100_EXAMPLE_CONFIG})
    add_custom_command(TARGET sg100_runtime_assets POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SG100_EXAMPLE_CONFIG}"
        "${SG100_RUNTIME_DIR}/serial_config.json"
        COMMENT "复制 serial_config.json"
        VERBATIM
    )
endif()

if(EXISTS ${SG100_JOINT_ANGLE_MAP})
    add_custom_command(TARGET sg100_runtime_assets POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SG100_JOINT_ANGLE_MAP}"
        "${SG100_RUNTIME_DIR}/joint_angle_map.json"
        COMMENT "复制 joint_angle_map.json"
        VERBATIM
    )
endif()

if(EXISTS ${SG100_JOINT_ANGLE_MAP_V3})
    add_custom_command(TARGET sg100_runtime_assets POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SG100_JOINT_ANGLE_MAP_V3}"
        "${SG100_RUNTIME_DIR}/joint_angle_map_v3.0.json"
        COMMENT "复制 joint_angle_map_v3.0.json"
        VERBATIM
    )
endif()

if(WIN32 AND EXISTS ${SG100_SDK_DLL})
    add_custom_command(TARGET sg100_runtime_assets POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SG100_SDK_DLL}"
        "${SG100_RUNTIME_DIR}/"
        COMMENT "复制 SDK 动态库到 ${SG100_RUNTIME_DIR}"
        VERBATIM
    )
elseif(UNIX)
    add_custom_command(TARGET sg100_runtime_assets POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SG100_SDK_LIB}"
        "${SG100_RUNTIME_DIR}/"
        COMMENT "复制 libsg100_sdk 到 ${SG100_RUNTIME_DIR}"
        VERBATIM
    )
endif()

function(sg100_add_demo target_name source_file)
    if(NOT EXISTS ${source_file})
        message(WARNING "跳过示例 ${target_name}: ${source_file} 不存在")
        return()
    endif()

    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} ${SG100_SDK_LIB})
    add_dependencies(${target_name} sg100_runtime_assets)

    if(UNIX)
        target_link_libraries(${target_name} pthread)
    endif()

    set_target_properties(${target_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${SG100_RUNTIME_DIR}
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${SG100_RUNTIME_DIR}
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${SG100_RUNTIME_DIR}
    )

    if(APPLE)
        set_target_properties(${target_name} PROPERTIES
            INSTALL_RPATH "@loader_path"
            BUILD_RPATH "@loader_path"
            BUILD_WITH_INSTALL_RPATH TRUE
        )
    elseif(UNIX)
        set_target_properties(${target_name} PROPERTIES
            INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib"
            BUILD_WITH_INSTALL_RPATH TRUE
        )
    endif()

    list(APPEND SG100_DEMO_TARGETS ${target_name})
    set(SG100_DEMO_TARGETS ${SG100_DEMO_TARGETS} PARENT_SCOPE)
endfunction()

# ========== 示例程序 ==========
sg100_add_demo(hand_control_example ${EXAMPLES_DIR}/hand_control_example.cpp)
sg100_add_demo(ota_upgrade_example ${EXAMPLES_DIR}/ota_upgrade_example.cpp)
sg100_add_demo(comm_status_test ${EXAMPLES_DIR}/comm_status_test.cpp)
sg100_add_demo(homing_and_realtime_control_example ${EXAMPLES_DIR}/homing_and_realtime_control_example.cpp)
sg100_add_demo(realtime_joint_control_example ${EXAMPLES_DIR}/realtime_joint_control_example.cpp)
sg100_add_demo(interpolated_joint_motion_example ${EXAMPLES_DIR}/interpolated_joint_motion_example.cpp)
sg100_add_demo(palm_runtime_mode_example ${EXAMPLES_DIR}/palm_runtime_mode_example.cpp)
sg100_add_demo(impedance_control_example ${EXAMPLES_DIR}/impedance_control_example.cpp)
sg100_add_demo(impedance_position_sin_test ${EXAMPLES_DIR}/impedance_position_sin_test.cpp)
sg100_add_demo(teaching_mode_example ${EXAMPLES_DIR}/teaching_mode_example.cpp)

if(SG100_DEMO_TARGETS)
    install(TARGETS ${SG100_DEMO_TARGETS} RUNTIME DESTINATION bin)
endif()

if(EXISTS ${SG100_EXAMPLE_CONFIG})
    install(FILES "${SG100_EXAMPLE_CONFIG}" DESTINATION bin)
endif()

if(EXISTS ${SG100_JOINT_ANGLE_MAP})
    install(FILES "${SG100_JOINT_ANGLE_MAP}" DESTINATION .)
endif()
if(EXISTS ${SG100_JOINT_ANGLE_MAP_V3})
    install(FILES "${SG100_JOINT_ANGLE_MAP_V3}" DESTINATION .)
endif()

if(WIN32 AND EXISTS ${SG100_SDK_DLL})
    install(FILES ${SG100_SDK_DLL} DESTINATION bin)
elseif(UNIX)
    install(FILES "${SG100_SDK_LIB}" DESTINATION lib)
endif()

message(STATUS "")
message(STATUS "构建目标: ${SG100_DEMO_TARGETS}")
message(STATUS "输出目录: ${SG100_RUNTIME_DIR}")
message(STATUS "========================================")
