cmake_minimum_required(VERSION 3.24)
project(rmcs_core)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-packed-bitfield-compat")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(FetchContent)
set(BUILD_STATIC_LIBRMCS ON CACHE BOOL "Build static librmcs SDK" FORCE)
FetchContent_Declare(
    librmcs
    URL https://github.com/Alliance-Algorithm/librmcs/releases/download/v3.3.0b1/librmcs-sdk-src-3.3.0-beta.1-debug.zip
    URL_HASH SHA256=446d632b23b652dd309f35075ef90928dee89f42e8ec71417fa2439445e0b4ff
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(librmcs)

file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.c
)

ament_auto_add_library(
    ${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
target_link_libraries(${PROJECT_NAME} librmcs-sdk)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

if(BUILD_TESTING)
    enable_testing()
    foreach(test_name yaw_excitation_test yaw_single_loop_test yaw_accel_feedforward_test)
        ament_auto_add_executable(${test_name} test/${test_name}.cpp)
        # Do not install the hardware-free test fixtures onto the robot.
        list(REMOVE_ITEM ${PROJECT_NAME}_EXECUTABLES ${test_name})
        add_test(NAME ${test_name} COMMAND ${test_name}
            --ros-args --params-file ${CMAKE_CURRENT_SOURCE_DIR}/test/yaw_excitation_test.yaml)
    endforeach()
    foreach(ff_scale 0.0 0.25 0.5 1.0)
        add_test(NAME yaw_accel_feedforward_scale_${ff_scale} COMMAND yaw_accel_feedforward_test
            --ros-args --params-file ${CMAKE_CURRENT_SOURCE_DIR}/test/yaw_excitation_test.yaml
            -p yaw_accel_ff_enabled:=true -p yaw_accel_ff_scale:=${ff_scale})
    endforeach()
endif()

ament_auto_package()
