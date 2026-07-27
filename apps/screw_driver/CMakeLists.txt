cmake_minimum_required(VERSION 3.16)
project(screw_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(USE_OPENARM_CAN "Build with real openarm_can backend" OFF)
set(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(THIRD_PARTY_ROOT "${PROJECT_ROOT}/third_party")
set(OPENARM_CAN_ROOT "${THIRD_PARTY_ROOT}/openarm_can-1.1.0")
find_package(Threads REQUIRED)

add_library(screw_driver_core
    src/screw_driver.cpp
)
target_include_directories(screw_driver_core PUBLIC include)
target_include_directories(screw_driver_core PUBLIC
    ${THIRD_PARTY_ROOT}
)

if (USE_OPENARM_CAN)
    if (NOT EXISTS "${OPENARM_CAN_ROOT}/CMakeLists.txt")
        message(FATAL_ERROR "Vendored openarm_can path is missing")
    endif()
    add_subdirectory("${OPENARM_CAN_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/openarm_can")
    target_compile_definitions(screw_driver_core PUBLIC USE_OPENARM_CAN=1)
    target_include_directories(screw_driver_core PUBLIC
        ${OPENARM_CAN_ROOT}/include
    )
    target_link_libraries(screw_driver_core PUBLIC
        openarm_can
    )
endif()

add_executable(screw_server
    src/screw_server.cpp
)
target_link_libraries(screw_server PRIVATE screw_driver_core)
target_link_libraries(screw_server PRIVATE Threads::Threads)

add_executable(screw_cli
    src/screw_cli.cpp
)
target_link_libraries(screw_cli PRIVATE screw_driver_core)
target_link_libraries(screw_cli PRIVATE Threads::Threads)

add_executable(screw_monitor
    src/screw_monitor.cpp
)
target_link_libraries(screw_monitor PRIVATE screw_driver_core)
target_link_libraries(screw_monitor PRIVATE Threads::Threads)

if (WIN32)
    target_link_libraries(screw_server PRIVATE ws2_32)
endif()
#[[ PROJECT FILE HEADER
文件：apps/screw_driver/CMakeLists.txt
作用：配置 C++ 电批服务、命令行工具和监控程序的构建目标。
用法：在独立构建目录执行 cmake ../apps/screw_driver && cmake --build .。
注意：不要把 build/、CMakeFiles/ 等构建产物写入源码目录。
END PROJECT FILE HEADER ]]
