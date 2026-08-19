cmake_minimum_required(VERSION 3.16)

project(fmcw_radar_sensor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lotusim_sensor_base REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_subdirectory(src/fmcw_radar)

if(NOT TARGET fmcw_radar::fmcw_radar)
    message(FATAL_ERROR
        "The internal FMCW model did not create "
        "the target fmcw_radar::fmcw_radar")
endif()

add_library(${PROJECT_NAME} SHARED
    src/fmcw_radar_sensor.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    std_msgs
    lotusim_sensor_base
)

target_link_libraries(${PROJECT_NAME}
    fmcw_radar::fmcw_radar
    gz-sim8::gz-sim8
    Threads::Threads
)

add_executable(fmcw_radar_marine_ppi
    src/fmcw_radar_marine_ppi.cpp
)

ament_target_dependencies(fmcw_radar_marine_ppi
    rclcpp
    std_msgs
)

target_link_libraries(fmcw_radar_marine_ppi
    OpenGL::GL
    glfw
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    TARGETS 
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS fmcw_radar_marine_ppi
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY src/fmcw_radar/configs/
    DESTINATION share/${PROJECT_NAME}/configs
)

ament_export_targets(
    export_${PROJECT_NAME}
    HAS_LIBRARY_TARGET
)

ament_export_dependencies(
    rclcpp
    sensor_msgs
    std_msgs
    lotusim_sensor_base
    fmcw_radar
    gz-sim8
)

ament_package()
