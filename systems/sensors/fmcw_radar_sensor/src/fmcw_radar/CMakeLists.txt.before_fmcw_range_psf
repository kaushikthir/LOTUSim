cmake_minimum_required(VERSION 3.16)

project(
    fmcw_radar
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(
    nlohmann_json REQUIRED
)


include(
    GNUInstallDirs
)

include(
    CMakePackageConfigHelpers
)


# ============================================================
# RADAR PACKAGE
# ============================================================

add_library(
    fmcw_radar
    SHARED

    src/fmcw_radar.cpp

    src/config/ConfigLoader.cpp

    src/math/RadarMath.cpp

    src/factory/ModelFactory.cpp

    src/scan/ScanController.cpp
    src/scan/BeamGeometry.cpp

    src/response/RadarReturnModel.cpp

    src/output/RangeBinner.cpp

    src/debug/DebugRecorder.cpp

    src/antenna/SincAntennaPattern.cpp

    src/propagation/FreeSpacePropagation.cpp

    src/scattering/ConstantRcsScatter.cpp

    src/sampling/GridBeamSampler.cpp

    src/noise/NoNoise.cpp
)


add_library(
    fmcw_radar::fmcw_radar
    ALIAS
    fmcw_radar
)


target_include_directories(
    fmcw_radar

    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>

    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)


target_link_libraries(
    fmcw_radar

    PRIVATE
        nlohmann_json::nlohmann_json
)


# ============================================================
# INSTALL
# ============================================================

install(
    TARGETS fmcw_radar

    EXPORT fmcw_radarTargets

    ARCHIVE DESTINATION
        ${CMAKE_INSTALL_LIBDIR}

    LIBRARY DESTINATION
        ${CMAKE_INSTALL_LIBDIR}

    RUNTIME DESTINATION
        ${CMAKE_INSTALL_BINDIR}

    INCLUDES DESTINATION
        ${CMAKE_INSTALL_INCLUDEDIR}
)


install(
    DIRECTORY include/

    DESTINATION
        ${CMAKE_INSTALL_INCLUDEDIR}
)


install(
    EXPORT fmcw_radarTargets

    FILE
        fmcw_radarTargets.cmake

    NAMESPACE
        fmcw_radar::

    DESTINATION
        ${CMAKE_INSTALL_LIBDIR}/cmake/fmcw_radar
)


configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/fmcw_radarConfig.cmake.in

    ${CMAKE_CURRENT_BINARY_DIR}/fmcw_radarConfig.cmake

    INSTALL_DESTINATION
        ${CMAKE_INSTALL_LIBDIR}/cmake/fmcw_radar
)


write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/fmcw_radarConfigVersion.cmake

    VERSION
        ${PROJECT_VERSION}

    COMPATIBILITY
        SameMajorVersion
)


install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/fmcw_radarConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/fmcw_radarConfigVersion.cmake

    DESTINATION
        ${CMAKE_INSTALL_LIBDIR}/cmake/fmcw_radar
)
