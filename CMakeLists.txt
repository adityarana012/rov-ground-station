cmake_minimum_required(VERSION 3.21)

project(RovGroundStation
    VERSION 0.1.0
    DESCRIPTION "ROV Telemetry Ground Station - native Qt6/QML desktop application"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Put binaries somewhere predictable regardless of generator.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(ROV_BUILD_TESTS "Build the headless simulation unit tests" ON)

# ---------------------------------------------------------------------------
# Qt
# ---------------------------------------------------------------------------
# Only the GUI layer depends on Qt. The simulation core (src/sim) is deliberately
# Qt-free so it can be compiled and tested without a display or an event loop.
find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Quick QuickControls2)

qt_standard_project_setup(REQUIRES 6.5)

# ---------------------------------------------------------------------------
# Targets
# ---------------------------------------------------------------------------
add_subdirectory(src/sim)
add_subdirectory(src/app)

if(ROV_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# Configuration summary
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "  ${PROJECT_NAME} ${PROJECT_VERSION}")
message(STATUS "  Qt version ......... ${Qt6_VERSION}")
message(STATUS "  Build type ......... ${CMAKE_BUILD_TYPE}")
message(STATUS "  Unit tests ......... ${ROV_BUILD_TESTS}")
message(STATUS "")
