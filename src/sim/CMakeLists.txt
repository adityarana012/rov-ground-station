# ---------------------------------------------------------------------------
# rovsim - the simulation core
# ---------------------------------------------------------------------------
# IMPORTANT ARCHITECTURAL RULE:
#   This target must NEVER link against Qt and must never include a Qt header.
#   The vehicle model, the control loops and the safety interlocks all live here
#   so that they can be exercised headlessly by the unit tests in tests/ and so
#   that the GUI can be replaced (or driven by real MAVLink telemetry) without
#   touching a line of the physics.
# ---------------------------------------------------------------------------

add_library(rovsim STATIC
    Simulator.cpp
    Simulator.h
    VehicleState.h
)

target_include_directories(rovsim
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(rovsim PUBLIC cxx_std_17)

# No Qt. On purpose. Do not add Qt6::Core here.
