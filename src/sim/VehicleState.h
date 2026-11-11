#pragma once

namespace rov {

/// Operating mode of the vehicle.
///
/// Manual       - operator command vector is applied directly to the thrusters.
/// DepthHold    - vertical axis is closed-loop on a target depth; the operator
///                retains manual authority over the horizontal axes.
/// HeadingHold  - yaw axis is closed-loop on a target heading; the operator
///                retains manual authority over the remaining axes.
enum class Mode {
    Manual = 0,
    DepthHold = 1,
    HeadingHold = 2
};

/// Normalised operator demand, each component in [-1, 1].
///
/// These are *demands*, not thrusts. The simulator passes them through the
/// thruster first-order lag before they become real force, which is what
/// produces the "short thruster-response delay" the vehicle is supposed to have.
struct ControlInput {
    double surge = 0.0;  ///< forward (+) / backward (-)
    double sway  = 0.0;  ///< right (+)   / left (-)
    double heave = 0.0;  ///< down (+)    / up (-)
    double yaw   = 0.0;  ///< clockwise (+) / counter-clockwise (-)

    void clear() { surge = sway = heave = yaw = 0.0; }
};

/// Full observable state of the simulated vehicle at one instant.
///
/// This is the single struct the UI layer reads. Nothing in the GUI is allowed
/// to reach past it into the simulator internals.
struct VehicleState {
    // --- Position / attitude -------------------------------------------------
    double north = 0.0;        ///< metres from the launch origin, +north
    double east  = 0.0;        ///< metres from the launch origin, +east
    double depth = 0.0;        ///< metres below the surface, always >= 0

    double roll  = 0.0;        ///< degrees, +starboard down
    double pitch = 0.0;        ///< degrees, +nose up
    double yaw   = 0.0;        ///< degrees, [0, 360), 0 = north

    // --- Velocities (body frame) --------------------------------------------
    double surgeVel = 0.0;     ///< m/s
    double swayVel  = 0.0;     ///< m/s
    double heaveVel = 0.0;     ///< m/s
    double yawRate  = 0.0;     ///< deg/s

    // --- Actuator state ------------------------------------------------------
    // The *actual* thrust currently being produced, after the response lag.
    double surgeThrust = 0.0;  ///< [-1, 1]
    double swayThrust  = 0.0;  ///< [-1, 1]
    double heaveThrust = 0.0;  ///< [-1, 1]
    double yawThrust   = 0.0;  ///< [-1, 1]

    // --- Housekeeping --------------------------------------------------------
    bool   armed = false;
    Mode   mode = Mode::Manual;
    double batteryVolts = 16.8;   ///< 4S Li-ion pack, full
    double missionTime = 0.0;     ///< seconds since reset

    double targetDepth = 0.0;     ///< metres, used in DepthHold
    double targetHeading = 0.0;   ///< degrees, used in HeadingHold
};

} // namespace rov
