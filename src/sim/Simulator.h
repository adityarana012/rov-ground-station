#pragma once

#include "VehicleState.h"

namespace rov {

/// Headless simulation of a small underwater vehicle.
///
/// Phase 0 scope: the public interface, the lifecycle (reset / arm / mode) and
/// the disarm safety interlock. The hydrodynamics, the thruster lag and the
/// hold-mode controllers are added in Phase 1 and Phase 2 behind this same
/// interface, so the GUI built on top of it never has to change.
///
/// Threading: this class is not thread-safe. It is owned and stepped by exactly
/// one driver (the Qt timer in the app layer, or a test harness).
class Simulator {
public:
    /// Fixed simulation timestep. The physics always advances in increments of
    /// this size; the driver may call step() less often and pass a larger dt,
    /// in which case step() sub-steps internally. Decoupling the physics rate
    /// from the render rate keeps the integration stable when a frame is late.
    static constexpr double kFixedStep = 0.02;   // 50 Hz

    /// Manual mode dead-man timeout. If the operator supplies no input for this
    /// long, thrust is driven back to zero.
    static constexpr double kManualInputTimeout = 1.0;   // seconds

    Simulator();

    /// Return the vehicle to its power-on state at the surface, disarmed.
    void reset();

    /// Advance the simulation by \p dt seconds (sub-stepped at kFixedStep).
    void step(double dt);

    // --- Operator commands ---------------------------------------------------

    void setArmed(bool armed);
    void setMode(Mode mode);

    /// Supply a new operator demand. Also resets the dead-man timer, so this
    /// must only be called when the operator actually moved a control.
    void setControlInput(const ControlInput& input);

    void setTargetDepth(double metres);
    void setTargetHeading(double degrees);

    // --- Observation ---------------------------------------------------------

    const VehicleState& state() const { return m_state; }

    /// Seconds since the last setControlInput() call. Exposed for the UI's
    /// dead-man indicator and for the tests.
    double timeSinceInput() const { return m_timeSinceInput; }

private:
    /// Advance exactly one kFixedStep. All physics lives here.
    void integrate(double dt);

    /// Force every thruster output to zero. Called whenever the vehicle is
    /// disarmed and whenever the Manual dead-man timer expires.
    void zeroThrust();

    VehicleState m_state;
    ControlInput m_input;

    double m_timeSinceInput = 0.0;
    double m_stepAccumulator = 0.0;
};

} // namespace rov
