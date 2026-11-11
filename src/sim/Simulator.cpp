#include "Simulator.h"

#include <algorithm>

namespace rov {

Simulator::Simulator()
{
    reset();
}

void Simulator::reset()
{
    m_state = VehicleState{};
    m_input.clear();
    m_timeSinceInput = 0.0;
    m_stepAccumulator = 0.0;
}

void Simulator::setArmed(bool armed)
{
    if (m_state.armed == armed)
        return;

    m_state.armed = armed;

    // Disarming is immediate and unconditional - it does not wait for the next
    // integration step.
    if (!armed) {
        m_input.clear();
        zeroThrust();
    }
}

void Simulator::setMode(Mode mode)
{
    if (m_state.mode == mode)
        return;

    m_state.mode = mode;

    // Phase 2 will also reset the controller integrators here, so that a mode
    // switch never inherits accumulated error from the previous mode.
}

void Simulator::setControlInput(const ControlInput& input)
{
    m_input = input;
    m_input.surge = std::clamp(m_input.surge, -1.0, 1.0);
    m_input.sway  = std::clamp(m_input.sway,  -1.0, 1.0);
    m_input.heave = std::clamp(m_input.heave, -1.0, 1.0);
    m_input.yaw   = std::clamp(m_input.yaw,   -1.0, 1.0);

    m_timeSinceInput = 0.0;
}

void Simulator::setTargetDepth(double metres)
{
    m_state.targetDepth = std::max(0.0, metres);
}

void Simulator::setTargetHeading(double degrees)
{
    // Normalise into [0, 360) so the UI can hand us anything.
    while (degrees < 0.0)    degrees += 360.0;
    while (degrees >= 360.0) degrees -= 360.0;
    m_state.targetHeading = degrees;
}

void Simulator::step(double dt)
{
    if (dt <= 0.0)
        return;

    // Guard against a stalled driver handing us a huge dt (debugger breakpoint,
    // machine sleep). Better to lose simulated time than to explode the
    // integrator.
    dt = std::min(dt, 0.25);

    m_stepAccumulator += dt;

    // The epsilon matters. Without it, an accumulator built from values that are
    // not exactly representable in binary (0.1 s of dt against a 0.02 s step)
    // lands a hair below the step size and silently drops a tick every time.
    constexpr double kEpsilon = 1e-9;
    while (m_stepAccumulator + kEpsilon >= kFixedStep) {
        integrate(kFixedStep);
        m_stepAccumulator -= kFixedStep;
    }
}

void Simulator::integrate(double dt)
{
    m_state.missionTime += dt;
    m_timeSinceInput += dt;

    // ---- Safety interlocks -------------------------------------------------
    // These are enforced here, inside the simulation core, rather than in the
    // UI. A bug, a stuck key or a future control source cannot route around
    // them.

    // 1. Thrust is zero whenever the vehicle is disarmed.
    if (!m_state.armed) {
        zeroThrust();
        return;
    }

    // 2. In Manual mode, thrust returns to zero after one second without input.
    if (m_state.mode == Mode::Manual && m_timeSinceInput >= kManualInputTimeout) {
        m_input.clear();
    }

    // ---- Phase 1: hydrodynamics + thruster lag -----------------------------
    // ---- Phase 2: depth-hold and heading-hold controllers ------------------
    // Not yet implemented. The vehicle currently holds station.
}

void Simulator::zeroThrust()
{
    m_state.surgeThrust = 0.0;
    m_state.swayThrust  = 0.0;
    m_state.heaveThrust = 0.0;
    m_state.yawThrust   = 0.0;
}

} // namespace rov
