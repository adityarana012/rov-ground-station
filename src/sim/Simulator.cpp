#include "Simulator.h"

#include <algorithm>
#include <cmath>

namespace rov {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegPerRad = 180.0 / kPi;
constexpr double kRadPerDeg = kPi / 180.0;

/// Drag force opposing motion: linear term + quadratic term.
/// Returns a force with the sign already flipped to oppose \p v.
double dragForce(double v, double linear, double quadratic)
{
    return -(linear * v + quadratic * v * std::fabs(v));
}

/// Move \p current toward \p target through a first-order lag of time
/// constant \p tau. Written as an exact exponential rather than
/// `current += (target - current) * dt / tau` so that the response is
/// identical regardless of step size and cannot overshoot if dt > tau.
double firstOrderLag(double current, double target, double tau, double dt)
{
    if (tau <= 0.0)
        return target;
    const double alpha = 1.0 - std::exp(-dt / tau);
    return current + (target - current) * alpha;
}

/// Normalise degrees into [0, 360).
double wrap360(double degrees)
{
    degrees = std::fmod(degrees, 360.0);
    if (degrees < 0.0)
        degrees += 360.0;
    return degrees;
}

} // namespace

Simulator::Simulator(const VehicleParams& params)
    : m_params(params)
{
    m_depthPid.setGains(m_params.depthKp, m_params.depthKi, m_params.depthKd);
    m_depthPid.setOutputLimits(-1.0, 1.0);

    m_headingPid.setGains(m_params.headingKp, m_params.headingKi, m_params.headingKd);
    m_headingPid.setOutputLimits(-1.0, 1.0);

    reset();
}

void Simulator::reset()
{
    m_state = VehicleState{};
    m_input.clear();
    m_timeSinceInput = 0.0;
    m_stepAccumulator = 0.0;
    m_rollRate = 0.0;
    m_pitchRate = 0.0;

    m_depthPid.reset();
    m_headingPid.reset();

    m_charge = m_params.packCapacity;
    m_batteryCurrent = 0.0;
    m_state.batteryVolts = m_params.cellCountS * m_params.cellFullVolts;
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

    // Reset the controller integrators on every mode change. Carrying an
    // integrator across modes means the vehicle inherits accumulated error it
    // was never supposed to answer for, and the first second after a mode
    // switch is spent discharging it with full thrust the wrong way.
    m_depthPid.reset();
    m_headingPid.reset();
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
    m_state.targetHeading = wrap360(degrees);
}

void Simulator::setCurrent(double north, double east, double down)
{
    m_currentNorth = north;
    m_currentEast  = east;
    m_currentDown  = down;
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

    // 1. In Manual mode, demand returns to zero after one second without input.
    //    Checked before the arm test so the dead-man timer keeps running and
    //    the state is consistent either way.
    if (m_state.mode == Mode::Manual && m_timeSinceInput >= kManualInputTimeout)
        m_input.clear();

    // 2. Thrust is zero whenever the vehicle is disarmed. Note that the vehicle
    //    still moves afterwards: it coasts, it is dragged to a stop, and its
    //    positive buoyancy floats it up. Disarmed means no thrust, not frozen.
    ControlInput demand = m_input;
    if (!m_state.armed)
        demand.clear();

    // ---- Hold-mode controllers ---------------------------------------------
    // Each mode overrides exactly one axis of the operator demand. The others
    // remain under manual control - Depth Hold does NOT take the vehicle fully
    // autonomous, it just takes the vertical axis off the operator's hands.

    if (m_state.armed) {
        if (m_state.mode == Mode::DepthHold) {
            // Depth is positive downward; heave is positive downward.
            // error = target - depth is positive when the vehicle is too shallow,
            // calling for +heave (downward) to dive.
            const double error = m_state.targetDepth - m_state.depth;
            demand.heave = m_depthPid.update(error, m_state.heaveVel, dt);
        } else if (m_state.mode == Mode::HeadingHold) {
            // Wrap the error into [-180, 180) so the controller always turns
            // the short way around the compass. This is the 359 -> 1 boundary
            // test that interviewers love to ask about.
            const double rawError = m_state.targetHeading - m_state.yaw;
            const double error = wrapError180(rawError);
            demand.yaw = m_headingPid.update(error, m_state.yawRate, dt);
        }
    }

    applyThrusterLag(demand, dt);

    if (!m_state.armed)
        zeroThrust();   // belt and braces: the lag must never leak thrust

    integrateTranslation(dt);
    integrateYaw(dt);
    integrateAttitude(dt);
    integrateBattery(dt);
}

void Simulator::applyThrusterLag(const ControlInput& demand, double dt)
{
    const double tau = m_params.thrusterTau;
    m_state.surgeThrust = firstOrderLag(m_state.surgeThrust, demand.surge, tau, dt);
    m_state.swayThrust  = firstOrderLag(m_state.swayThrust,  demand.sway,  tau, dt);
    m_state.heaveThrust = firstOrderLag(m_state.heaveThrust, demand.heave, tau, dt);
    m_state.yawThrust   = firstOrderLag(m_state.yawThrust,   demand.yaw,   tau, dt);
}

void Simulator::integrateTranslation(double dt)
{
    const VehicleParams& p = m_params;

    // --- Body-frame accelerations -------------------------------------------
    const double fSurge = m_state.surgeThrust * p.maxForceSurge
                        + dragForce(m_state.surgeVel, p.dragLinSurge, p.dragQuadSurge);

    const double fSway  = m_state.swayThrust * p.maxForceSway
                        + dragForce(m_state.swayVel, p.dragLinSway, p.dragQuadSway);

    // Heave is positive downward, so positive buoyancy is a negative force.
    const double fHeave = m_state.heaveThrust * p.maxForceHeave
                        + dragForce(m_state.heaveVel, p.dragLinHeave, p.dragQuadHeave)
                        - p.netBuoyancy;

    m_state.surgeVel += (fSurge / p.massSurge) * dt;
    m_state.swayVel  += (fSway  / p.massSway)  * dt;
    m_state.heaveVel += (fHeave / p.massHeave) * dt;

    // --- World-frame motion --------------------------------------------------
    // The body-frame velocities above are the vehicle's motion THROUGH the
    // water - that is what drag acts on. The water itself is moving (the ocean
    // current), and the vehicle's ground track is water-relative + current.
    //
    // This is why Depth Hold has to work against a vertical current and why a
    // disarmed vehicle drifts horizontally in a crosscurrent even with zero
    // body-frame velocity.
    const double yawRad = m_state.yaw * kRadPerDeg;
    const double c = std::cos(yawRad);
    const double s = std::sin(yawRad);

    const double vNorth = m_state.surgeVel * c - m_state.swayVel * s
                        + m_currentNorth;
    const double vEast  = m_state.surgeVel * s + m_state.swayVel * c
                        + m_currentEast;
    const double vDown  = m_state.heaveVel + m_currentDown;

    m_state.north += vNorth * dt;
    m_state.east  += vEast  * dt;

    // --- Depth and surface clamp --------------------------------------------
    m_state.depth += vDown * dt;
    if (m_state.depth <= 0.0) {
        // The surface is a hard floor at zero. Upward motion is absorbed
        // there - whatever is left of heaveVel + currentDown that points up is
        // cancelled by zeroing the body-frame heave velocity.
        m_state.depth = 0.0;
        if (m_state.heaveVel + m_currentDown < 0.0)
            m_state.heaveVel = -m_currentDown;
    }
}

void Simulator::integrateYaw(double dt)
{
    const VehicleParams& p = m_params;

    // Work in radians internally; publish degrees.
    double omega = m_state.yawRate * kRadPerDeg;

    const double torque = m_state.yawThrust * p.maxTorqueYaw
                        + dragForce(omega, p.dragLinYaw, p.dragQuadYaw);

    omega += (torque / p.yawInertia) * dt;

    m_state.yawRate = omega * kDegPerRad;
    m_state.yaw = wrap360(m_state.yaw + m_state.yawRate * dt);
}

void Simulator::integrateAttitude(double dt)
{
    const VehicleParams& p = m_params;

    // Where the hull wants to sit given what the thrusters are currently doing.
    const double pitchTarget = p.pitchPerSurge * m_state.surgeThrust
                             + p.pitchPerHeave * m_state.heaveThrust;
    const double rollTarget  = p.rollPerSway * m_state.swayThrust
                             + p.rollPerYaw  * m_state.yawThrust;

    // Damped second-order response toward that attitude.
    const double wn = p.attitudeFreq;
    const double zeta = p.attitudeDamp;

    m_pitchRate += (wn * wn * (pitchTarget - m_state.pitch)
                    - 2.0 * zeta * wn * m_pitchRate) * dt;
    m_state.pitch += m_pitchRate * dt;

    m_rollRate += (wn * wn * (rollTarget - m_state.roll)
                   - 2.0 * zeta * wn * m_rollRate) * dt;
    m_state.roll += m_rollRate * dt;
}

void Simulator::integrateBattery(double dt)
{
    const VehicleParams& p = m_params;

    const double thrustSum = std::fabs(m_state.surgeThrust)
                           + std::fabs(m_state.swayThrust)
                           + std::fabs(m_state.heaveThrust)
                           + std::fabs(m_state.yawThrust);

    m_batteryCurrent = p.hotelCurrent + p.currentPerUnit * thrustSum;

    m_charge = std::max(0.0, m_charge - m_batteryCurrent * dt);

    // Open-circuit voltage falls linearly with state of charge, then sags
    // further under load through the pack's internal resistance. Crude, but it
    // reproduces the two things an operator actually watches for: a pack that
    // drains faster when working hard, and a needle that dips on every burst of
    // thrust and recovers when the thrusters ease off.
    const double soc = batteryFraction();
    const double openCircuit = p.cellCountS
        * (p.cellEmptyVolts + (p.cellFullVolts - p.cellEmptyVolts) * soc);

    m_state.batteryVolts = openCircuit - m_batteryCurrent * p.packResistance;
}

void Simulator::zeroThrust()
{
    m_state.surgeThrust = 0.0;
    m_state.swayThrust  = 0.0;
    m_state.heaveThrust = 0.0;
    m_state.yawThrust   = 0.0;
}

} // namespace rov
