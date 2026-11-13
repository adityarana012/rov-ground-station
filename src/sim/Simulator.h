#pragma once

#include "Pid.h"
#include "VehicleState.h"

namespace rov {

/// Every tunable number in the vehicle model, in one place.
///
/// Values are loosely those of a small observation-class ROV (~11 kg, four
/// horizontal + two vertical thrusters). They are not meant to match a specific
/// vehicle; they are meant to be plausible, stable, and easy to point at when
/// someone asks "where does that behaviour come from?".
struct VehicleParams {
    // --- Inertia -------------------------------------------------------------
    // Added mass is the water the hull drags with it. It is direction-dependent:
    // a blunt body moving sideways entrains far more water than one moving
    // forward, which is why the vehicle feels sluggish when crabbing.
    double massSurge = 11.0 * 1.1;   ///< kg, dry mass x added-mass factor
    double massSway  = 11.0 * 1.4;   ///< kg
    double massHeave = 11.0 * 1.6;   ///< kg
    double yawInertia = 0.28;        ///< kg m^2, about the vertical axis

    // --- Actuators -----------------------------------------------------------
    double maxForceSurge = 50.0;     ///< N at full demand
    double maxForceSway  = 40.0;     ///< N
    double maxForceHeave = 35.0;     ///< N
    double maxTorqueYaw  = 6.0;      ///< N m

    /// First-order thruster response time constant. A step demand reaches
    /// ~63% of its final value after this long. This is the "short
    /// thruster-response delay" the vehicle is required to have.
    double thrusterTau = 0.25;       ///< seconds

    // --- Hydrodynamic drag ---------------------------------------------------
    // Total drag = linear*v + quadratic*v*|v|. The quadratic term dominates at
    // operating speed and is what sets terminal velocity; the linear term keeps
    // the model well behaved near zero, where pure quadratic drag would let the
    // vehicle coast forever.
    double dragLinSurge = 8.0,  dragQuadSurge = 45.0;
    double dragLinSway  = 12.0, dragQuadSway  = 70.0;
    double dragLinHeave = 10.0, dragQuadHeave = 60.0;
    double dragLinYaw   = 2.0,  dragQuadYaw   = 8.0;

    // --- Buoyancy ------------------------------------------------------------
    /// Net upward force with the thrusters off. Real ROVs are trimmed slightly
    /// positive so that a total power failure floats the vehicle rather than
    /// losing it. Here it also gives Depth Hold something real to fight.
    double netBuoyancy = 1.5;        ///< N, upward

    // --- Attitude ------------------------------------------------------------
    // Roll and pitch are not independently actuated. They are a damped
    // second-order response to the forces the vehicle is currently producing -
    // the hull tips under thrust and settles back when it stops.
    //
    // pitchPerHeave: real ROVs with vertical thrusters mounted ahead of the
    // centre of gravity (the common 6-thruster "vectored" layout) tip nose-
    // down when they dive and nose-up when they climb. Without this term,
    // Depth Hold produces a visually dead artificial horizon even during a
    // large depth change, which looks wrong to any operator used to real
    // vehicle footage.
    double pitchPerSurge = -4.0;     ///< degrees of pitch at full surge
    double pitchPerHeave = -3.0;     ///< degrees of pitch at full down-heave
    double rollPerSway   =  5.0;     ///< degrees of roll at full sway
    double rollPerYaw    =  3.0;     ///< degrees of roll at full yaw
    double attitudeFreq  =  3.0;     ///< rad/s, natural frequency
    double attitudeDamp  =  0.6;     ///< damping ratio

    // --- Hold-mode controllers ----------------------------------------------
    // Kp is in output-units per error-unit. For depth that is (thrust fraction)
    // per metre; for heading, (thrust fraction) per degree. The numbers are
    // conservative on purpose - a stable, slightly slow hold is far more
    // valuable to demonstrate than an aggressive one that oscillates.

    double depthKp = 0.35;    ///< ~0.7 thrust fraction at 2 m error
    double depthKi = 0.08;    ///< trims out the steady-state buoyancy offset
    double depthKd = 0.60;    ///< damps the dive/climb rate

    // Heading hold is pure PD on purpose. There is no persistent torque on the
    // vehicle - no current, no thruster mis-trim - so there is no steady-state
    // error to integrate out. Adding Ki here only buys overshoot and a long
    // slow unwind afterwards; it does not improve the eventual accuracy.
    double headingKp = 0.030; ///< ~0.9 thrust fraction at 30 deg error
    double headingKi = 0.000;
    double headingKd = 0.060; ///< acts on yaw rate (deg/s), so small by units

    // --- Battery -------------------------------------------------------------
    double packCapacity   = 18000.0; ///< coulombs (5 Ah)
    double hotelCurrent   = 1.5;     ///< A drawn by electronics alone
    double currentPerUnit = 12.0;    ///< A per unit of summed thrust magnitude
    double cellCountS     = 4.0;     ///< cells in series
    double cellFullVolts  = 4.2;
    double cellEmptyVolts = 3.5;
    double packResistance = 0.03;    ///< ohms, produces sag under load
};

/// Headless simulation of a small underwater vehicle.
///
/// Phase 0 established the lifecycle and the safety interlocks.
/// Phase 1 adds the vehicle dynamics: thruster lag, quadratic drag, buoyancy,
/// attitude response, dead-reckoned position and battery discharge.
/// Phase 2 will add the Depth Hold and Heading Hold controllers behind this
/// same interface.
///
/// Threading: not thread-safe. Owned and stepped by exactly one driver.
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

    explicit Simulator(const VehicleParams& params = VehicleParams{});

    /// Return the vehicle to its power-on state at the surface, disarmed.
    /// Does not change the tuning parameters.
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

    /// Set a constant ocean current. Values are world-frame m/s:
    ///   north   positive-north,
    ///   east    positive-east,
    ///   down    positive-downward (a down-current pushes the vehicle deeper).
    /// The current enters the model as a water velocity: drag acts on the
    /// vehicle's velocity THROUGH the water (unchanged), and the dead-reckoned
    /// world position advances with vehicle-velocity-through-water + current.
    /// A disarmed vehicle therefore drifts with the current and Depth Hold
    /// must work harder when the current is pushing it off target.
    void setCurrent(double north, double east, double down);

    // --- Observation ---------------------------------------------------------

    const VehicleState& state() const { return m_state; }
    const VehicleParams& params() const { return m_params; }

    double currentNorth() const { return m_currentNorth; }
    double currentEast()  const { return m_currentEast; }
    double currentDown()  const { return m_currentDown; }

    /// Seconds since the last setControlInput() call. Exposed for the UI's
    /// dead-man indicator and for the tests.
    double timeSinceInput() const { return m_timeSinceInput; }

    /// Total pack current being drawn right now, in amperes.
    double batteryCurrent() const { return m_batteryCurrent; }

    /// Remaining charge as a fraction in [0, 1].
    double batteryFraction() const { return m_charge / m_params.packCapacity; }

private:
    /// Advance exactly one kFixedStep. All physics lives here.
    void integrate(double dt);

    /// Move the actual thrust one step toward the demand through the
    /// first-order actuator lag.
    void applyThrusterLag(const ControlInput& demand, double dt);

    void integrateTranslation(double dt);
    void integrateYaw(double dt);
    void integrateAttitude(double dt);
    void integrateBattery(double dt);

    /// Force every thruster output to zero. Called whenever the vehicle is
    /// disarmed and whenever the Manual dead-man timer expires.
    void zeroThrust();

    VehicleParams m_params;
    VehicleState m_state;
    ControlInput m_input;

    Pid m_depthPid;
    Pid m_headingPid;

    double m_timeSinceInput = 0.0;
    double m_stepAccumulator = 0.0;

    // Battery bookkeeping, in coulombs and amperes.
    double m_charge = 0.0;
    double m_batteryCurrent = 0.0;

    // Attitude rates, degrees per second. Not part of the published state.
    double m_rollRate = 0.0;
    double m_pitchRate = 0.0;

    // World-frame ocean current. Zero by default.
    double m_currentNorth = 0.0;
    double m_currentEast  = 0.0;
    double m_currentDown  = 0.0;
};

} // namespace rov
