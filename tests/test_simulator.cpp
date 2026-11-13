// Minimal dependency-free test harness. Deliberately not GoogleTest or Qt Test:
// the simulation core has no third-party dependencies and the tests should not
// introduce one.

#include "CsvLogger.h"
#include "Pid.h"
#include "Simulator.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool condition, const std::string& what)
{
    ++g_checks;
    if (!condition) {
        ++g_failures;
        std::printf("  FAIL  %s\n", what.c_str());
    } else {
        std::printf("  ok    %s\n", what.c_str());
    }
}

void checkNear(double actual, double expected, double tol, const std::string& what)
{
    check(std::fabs(actual - expected) <= tol,
          what + "  (got " + std::to_string(actual) + ", expected "
               + std::to_string(expected) + ")");
}

void section(const char* name)
{
    std::printf("\n%s\n", name);
}

/// Drive the simulator for \p seconds of simulated time at the fixed rate,
/// with no operator input.
void run(rov::Simulator& sim, double seconds)
{
    const int steps = static_cast<int>(seconds / rov::Simulator::kFixedStep);
    for (int i = 0; i < steps; ++i)
        sim.step(rov::Simulator::kFixedStep);
}

/// Drive the simulator while the operator holds \p input, re-asserting the
/// demand on every tick so the dead-man timer never expires.
void runHolding(rov::Simulator& sim, double seconds, const rov::ControlInput& input)
{
    const int steps = static_cast<int>(seconds / rov::Simulator::kFixedStep);
    for (int i = 0; i < steps; ++i) {
        sim.setControlInput(input);
        sim.step(rov::Simulator::kFixedStep);
    }
}

} // namespace

int main()
{
    using namespace rov;

    // =======================================================================
    // Phase 0 - lifecycle and safety
    // =======================================================================

    section("power-on state");
    {
        Simulator sim;
        const auto& s = sim.state();
        check(!s.armed, "vehicle starts disarmed");
        check(s.mode == Mode::Manual, "vehicle starts in Manual mode");
        checkNear(s.depth, 0.0, 1e-9, "starts at the surface");
        checkNear(s.batteryVolts, 16.8, 1e-9, "battery starts full");
        checkNear(s.missionTime, 0.0, 1e-9, "mission clock starts at zero");
        checkNear(sim.batteryFraction(), 1.0, 1e-9, "pack starts at 100%");
    }

    section("fixed-step integration");
    {
        Simulator sim;
        run(sim, 1.0);
        checkNear(sim.state().missionTime, 1.0, 1e-6,
                  "one second of stepping advances the clock by one second");

        Simulator sim2;
        sim2.step(0.1);
        checkNear(sim2.state().missionTime, 0.1, 1e-6,
                  "a 100 ms dt is sub-stepped into five 20 ms steps");

        Simulator sim3;
        sim3.step(10.0);
        check(sim3.state().missionTime <= 0.25 + 1e-6,
              "a stalled-driver dt is clamped instead of exploding the integrator");
    }

    section("safety: thrust is zero whenever disarmed");
    {
        Simulator sim;
        runHolding(sim, 2.0, ControlInput{1.0, 1.0, 1.0, 1.0});

        const auto& s = sim.state();
        checkNear(s.surgeThrust, 0.0, 1e-9, "surge thrust stays zero");
        checkNear(s.swayThrust,  0.0, 1e-9, "sway thrust stays zero");
        checkNear(s.heaveThrust, 0.0, 1e-9, "heave thrust stays zero");
        checkNear(s.yawThrust,   0.0, 1e-9, "yaw thrust stays zero");
        checkNear(s.surgeVel,    0.0, 1e-9, "and the vehicle never accelerates");
    }

    section("safety: disarming is immediate");
    {
        Simulator sim;
        sim.setArmed(true);
        runHolding(sim, 1.0, ControlInput{1.0, 0.0, 0.0, 0.0});
        check(sim.state().surgeThrust > 0.5, "thrust was established while armed");

        sim.setArmed(false);
        checkNear(sim.state().surgeThrust, 0.0, 1e-9,
                  "thrust is zero the instant disarm is commanded");
    }

    section("dead-man timer");
    {
        Simulator sim;
        sim.setArmed(true);
        sim.setControlInput(ControlInput{1.0, 0.0, 0.0, 0.0});

        run(sim, 0.5);
        check(sim.timeSinceInput() < Simulator::kManualInputTimeout,
              "timer has not expired half a second after input");
        check(sim.state().surgeThrust > 0.5,
              "thrust is still being produced before the timeout");

        run(sim, 2.0);
        check(sim.timeSinceInput() >= Simulator::kManualInputTimeout,
              "timer expires after one second without input");
        check(std::fabs(sim.state().surgeThrust) < 0.05,
              "thrust has decayed back to zero after the timeout");
    }

    section("target setters normalise their inputs");
    {
        Simulator sim;
        sim.setTargetDepth(-5.0);
        checkNear(sim.state().targetDepth, 0.0, 1e-9,
                  "a negative target depth is clamped to the surface");

        sim.setTargetHeading(-90.0);
        checkNear(sim.state().targetHeading, 270.0, 1e-9,
                  "-90 deg normalises to 270 deg");

        sim.setTargetHeading(725.0);
        checkNear(sim.state().targetHeading, 5.0, 1e-9,
                  "725 deg normalises to 5 deg");
    }

    section("reset");
    {
        Simulator sim;
        sim.setArmed(true);
        sim.setMode(Mode::DepthHold);
        sim.setTargetDepth(12.0);
        runHolding(sim, 3.0, ControlInput{1.0, 0.0, 1.0, 0.0});

        sim.reset();
        const auto& s = sim.state();
        check(!s.armed, "reset disarms");
        check(s.mode == Mode::Manual, "reset returns to Manual");
        checkNear(s.targetDepth, 0.0, 1e-9, "reset clears the target depth");
        checkNear(s.missionTime, 0.0, 1e-9, "reset clears the mission clock");
        checkNear(s.depth, 0.0, 1e-9, "reset returns to the surface");
        checkNear(sim.batteryFraction(), 1.0, 1e-9, "reset refills the pack");
    }

    // =======================================================================
    // Phase 1 - vehicle dynamics
    // =======================================================================

    section("thruster response lag");
    {
        Simulator sim;
        sim.setArmed(true);
        const double tau = sim.params().thrusterTau;

        runHolding(sim, tau, ControlInput{1.0, 0.0, 0.0, 0.0});
        checkNear(sim.state().surgeThrust, 0.632, 0.02,
                  "thrust reaches ~63% of demand after one time constant");

        runHolding(sim, tau * 4.0, ControlInput{1.0, 0.0, 0.0, 0.0});
        checkNear(sim.state().surgeThrust, 1.0, 0.02,
                  "thrust reaches full demand after five time constants");

        check(sim.state().surgeThrust <= 1.0 + 1e-9,
              "the lag never overshoots the demand");
    }

    section("drag produces a terminal velocity");
    {
        Simulator sim;
        sim.setArmed(true);
        runHolding(sim, 30.0, ControlInput{1.0, 0.0, 0.0, 0.0});

        const double v1 = sim.state().surgeVel;
        runHolding(sim, 5.0, ControlInput{1.0, 0.0, 0.0, 0.0});
        const double v2 = sim.state().surgeVel;

        check(v1 > 0.5 && v1 < 2.0, "forward terminal speed is plausible");
        checkNear(v2, v1, 1e-3, "velocity has stopped growing - drag balances thrust");

        const auto& p = sim.params();
        const double drag = p.dragLinSurge * v2 + p.dragQuadSurge * v2 * v2;
        checkNear(drag, p.maxForceSurge, 0.5,
                  "drag at terminal velocity equals maximum thrust force");

        Simulator crab;
        crab.setArmed(true);
        runHolding(crab, 30.0, ControlInput{0.0, 1.0, 0.0, 0.0});
        check(crab.state().swayVel < v2,
              "the vehicle crabs sideways more slowly than it swims forward");
    }

    section("the vehicle coasts to a stop");
    {
        Simulator sim;
        sim.setArmed(true);
        runHolding(sim, 20.0, ControlInput{1.0, 0.0, 0.0, 0.0});
        check(sim.state().surgeVel > 0.5, "moving before the controls are released");

        run(sim, 10.0);
        check(std::fabs(sim.state().surgeVel) < 0.02,
              "drag brings the vehicle to rest once thrust is removed");
    }

    section("depth, buoyancy and the surface");
    {
        Simulator sim;
        sim.setArmed(true);
        runHolding(sim, 20.0, ControlInput{0.0, 0.0, 1.0, 0.0});
        const double deep = sim.state().depth;
        check(deep > 5.0, "full downward thrust takes the vehicle below 5 m");

        sim.setArmed(false);
        run(sim, 30.0);
        check(sim.state().depth < deep,
              "a disarmed vehicle floats back up - the trim is positive");
        check(sim.state().depth >= 0.0, "depth never goes negative");
    }

    section("the surface is a hard floor");
    {
        Simulator sim;
        sim.setArmed(true);
        runHolding(sim, 20.0, ControlInput{0.0, 0.0, -1.0, 0.0});

        checkNear(sim.state().depth, 0.0, 1e-9,
                  "sustained upward thrust at the surface does not fly the vehicle out");
        check(sim.state().heaveVel >= 0.0,
              "upward momentum is absorbed at the surface, not retained");
    }

    section("dead reckoning respects heading");
    {
        Simulator sim;
        sim.setArmed(true);

        runHolding(sim, 6.0, ControlInput{1.0, 0.0, 0.0, 0.0});
        check(sim.state().north > 1.0, "swimming north increases northing");
        check(std::fabs(sim.state().east) < 1e-6, "and leaves easting untouched");

        Simulator turn;
        turn.setArmed(true);
        int guard = 0;
        while (turn.state().yaw < 80.0 && guard++ < 5000) {
            turn.setControlInput(ControlInput{0.0, 0.0, 0.0, 1.0});
            turn.step(Simulator::kFixedStep);
        }
        check(guard < 5000, "yaw thrust actually turns the vehicle");
        run(turn, 5.0);

        const double heading = turn.state().yaw;
        const double n0 = turn.state().north;
        const double e0 = turn.state().east;
        runHolding(turn, 8.0, ControlInput{1.0, 0.0, 0.0, 0.0});

        const double dn = turn.state().north - n0;
        const double de = turn.state().east - e0;
        const double travelled = std::atan2(de, dn) * 180.0 / 3.14159265358979323846;

        check(heading > 80.0 && heading < 180.0,
              "a commanded right turn ends somewhere in the eastern half");
        check(de > 0.0, "swimming on an easterly heading increases easting");
        checkNear(travelled, heading, 8.0,
                  "the track direction matches the published heading");
    }

    section("attitude responds to thrust and settles");
    {
        Simulator sim;
        sim.setArmed(true);
        checkNear(sim.state().pitch, 0.0, 1e-9, "level at rest");

        runHolding(sim, 6.0, ControlInput{1.0, 0.0, 0.0, 0.0});
        checkNear(sim.state().pitch, sim.params().pitchPerSurge, 0.3,
                  "the hull trims to a steady pitch under full forward thrust");

        run(sim, 8.0);
        checkNear(sim.state().pitch, 0.0, 0.3, "and returns level when thrust stops");

        Simulator roll;
        roll.setArmed(true);
        runHolding(roll, 6.0, ControlInput{0.0, 1.0, 0.0, 0.0});
        check(roll.state().roll > 1.0, "the hull heels over when crabbing sideways");
    }

    section("hull tips nose-down when diving, nose-up when climbing");
    {
        // Full down-heave thrust - simulates a straight-down dive with no
        // horizontal motion. The hull should tip nose-down by the full
        // pitchPerHeave amount.
        Simulator dive;
        dive.setArmed(true);
        runHolding(dive, 6.0, ControlInput{0.0, 0.0, 1.0, 0.0});
        checkNear(dive.state().pitch, dive.params().pitchPerHeave, 0.3,
                  "hull trims to pitchPerHeave under full down-heave");
        check(dive.state().pitch < -0.5, "pitch is clearly nose-down");

        // Full up-heave thrust (climbing).
        Simulator climb;
        climb.setArmed(true);
        runHolding(climb, 6.0, ControlInput{0.0, 0.0, -1.0, 0.0});
        check(climb.state().pitch > 0.5, "pitch is clearly nose-up when climbing");
    }

    section("Depth Hold tips the hull during a large depth change");
    {
        // The whole point of this coupling - operators watching the ADI should
        // see the vehicle tip during a transition to a new target depth, not
        // only read it off the depth chart.
        Simulator sim;
        sim.setArmed(true);
        sim.setMode(Mode::DepthHold);
        sim.setTargetDepth(10.0);

        // Watch for a nose-down peak during the dive-in phase.
        double peakNoseDown = 0.0;   // most negative pitch
        for (int i = 0; i < 400; ++i) {
            sim.step(Simulator::kFixedStep);
            if (sim.state().pitch < peakNoseDown)
                peakNoseDown = sim.state().pitch;
        }
        check(peakNoseDown < -1.0,
              "ADI shows a visible nose-down trim during the dive to 10 m");

        // Once settled, pitch returns close to level (only a tiny trim remains
        // to balance buoyancy via the integrator).
        run(sim, 40.0);
        check(std::fabs(sim.state().pitch) < 0.6,
              "at the target depth, the hull sits nearly level");
    }

    section("battery");
    {
        Simulator idle;
        idle.setArmed(true);
        run(idle, 60.0);

        Simulator working;
        working.setArmed(true);
        runHolding(working, 60.0, ControlInput{1.0, 1.0, 1.0, 1.0});

        check(idle.batteryFraction() < 1.0, "the pack drains even at idle");
        check(working.batteryFraction() < idle.batteryFraction(),
              "a working vehicle drains the pack faster than an idle one");
        check(working.state().batteryVolts < idle.state().batteryVolts,
              "pack voltage is lower under load");

        const double underLoad = working.state().batteryVolts;
        run(working, 2.0);
        check(working.state().batteryVolts > underLoad,
              "voltage recovers when the thrusters ease off (sag, not discharge)");

        check(idle.batteryCurrent() > 0.0, "idle still draws hotel current");
        check(working.state().batteryVolts > 10.0, "voltage stays physically sane");
    }

    section("the model stays bounded under abuse");
    {
        Simulator sim;
        sim.setArmed(true);
        for (int i = 0; i < 180000; ++i) {
            const double s = (i % 2 == 0) ? 1.0 : -1.0;
            sim.setControlInput(ControlInput{s, -s, s, -s});
            sim.step(Simulator::kFixedStep);
        }
        const auto& st = sim.state();
        check(std::isfinite(st.surgeVel) && std::isfinite(st.depth)
                  && std::isfinite(st.yaw) && std::isfinite(st.north),
              "no NaN or infinity after an hour of full-rate reversals");
        check(st.depth >= 0.0, "depth is still valid");
        check(st.yaw >= 0.0 && st.yaw < 360.0, "heading is still wrapped into [0, 360)");
    }

    // =======================================================================
    // Phase 2 - hold-mode controllers
    // =======================================================================

    section("heading error wrapping");
    {
        checkNear(wrapError180(10.0),   10.0,  1e-9, "small positive stays positive");
        checkNear(wrapError180(-10.0), -10.0,  1e-9, "small negative stays negative");
        checkNear(wrapError180(190.0), -170.0, 1e-9, "190 wraps to -170 (turn left)");
        checkNear(wrapError180(-190.0), 170.0, 1e-9, "-190 wraps to 170 (turn right)");
        checkNear(wrapError180(370.0),  10.0,  1e-9, "370 wraps to 10");
        checkNear(wrapError180(-370.0), -10.0, 1e-9, "-370 wraps to -10");
        // The 359 -> 1 boundary, the classic interview question.
        checkNear(wrapError180(1.0 - 359.0), 2.0, 1e-9,
                  "target 1 deg from yaw 359 deg: error is +2, not -358");
    }

    section("depth hold reaches and holds its target");
    {
        Simulator sim;
        sim.setArmed(true);
        sim.setMode(Mode::DepthHold);
        sim.setTargetDepth(5.0);
        run(sim, 40.0);

        checkNear(sim.state().depth, 5.0, 0.3,
                  "depth-hold settles within 30 cm of the target");
        check(std::fabs(sim.state().heaveVel) < 0.15,
              "vertical velocity is near zero at steady state");
        check(sim.state().heaveThrust > 0.0,
              "steady-state heave thrust is slightly down, trimming the positive buoyancy");
    }

    section("depth hold responds to a setpoint change");
    {
        Simulator sim;
        sim.setArmed(true);
        sim.setMode(Mode::DepthHold);
        sim.setTargetDepth(3.0);
        run(sim, 30.0);
        checkNear(sim.state().depth, 3.0, 0.3, "settled at the first target");

        sim.setTargetDepth(8.0);
        run(sim, 40.0);
        checkNear(sim.state().depth, 8.0, 0.3, "settled at the new target");

        // Overshoot check: the depth should not have blown past the new target
        // by more than a reasonable amount on its way down.
        // (We can only sample the final state - a tighter test would log
        // the trajectory, which Phase 5 adds via CSV logging.)
        check(sim.state().depth < 10.0, "no gross overshoot of the new target");
    }

    section("depth hold leaves horizontal axes to the operator");
    {
        Simulator sim;
        sim.setArmed(true);
        sim.setMode(Mode::DepthHold);
        sim.setTargetDepth(4.0);
        runHolding(sim, 20.0, ControlInput{1.0, 0.0, 0.0, 0.0});

        check(sim.state().surgeThrust > 0.5,
              "forward thrust still obeys the operator in Depth Hold");
        checkNear(sim.state().depth, 4.0, 0.5,
                  "depth is still held while swimming forward");
    }

    section("heading hold turns the short way around the compass");
    {
        // Start at yaw = 0, target = 350. The short way is -10 deg (through
        // north, counter-clockwise). The long way would be +350 deg clockwise,
        // the whole long way around through south.
        Simulator sim;
        sim.setArmed(true);
        sim.setMode(Mode::HeadingHold);
        sim.setTargetHeading(350.0);

        // Three seconds in, the vehicle must still be in the upper arc
        // (315..360). If the controller took the long way around, by now the
        // heading would be nowhere near 315 - at ~40 deg/s terminal turn rate,
        // three seconds of turning the wrong way would land us somewhere near
        // 100 deg.
        run(sim, 3.0);
        check(sim.state().yaw > 315.0 && sim.state().yaw <= 360.0,
              "mid-approach, yaw is in the upper arc - turning the short way");

        run(sim, 15.0);
        checkNear(wrapError180(sim.state().yaw - 350.0), 0.0, 3.0,
                  "heading-hold settles near the 350 deg target");
    }

    section("heading hold reaches a 90 deg target from zero");
    {
        Simulator sim;
        sim.setArmed(true);
        sim.setMode(Mode::HeadingHold);
        sim.setTargetHeading(90.0);
        run(sim, 15.0);

        checkNear(sim.state().yaw, 90.0, 2.0, "settled within 2 deg of 90 deg");
        check(std::fabs(sim.state().yawRate) < 1.0,
              "yaw rate is near zero at steady state");
    }

    section("heading hold does not overshoot");
    {
        // A well-tuned PD controller with D on measurement reaches the setpoint
        // monotonically. Overshoot would mean carrying too much yaw rate into
        // the target, and no amount of post-hoc hunting would make it clean.
        Simulator sim;
        sim.setArmed(true);
        sim.setMode(Mode::HeadingHold);
        sim.setTargetHeading(60.0);

        double peakYaw = 0.0;
        for (int i = 0; i < 1500; ++i) {
            sim.step(Simulator::kFixedStep);
            if (sim.state().yaw > peakYaw)
                peakYaw = sim.state().yaw;
        }
        check(peakYaw <= 60.5,
              "the vehicle never swings past the target by more than 0.5 deg");
    }

    section("hold modes do nothing when the vehicle is disarmed");
    {
        Simulator sim;
        sim.setMode(Mode::DepthHold);
        sim.setTargetDepth(5.0);
        run(sim, 10.0);

        checkNear(sim.state().heaveThrust, 0.0, 1e-9,
                  "depth-hold produces no thrust while disarmed");
        checkNear(sim.state().depth, 0.0, 1e-9,
                  "vehicle has not moved");
    }

    section("mode switch resets the integrators");
    {
        Simulator sim;
        sim.setArmed(true);
        sim.setMode(Mode::DepthHold);
        sim.setTargetDepth(10.0);
        // A long deep dive builds up integrator charge.
        run(sim, 5.0);

        sim.setMode(Mode::Manual);
        // The setMode() call itself should have reset the integrators; running
        // another tick in Manual must not leak heave thrust from a stale
        // integrator from the previous mode.
        run(sim, 0.1);
        // The thruster lag means some heave thrust is still decaying from the
        // last Depth Hold tick, but the controller itself must not be adding
        // any fresh demand on top of that.
        sim.setMode(Mode::DepthHold);
        sim.setTargetDepth(sim.state().depth);  // already there, error ~= 0
        run(sim, 2.0);

        // With error near zero on entry and the integrator freshly reset, the
        // controller should barely move the vehicle.
        const double startDepth = sim.state().depth;
        run(sim, 3.0);
        check(std::fabs(sim.state().depth - startDepth) < 0.3,
              "fresh Depth Hold at current depth does not jerk - integrator was reset");
    }

    // =======================================================================
    // PID component tests (exercised in isolation)
    // =======================================================================

    section("PID anti-windup");
    {
        Pid pid;
        pid.setGains(1.0, 1.0, 0.0);
        pid.setOutputLimits(-1.0, 1.0);

        // Hold a large positive error for a long time at full saturation.
        for (int i = 0; i < 1000; ++i)
            pid.update(5.0, 0.0, 0.01);

        // The integrator must not have grown without bound.
        check(pid.integral() < 2.0,
              "integrator is bounded while the output is saturated");

        // Now give it a large negative error - it should respond at once, not
        // after discharging an enormous positive integrator.
        const double out = pid.update(-5.0, 0.0, 0.01);
        checkNear(out, -1.0, 1e-6,
                  "controller swings to the negative rail immediately on error reversal");
    }

    // =======================================================================
    // Phase 5 - ocean current and CSV logger
    // =======================================================================

    section("disarmed vehicle drifts with horizontal current");
    {
        Simulator sim;
        sim.setCurrent(0.3, 0.0, 0.0);   // 0.3 m/s to the north
        run(sim, 10.0);

        const auto& s = sim.state();
        check(!s.armed, "still disarmed");
        checkNear(s.north, 3.0, 0.1, "drifted ~3 m north in 10 s of current");
        checkNear(s.east,  0.0, 0.05, "no eastward drift");
        checkNear(s.surgeVel, 0.0, 1e-6,
                  "body-frame velocity through water stays zero - drag acts on none");
    }

    section("drag acts on the relative velocity, not the ground track");
    {
        // The vehicle swims into a headwind. Its ground speed should fall below
        // its still-water terminal speed because drag is set by motion through
        // the water, and the ground speed = water-through-vehicle + current.
        Simulator still;
        still.setArmed(true);
        runHolding(still, 30.0, ControlInput{1.0, 0.0, 0.0, 0.0});
        const double vGroundStill = still.state().surgeVel;   // body=ground here

        Simulator headwind;
        headwind.setArmed(true);
        headwind.setCurrent(-0.3, 0.0, 0.0);   // current flows SOUTH, we swim NORTH
        runHolding(headwind, 60.0, ControlInput{1.0, 0.0, 0.0, 0.0});

        // Over the last second, how much ground did the vehicle actually cover?
        const double n1 = headwind.state().north;
        runHolding(headwind, 1.0, ControlInput{1.0, 0.0, 0.0, 0.0});
        const double groundSpeed = headwind.state().north - n1;

        check(groundSpeed < vGroundStill - 0.1,
              "ground speed into a headwind is lower than still-water terminal speed");
        check(groundSpeed > 0.0, "the vehicle still makes net ground");
    }

    section("depth hold compensates for a vertical current");
    {
        Simulator sim;
        sim.setArmed(true);
        sim.setMode(Mode::DepthHold);
        sim.setTargetDepth(5.0);
        sim.setCurrent(0.0, 0.0, 0.1);   // 10 cm/s downward disturbance

        run(sim, 40.0);
        checkNear(sim.state().depth, 5.0, 0.4,
                  "depth-hold still settles near 5 m with a downward current");

        // The controller must be producing more upward trim (less positive
        // heave thrust) than it would without the current - intuition only,
        // the integrator handles the exact value.
        check(sim.state().heaveThrust < 0.3,
              "heaveThrust is modest, not saturated");
    }

    // Build a unique temp path for a test-run. tmpnam() is correct enough for
    // a self-contained test but linkers warn about it, so we build our own.
    auto tempPath = []() {
        static int counter = 0;
        std::ostringstream oss;
        oss << "/tmp/rov_test_" << std::time(nullptr)
            << "_" << (++counter) << ".csv";
        return oss.str();
    };

    section("CsvLogger writes a header and one row per call");
    {
        const std::string p = tempPath();

        {
            CsvLogger log;
            check(log.open(p), "open() succeeds");
            check(log.isOpen(), "logger reports open");

            Simulator sim;
            sim.setArmed(true);
            sim.setMode(Mode::DepthHold);
            sim.setTargetDepth(3.0);
            for (int i = 0; i < 5; ++i) {
                sim.step(Simulator::kFixedStep);
                log.write(sim.state().missionTime, sim.state(),
                          sim.batteryCurrent(), sim.batteryFraction());
            }

            check(log.rowCount() == 5u, "five rows written");
            log.close();
            check(!log.isOpen(), "logger closes cleanly");
        }

        // Parse the file back.
        std::ifstream in(p);
        check(in.is_open(), "file exists after close");

        std::string header;
        std::getline(in, header);
        check(header.find("time_s") == 0, "header starts with time_s");
        check(header.find("mode") != std::string::npos,       "has mode column");
        check(header.find("depth_m") != std::string::npos,    "has depth column");
        check(header.find("target_depth_m") != std::string::npos,
              "has target_depth column");
        check(header.find("battery_v") != std::string::npos,  "has battery column");

        int rows = 0;
        std::string line;
        while (std::getline(in, line))
            if (!line.empty()) ++rows;
        check(rows == 5, "file contains five data rows");

        std::remove(p.c_str());
    }

    section("CsvLogger reopen closes the previous file");
    {
        const std::string path1 = tempPath();
        const std::string path2 = tempPath();

        CsvLogger log;
        check(log.open(path1), "first open succeeds");
        Simulator sim;
        for (int i = 0; i < 3; ++i) {
            sim.step(Simulator::kFixedStep);
            log.write(sim.state().missionTime, sim.state(),
                      sim.batteryCurrent(), sim.batteryFraction());
        }

        check(log.open(path2), "second open succeeds");
        check(log.rowCount() == 0u, "row counter was reset on reopen");
        check(log.path() == path2, "logger now reports the new path");

        log.close();
        std::remove(path1.c_str());
        std::remove(path2.c_str());
    }

    section("PID derivative on measurement does not kick");
    {
        Pid pid;
        pid.setGains(1.0, 0.0, 10.0);
        pid.setOutputLimits(-100.0, 100.0);

        // A setpoint jump with the measurement stationary should produce a
        // pure P response (= Kp * error), with NO derivative contribution. A
        // D-on-error controller would show a huge spike here.
        const double out = pid.update(5.0, 0.0, 0.01);
        checkNear(out, 5.0, 1e-9,
                  "setpoint step produces no derivative kick (Kd * 0 = 0)");
    }

    std::printf("\n%d checks, %d failure(s)\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
