// Minimal dependency-free test harness. Deliberately not GoogleTest or Qt Test:
// the simulation core has no third-party dependencies and the tests should not
// introduce one.

#include "Simulator.h"

#include <cmath>
#include <cstdio>
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

/// Drive the simulator for \p seconds of simulated time at the fixed rate.
void run(rov::Simulator& sim, double seconds)
{
    const int steps = static_cast<int>(seconds / rov::Simulator::kFixedStep);
    for (int i = 0; i < steps; ++i)
        sim.step(rov::Simulator::kFixedStep);
}

} // namespace

int main()
{
    using namespace rov;

    section("power-on state");
    {
        Simulator sim;
        const auto& s = sim.state();
        check(!s.armed, "vehicle starts disarmed");
        check(s.mode == Mode::Manual, "vehicle starts in Manual mode");
        checkNear(s.depth, 0.0, 1e-9, "starts at the surface");
        checkNear(s.batteryVolts, 16.8, 1e-9, "battery starts full");
        checkNear(s.missionTime, 0.0, 1e-9, "mission clock starts at zero");
    }

    section("fixed-step integration");
    {
        Simulator sim;
        run(sim, 1.0);
        checkNear(sim.state().missionTime, 1.0, 1e-6,
                  "one second of stepping advances the clock by one second");

        // A single oversized dt must be sub-stepped, not applied whole.
        Simulator sim2;
        sim2.step(0.1);
        checkNear(sim2.state().missionTime, 0.1, 1e-6,
                  "a 100 ms dt is sub-stepped into five 20 ms steps");

        // A pathological dt is clamped rather than integrated.
        Simulator sim3;
        sim3.step(10.0);
        check(sim3.state().missionTime <= 0.25 + 1e-6,
              "a stalled-driver dt is clamped instead of exploding the integrator");
    }

    section("safety: thrust is zero whenever disarmed");
    {
        Simulator sim;
        // Full demand on every axis while disarmed.
        sim.setControlInput(ControlInput{1.0, 1.0, 1.0, 1.0});
        run(sim, 2.0);

        const auto& s = sim.state();
        checkNear(s.surgeThrust, 0.0, 1e-9, "surge thrust stays zero");
        checkNear(s.swayThrust,  0.0, 1e-9, "sway thrust stays zero");
        checkNear(s.heaveThrust, 0.0, 1e-9, "heave thrust stays zero");
        checkNear(s.yawThrust,   0.0, 1e-9, "yaw thrust stays zero");
    }

    section("safety: disarming is immediate");
    {
        Simulator sim;
        sim.setArmed(true);
        sim.setControlInput(ControlInput{1.0, 0.0, 0.0, 0.0});
        run(sim, 0.5);

        sim.setArmed(false);
        // No step() in between - the interlock must already have fired.
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

        run(sim, 1.0);
        check(sim.timeSinceInput() >= Simulator::kManualInputTimeout,
              "timer expires after one second without input");
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
        run(sim, 3.0);

        sim.reset();
        const auto& s = sim.state();
        check(!s.armed, "reset disarms");
        check(s.mode == Mode::Manual, "reset returns to Manual");
        checkNear(s.targetDepth, 0.0, 1e-9, "reset clears the target depth");
        checkNear(s.missionTime, 0.0, 1e-9, "reset clears the mission clock");
    }

    std::printf("\n%d checks, %d failure(s)\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
