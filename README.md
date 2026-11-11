# ROV Telemetry Ground Station

A native desktop application (C++17 / Qt 6 / QML) that simulates a small
underwater vehicle and lets an operator monitor and control it.

Submitted for the Coratia Technologies software internship technical assessment.

---

## Status

| Phase | Scope | State |
|-------|-------|-------|
| 0 | Project skeleton, build system, dashboard layout, test harness | **done** |
| 1 | Vehicle dynamics, drag, thruster response lag, battery model | planned |
| 2 | Arm/disarm, Manual input, dead-man safety, Depth Hold, Heading Hold | planned |
| 3 | Live dashboard bound to the simulator, keyboard control | planned |
| 4 | Depth-vs-time plot, OpenStreetMap view | planned |
| 5 | CSV logging, simulated current, extended tests | planned |
| 6 | Documentation and video walkthrough | planned |

---

## Architecture

The one decision everything else follows from: **the simulation core does not
know the GUI exists.**

```
src/sim/      rovsim  — static library, pure C++17, links no Qt at all
                        vehicle state, hydrodynamics, control loops,
                        safety interlocks
src/app/      rov_ground_station — Qt6/QML executable
                        dashboard, operator input, plot, map
tests/        test_simulator — headless, links rovsim only
```

Consequences that matter:

- **The safety interlocks are enforced inside the simulator, not in the UI.**
  Thrust is zeroed in `Simulator::integrate()` whenever the vehicle is
  disarmed, and `Simulator::setArmed(false)` zeroes it immediately without
  waiting for the next tick. A UI bug, a stuck key, or a future control source
  cannot route around them.
- **The physics runs at a fixed 50 Hz timestep, independent of the frame rate.**
  `Simulator::step(dt)` sub-steps internally and carries the remainder, so a
  late frame loses no simulated time and a stalled driver cannot hand the
  integrator a dt large enough to destabilise it.
- **The physics is testable without a display.** `tests/test_simulator.cpp`
  links `rovsim` alone — no Qt, no event loop, no window.

---

## Building

### Requirements

- CMake 3.21+
- A C++17 compiler
- Qt 6.5 or newer (`Core`, `Gui`, `Quick`, `QuickControls2`)

On macOS:

```bash
brew install qt cmake
```

### Configure and build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

If CMake cannot find Qt, point it at your installation:

```bash
cmake -B build -DCMAKE_PREFIX_PATH="$(brew --prefix qt)"
```

### Run

```bash
./build/bin/rov_ground_station.app/Contents/MacOS/rov_ground_station   # macOS
./build/bin/rov_ground_station                                         # Linux
```

### Test

```bash
ctest --test-dir build --output-on-failure
```

The simulation tests also build standalone, without Qt present at all:

```bash
g++ -std=c++17 -I src/sim src/sim/Simulator.cpp tests/test_simulator.cpp -o /tmp/t && /tmp/t
```

---

## Conventions

- Angles are degrees everywhere in the public interface; radians only inside
  a computation that needs them.
- Headings are `[0, 360)`, 0 = north, increasing clockwise.
- Depth is metres **below** the surface and is always non-negative.
- Operator demands are normalised to `[-1, 1]`; they are demands, not thrusts.

---

## AI tool disclosure

Per the assessment rules, AI coding assistance is disclosed here.

- **Claude (Anthropic)** was used for: project planning and phase breakdown,
  scaffolding the CMake build, drafting the QML dashboard layout, and reviewing
  the simulation core.
- All code was read, understood and adjusted by hand before being committed.
  (Example: the first version of the fixed-timestep accumulator in
  `Simulator::step()` silently dropped a tick for any `dt` not exactly divisible
  by the step size; the unit test caught it and the comparison now carries an
  explicit epsilon.)
- The physics model, control strategy and safety design decisions are documented
  above and I am able to explain and modify any part of them.

*This section will be updated as later phases are implemented.*
