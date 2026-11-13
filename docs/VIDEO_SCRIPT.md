# Video walkthrough — script and recording guide

Target length: **~3 minutes** (brief allows "interface walkthrough only"; the
hold modes and safety responses do not need to be demonstrated, but showing
that they work costs nothing and is worth doing).

---

## Recording setup

- **Tool:** QuickTime Player (macOS, built-in). `Cmd + Shift + 5` → *Record
  Selected Portion* → draw a box around the app window → Record.
- **Resolution:** keep the app window at its default **1440 × 880** or larger.
  Retina scaling records at 2× so the output file is already crisp.
- **Audio:** built-in Mac mic is fine. Quiet room, no music.
- **Export:** QuickTime saves as `.mov` at ~1080p. Convert to mp4 if the
  submission portal needs it:
  ```bash
  ffmpeg -i walkthrough.mov -c:v libx264 -crf 23 -c:a aac walkthrough.mp4
  ```
- **Takes:** record 2–3 takes, keep the best. Don't try to do it in one.
- **Before you hit record:**
  - Close chat apps so notifications don't pop in.
  - **Disarm** the vehicle, click **Reset Simulation**, set current back to 0.
  - **Enable "Do Not Disturb"** so that Low-Battery popup you had before
    doesn't appear mid-shot.
  - Plug the charger in if the laptop is at low battery.

---

## Script

Lines in `"quotes"` are what you say.
Lines in **bold** are what you do on screen.

### 0:00 – 0:20   Intro

"Hi, this is my submission for the Coratia Technologies ROV Telemetry Ground
Station assessment. It's a native desktop application in C++17 with Qt 6 and
QML — no browser, no web view. I'm going to walk through the interface, then
spend the last thirty seconds on how it's put together."

### 0:20 – 0:55   Dashboard tour

**Mouse over each region as you name it.**

"The layout has four columns. On the left, numeric telemetry: position —
depth, north, east — power with battery voltage and current draw, signed
thrust bars for all four axes, and an Environment panel for the simulated
ocean current.

In the center is the attitude widget — an aviation-style artificial horizon
for roll and pitch, a compass below it for heading, and numeric readouts at
the bottom.

To its right is Flight Control: arm/disarm, the three flight modes, target
setpoints, CSV logging, and reset.

On the far right, three stacked panels: depth-vs-time, heading-vs-time, and
an embedded OpenStreetMap view with the live track."

### 0:55 – 1:20   Arm and manual

"Space arms the vehicle." **Press Space.** "The state pill goes green. The
battery starts drawing hotel current — about 1.5 amps at idle."

**Hold W for ~2 seconds, then release.**

"W is forward. Watch the surge thrust bar fill — and the artificial horizon:
the nose pitches down about four degrees because the forward thrusters are
behind the centre of mass. When I release, thrust doesn't drop to zero
instantly — there's a first-order lag with a 0.25 second time constant, and
you can see the bar deflating smoothly."

### 1:20 – 1:50   Flight modes

**Press 2 for Depth Hold. Click the target depth spinbox. Type 10. Press Enter.**

"Depth Hold. Target 10 metres. The depth chart curves down to the dashed
amber line — that's the target. The ADI shows a nose-down trim during the
dive, then levels off at the target because the controller only needs a
small sustained thrust to trim out positive buoyancy."

**Wait for it to settle, then press 3.**

"Heading Hold. Target heading…" **Click heading spinbox, type 350, Enter.**
"…350 degrees from current heading zero. The compass spins to the left
because the controller wraps the error — it's a ten-degree turn the short
way through north, not three hundred and fifty degrees the long way. The
orange bug on the compass rim is the target, and the heading chart draws
the convergence."

### 1:50 – 2:20   Mission features

"Ocean current." **Click Environment > Current spinbox, set to 0.3 m/s.** **Set
heading to 90.** "An amber arrow appears at the origin on the map showing
the flow direction. Drag acts on the vehicle's velocity through the water,
not its ground speed — so the vehicle drifts with the current and the depth
hold has to do a tiny bit more work."

**Click Start Logging.**

"CSV logging — writes one row every hundred milliseconds into a timestamped
file in my home directory. Every field an analyst would want is in there.
The toast shows the path, and the row counter ticks up live."

**Click Stop Logging.**

### 2:20 – 2:55   Architecture note

**Open a terminal or Finder in the project folder briefly.**

"One architectural decision worth calling out. The simulation lives in a
static C++17 library that doesn't link Qt. The physics runs at fixed 50 Hz,
decoupled from the UI frame rate. Safety interlocks — arm/disarm, the
one-second dead-man timer — are enforced inside the simulator, not in the
dashboard, so no UI bug can route around them.

The whole thing is covered by a 116-check headless test suite that builds
and runs with nothing but a C++ compiler. The one class that bridges Qt and
physics is RovController, exposed to QML as a singleton — it's the only
object in the codebase that includes headers from both sides."

### 2:55 – 3:05   Close

"Full architecture diagram and build instructions are in the README. AI
assistance is disclosed there too. Thanks for watching."

---

## Things worth pointing to on screen if you have the time

Only do these if your take ends up short of three minutes, otherwise skip.

- **Dead-man indicator** in the footer — press W, release, point at the
  "Dead-man" counter climbing toward 1.0 s, highlight when it goes amber.
- **Battery voltage dips** when thrust is applied and recovers when it's
  released (sag through pack resistance, not discharge).
- **Reset Simulation** button clears depth plot, heading plot, track,
  battery back to full, and mission clock back to zero.
- **ADI pitch ladder** — point at the ±10/20/30 marks as the nose tips.

---

## Things to NOT do

- Don't open developer tools / show code. Interface walkthrough only.
- Don't apologise for anything. If a take has a stumble, record again.
- Don't read the README aloud — the viewer can find it. Keep it to what the
  video shows.
- Don't exceed 4 minutes. 3 is ideal.
