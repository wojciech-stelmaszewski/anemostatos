# UI and Visualization

The screen has two jobs: show **what the drone does** (3D scene) and show
**why** (charts, numbers, explanations). Both must stay in sync and share
one visual language: the same colour always means the same quantity.

## 1. Layout

```
┌────────────────────────────────────────────────────────────┬──────────────────┐
│ ▶ ❚❚ ⏭  0.25× 1× 2×   t = 12.43 s   seed 1337   L1 ▾       │  PARAMETERS      │
├────────────────────────────────────────────────────────────┤  ▸ Controller    │
│                                                            │    Kp ━━━●━━ 6.0 │
│                       3D VIEWPORT                          │    Ki ━●━━━━ 2.0 │
│                                                            │    Kd ━━●━━━ 4.0 │
│        (drone, grid, arrows, setpoint ghost, wind)         │    [P][I][D] ... │
│                                                            │  ▸ Physics       │
│   ┌─────────────────────────────┐                          │  ▸ Wind          │
│   │ u = 6.0·0.42 + 1.87 − 0.95  │  ← live formula          │  ▸ Sensors       │
│   │   = 3.44 N  (+9.81 FF)      │                          │  ▸ Simulation    │
│   └─────────────────────────────┘                          │                  │
├────────────────────────────────────────────────────────────┤  LESSON          │
│ CHARTS  [10 s ▾]  loop: alt ▾                              │  3/12 Gravity    │
│ ┌ setpoint vs measurement ────┐ ┌ P / I / D / FF / u ─────┐│  droop …         │
│ └─────────────────────────────┘ └─────────────────────────┘│  [prev] [next]   │
│ ┌ error ──────────────────────┐ ┌ motors / wind ──────────┐│                  │
│ └─────────────────────────────┘ └─────────────────────────┘│                  │
└────────────────────────────────────────────────────────────┴──────────────────┘
```

- Right sidebar: collapsible parameter groups + lesson panel.
- Bottom: chart grid, resizable splitter, can be collapsed to maximise 3D.
- Narrow screens: sidebar and charts become tabs under the viewport.
- Dark theme by default (charts read better), light theme available.

## 2. 3D scene

### 2.1 The floor — reference grid

The only "scenery", so it has to carry spatial information well:

- Infinite-looking grid rendered by a shader on a large plane, fading with
  distance (no visible edge).
- Line hierarchy: **0.1 m** minor (fades in only near the camera), **1 m**
  regular, **5 m** emphasised.
- World axes drawn on the floor: X red, Z blue; origin marked.
- Coordinate labels every 5 m along the axes (sprite text).
- **Altitude aids:**
  - vertical drop line from the drone to the floor with a live height label,
  - the drone's soft shadow on the grid (strong depth cue),
  - an optional vertical ruler pole with 0.5 m ticks next to the drone,
  - a horizontal "setpoint plane" ring at the target altitude.
- Subtle gradient sky + distance fog. Nothing else.

### 2.2 The drone model

Procedural (a `<Drone>` R3F component built from Three.js primitives, no
external assets),
aimed at a clean "product render" look:

| Part             | Construction                                                                                                                                                                      |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Frame            | X-shaped carbon arms: flattened rounded boxes, dark graphite with subtle carbon-weave normal/roughness texture (generated).                                                       |
| Body             | Rounded box core + smooth top canopy in an accent colour (e.g. orange), small vent details.                                                                                       |
| Battery          | Pack strapped under the body, with a strap.                                                                                                                                       |
| Motors           | Metallic bells (cylinders with a chamfer), anodised accent rings.                                                                                                                 |
| Propellers       | Two-blade, slightly twisted blades; CW/CCW pairs mirrored. Spin speed ∝ √thrust. Above a threshold, blades fade into a translucent motion-blur disc whose opacity follows thrust. |
| Orientation cues | Front: small camera pod + white/green LEDs. Rear: red LEDs. Needed in L3 to read yaw.                                                                                             |
| Landing gear     | Four short legs / skids.                                                                                                                                                          |

- PBR materials (`MeshStandardMaterial`), image-based lighting from a
  generated `RoomEnvironment`, one directional key light casting soft shadows.
- **Motor saturation glow**: a motor at 0 % or 100 % tints its ring red — the
  actuator limit becomes visible on the model itself.
- Level of detail kept modest (well under 20k triangles).

### 2.3 Overlays (all toggleable)

| Overlay                    | Meaning                                                                                                                                                 |
| -------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Setpoint ghost             | Translucent wireframe drone (or crosshair) at the target. Draggable with the mouse to change the setpoint.                                              |
| Error line                 | Dashed line drone → setpoint, labelled with the error.                                                                                                  |
| Force arrows               | Thrust, gravity, drag/wind force, net force — scaled in N, colour-coded as in the charts.                                                               |
| Term arrows (L1/L2)        | Separate arrows for the P, I, D and feedforward contributions — the PID made visible _on the drone_.                                                    |
| Desired thrust vector (L3) | $\vec F_{des}$ from the cascade vs. actual thrust axis.                                                                                                 |
| Trail                      | Fading path of the last N seconds.                                                                                                                      |
| Wind                       | Streaking particles moving with the air velocity; density/brightness ∝ speed; a flash/wave when a gust starts. Plus a HUD wind indicator (arrow + m/s). |

### 2.4 Camera

- Orbit (default), follow (orbit around the moving drone), side view
  (locked, ideal for altitude lessons — reads like a chart), top view.
- Smooth transitions; double-click to refocus on the drone.

## 3. Live charts

All charts share the time axis, cursor and pause state (hover on one →
crosshair and values in all). Time window: 5 / 10 / 30 / 60 s.

| Chart             | Series                                                                                                                                     |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| **Tracking**      | setpoint (dashed), measurement, true value (if sensor noise/bias is on — shows the difference!), frozen "ghost" run for comparison.        |
| **Error**         | $e(t)$, ±2 % settling band after a step, shaded integral area (what the I term "sees").                                                    |
| **PID terms**     | P, I, D, FF contributions and total output $u$; output limits drawn as bands; saturated intervals shaded. The single most important chart. |
| **Actuators**     | Four motor thrusts (commanded vs. actual — shows motor lag), limits.                                                                       |
| **Disturbance**   | Wind components / magnitude, gust events as markers.                                                                                       |
| **Attitude (L3)** | roll/pitch/yaw setpoint vs actual; body rates.                                                                                             |

- **Loop selector**: in L2/L3 the Tracking/Error/PID charts follow the loop
  chosen in the inspector (`alt`, `vel.x`, `att.roll`, `rate.pitch`, …).
- **Colour semantics** (fixed across charts, overlays and panel):
  setpoint, measurement, P, I, D, FF, output, wind each get one hue, chosen
  for colour-blind safety and good contrast on dark and light backgrounds.
- **Freeze & compare**: "Snapshot" button stores the current run as a ghost
  trace; subsequent runs are overlaid.
- **Export**: CSV of the visible buffers.

### Step-response metrics

After every setpoint step the analyser measures rise time, overshoot,
settling time and steady-state error (definitions in
[pid-primer.md §3](pid-primer.md#3-reading-a-step-response)) and shows them in
a small table next to the Tracking chart, with markers on the chart itself
(10 %/90 % lines, peak, settling point). Also shown for the current gains
(L1 only): theoretical $\omega_n$, $\zeta$ and $e_{ss} = mg/K_p$ when relevant.

## 4. Parameter panel

Generated from the parameter schema (see
[software-architecture.md §4](software-architecture.md#4-state-parameters-and-presets)).
Every control has a unit, a range, a reset-to-default, and an ⓘ tooltip with a
one-paragraph explanation.

| Group          | Parameters                                                                                                                                                                                                                                                                                                            |
| -------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Controller** | $K_p, K_i, K_d$ (log-scale sliders + numeric input); P/I/D enable toggles; feedforward on/off and assumed mass $\hat m$; derivative on error/measurement; D filter cutoff; anti-windup mode (+ $K_b$ / limit); output limits; controller rate; parallel/standard form display ($T_i$, $T_d$); Ziegler–Nichols helper. |
| **Setpoint**   | value (or drag in 3D, or ↑/↓ keys); quick steps (±0.5 m, ±1 m); profile generator: step, square wave, ramp, sine (period, amplitude); setpoint rate limit.                                                                                                                                                            |
| **Physics**    | true mass; max motor thrust; motor time constant; drag coefficients; payload drop/add buttons.                                                                                                                                                                                                                        |
| **Wind**       | mean speed/heading/vertical; turbulence σ and τ; gust rate, amplitude range, duration range, vertical share; "Gust now" with direction picker; wind on/off master switch.                                                                                                                                             |
| **Sensors**    | noise σ, bias, delay.                                                                                                                                                                                                                                                                                                 |
| **Simulation** | level L1/L2/L3; seed (random / fixed); time scale; reset (keeps parameters) / full reset; share link; export CSV.                                                                                                                                                                                                     |

## 5. Lessons

A lesson = **preset** (parameters + seed) + **short text** (MDX with
KaTeX) + optional **scripted events** (e.g. "at t = 3 s step setpoint to
3 m", "at t = 8 s gust 6 m/s downward") + optional **goal check**
("get overshoot below 5 %").

Planned lessons (L1 unless stated):

| #   | Title                     | Idea                                                                                                       |
| --- | ------------------------- | ---------------------------------------------------------------------------------------------------------- |
| 1   | Meet the loop             | Tour of the screen: setpoint, error, output. Controller off → the drone falls.                             |
| 2   | Just a spring             | P-only with feedforward: it oscillates. Why? $\omega_n$, no damping.                                       |
| 3   | The gravity droop         | P-only, no feedforward: $e_{ss} = mg/K_p$, predicted vs measured.                                          |
| 4   | Add a damper              | Add D, reach critical damping; relate to $\zeta$.                                                          |
| 5   | Integral to the rescue    | Mass mismatch / constant wind → steady error → I removes it; I-induced overshoot.                          |
| 6   | Windup                    | Saturation + integral = overshoot; compare anti-windup strategies.                                         |
| 7   | Derivative kick           | Setpoint steps with D on error vs on measurement.                                                          |
| 8   | Noisy sensor              | D amplifies noise; filter trade-off.                                                                       |
| 9   | Too slow to react         | Lower controller rate / add sensor delay until unstable.                                                   |
| 10  | Find the edge             | Ziegler–Nichols ultimate gain experiment, guided.                                                          |
| 11  | Gust challenge            | Fixed seed, fixed gust sequence; score = RMS error + max deviation + energy used. Beat the reference tune. |
| 12  | Why drones tilt (L2 → L3) | Same wind, point mass vs quadrotor.                                                                        |
| 13  | Cascade                   | Tune rate → attitude → velocity → position, inside out.                                                    |
| 14  | Cascade inversion (L3)    | Outer loop faster than inner → failure.                                                                    |

Lessons are optional: the sandbox is always available with every parameter
unlocked.

## 6. Keyboard shortcuts

`Space` pause · `.` single step · `R` reset · `↑/↓` setpoint ±0.1 m
(Shift: ±1 m) · `G` gust now · `S` snapshot · `1/2/3` simulation level ·
`C` cycle camera · `H` hide panels.
