# Physics Model

The physics must be **simple enough to reason about** and **rich enough to
make control interesting**. Every parameter listed here is exposed in the UI
(grouped under "Physics" and "Wind") unless noted otherwise.

## 1. Units and conventions

- SI units everywhere: metres, seconds, kilograms, newtons, radians
  (UI shows degrees where friendlier).
- **World frame:** right-handed, **Y up** (matches Three.js), X and Z
  horizontal. The floor is the plane $y = 0$. Origin at the grid centre.
- **Body frame:** origin at the drone's centre of mass, $+x_b$ forward,
  $+y_b$ up (thrust direction), $+z_b$ right.
- Attitude stored as a unit **quaternion** $q$ (body → world). Euler angles
  (roll, pitch, yaw) are derived only for display and for the attitude
  setpoints.
- Gravity $\vec g = (0, -9.81, 0)\ \text{m/s}^2$, constant.

## 2. Simulation levels

The project grows through three levels of fidelity. All share the wind
model, integrator and telemetry; they differ in what the "plant" is.

| Level | Plant | State | Control input | Purpose |
| --- | --- | --- | --- | --- |
| **L1 — Altitude** | 1D point mass on the vertical axis | $y, \dot y$ | collective thrust $T$ | Pure, clean PID lessons. Drone is locked horizontally and level. |
| **L2 — Point mass 3D** | 3D point mass with a force vector | $\vec p, \vec v$ | force $\vec F$ (3 axes, limited) | Three independent PIDs; shows wind from any direction. Physically a "cheat" (drones can't push sideways), clearly labelled as such. |
| **L3 — Quadrotor** | 6-DoF rigid body with 4 motors | $\vec p, \vec v, q, \vec\omega$, motor speeds | 4 motor commands | Realistic: tilt to move, cascaded PID, mixer, motor saturation. |

In L1 and L2 the drone is still *rendered* as a full quadrotor (props
spinning according to thrust), just with constrained motion.

## 3. Drone parameters (defaults)

Roughly a 5-inch-class / small camera drone.

| Parameter | Symbol | Default | Notes |
| --- | --- | --- | --- |
| Mass | $m$ | 1.0 kg | Changeable at runtime (e.g. "add payload"). |
| Arm length (centre → motor) | $L$ | 0.17 m | X configuration. |
| Inertia | $I_{xx}, I_{yy}, I_{zz}$ | 0.0082, 0.0149, 0.0082 kg·m² | $I_{yy}$ = yaw axis. |
| Max thrust per motor | $f_{max}$ | 6.1 N | Thrust-to-weight ≈ 2.5. |
| Motor time constant | $\tau_m$ | 0.03 s | First-order lag. |
| Rotor torque coefficient | $c_\tau$ | 0.016 m | Yaw reaction torque $= c_\tau f_i$. |
| Linear drag, horizontal | $c_{h}$ | 0.05 N·s²/m² | Quadratic drag. |
| Linear drag, vertical | $c_{v}$ | 0.10 N·s²/m² | |
| Angular damping | $c_\omega$ | 0.002 N·m·s | Keeps L3 numerically tame. |

## 4. Equations of motion

### 4.1 Motors (all levels)

Each motor $i$ has a commanded thrust $f_i^{cmd} \in [0, f_{max}]$ and an
actual thrust following a first-order lag:

$$
\dot f_i = \frac{f_i^{cmd} - f_i}{\tau_m}
$$

In L1 the four motors receive the same command ($f_i^{cmd} = T/4$). In L2
the motors are purely decorative (the force vector is applied directly, with
the same lag and limits per axis).

Propeller visual spin rate is derived from $\omega_i \propto \sqrt{f_i}$.

### 4.2 Translation

$$
m\,\dot{\vec v} = R(q)\begin{pmatrix}0\\ \sum f_i\\ 0\end{pmatrix}
+ m\vec g + \vec F_{drag} + \vec F_{ext}
$$

- $R(q)$ — rotation body→world (identity in L1).
- $\vec F_{ext}$ — manual "poke" impulses from the UI.
- Aerodynamic drag uses the velocity **relative to the air**:

$$
\vec v_{rel} = \vec v - \vec w,\qquad
F_{drag,j} = -c_j\,|v_{rel,j}|\,v_{rel,j}\quad (j \in \{x,y,z\},\ c_y = c_v,\ c_x = c_z = c_h)
$$

  This is how wind acts on the drone: a hovering drone ($\vec v = 0$) in wind
  $\vec w$ feels a force *along* the wind, growing with its square.

### 4.3 Rotation (L3 only)

Motor layout (X configuration, seen from above, $+x_b$ forward):

```
        front
   M1 (CW)   M2 (CCW)
        \   /
         [ ]
        /   \
   M4 (CCW)  M3 (CW)
        rear
```

With $d = L/\sqrt2$, motor positions in the body frame are
M1 $(d, 0, -d)$, M2 $(d, 0, d)$, M3 $(-d, 0, d)$, M4 $(-d, 0, -d)$.
Each motor's torque is $\vec r_i \times (0, f_i, 0) = (-r_{z,i} f_i,\ 0,\ r_{x,i} f_i)$,
plus the rotor reaction torque about $y_b$ (a CW rotor seen from above
pushes the body CCW, i.e. $+y_b$):

$$
\begin{aligned}
\tau_x\ (\text{roll}) &= d\,(f_1 + f_4 - f_2 - f_3) \\
\tau_z\ (\text{pitch}) &= d\,(f_1 + f_2 - f_3 - f_4) \\
\tau_y\ (\text{yaw}) &= c_\tau\,(f_1 - f_2 + f_3 - f_4)
\end{aligned}
$$

Positive $\tau_x$ lifts the left side (the drone rolls right), positive
$\tau_z$ lifts the nose. These signs are pinned down by unit tests.

Euler's rotation equation:

$$
I\,\dot{\vec\omega} = \vec\tau - \vec\omega \times (I\vec\omega) - c_\omega\,\vec\omega
$$

Quaternion kinematics:

$$
\dot q = \tfrac12\, q \otimes (0, \vec\omega)
$$

### 4.4 Ground

- If $y \le 0$ and $\dot y < 0$: clamp $y = 0$, $\dot y = 0$, apply strong
  horizontal friction, and (L3) force the attitude level. The drone is then
  *landed*.
- Simulation starts landed with motors idle; "Take off" enables the
  controller with the initial setpoint.
- A crash (hitting the ground with $|\dot y| > 3$ m/s or tilt > 80°) marks
  the run as crashed and offers a reset — failures are part of learning.

## 5. Wind model

Wind is a 3D air-velocity field $\vec w(t)$, uniform in space (the drone is
small; spatial variation adds nothing educationally). It is the sum of three
components:

$$
\vec w(t) = \vec w_{mean} + \vec w_{turb}(t) + \vec w_{gust}(t)
$$

### 5.1 Mean wind

Constant vector set by the user: speed (0–15 m/s), heading, and optional
vertical component. A constant wind is the perfect demonstration for the
**I term**: it is a constant disturbance, just like gravity.

### 5.2 Turbulence — Ornstein–Uhlenbeck process

Continuous, correlated random fluctuation, independently per axis:

$$
dw_{turb} = -\frac{w_{turb}}{\tau_w}\,dt + \sigma_w\sqrt{\frac{2}{\tau_w}}\,dW
$$

- $\sigma_w$ — intensity (standard deviation, m/s), default 0.8.
- $\tau_w$ — correlation time, default 1.5 s (smaller = more "jittery").

Discretised exactly with step $\Delta t$:

$$
w_{k+1} = w_k\, e^{-\Delta t/\tau_w} + \sigma_w\sqrt{1 - e^{-2\Delta t/\tau_w}}\;\mathcal N(0,1)
$$

### 5.3 Discrete gusts — "1 − cosine" profile

The classic aviation gust shape. Gusts arrive as a Poisson process with rate
$\lambda$ (default: one every 6 s on average). Each gust draws a random
direction (horizontal, with a configurable vertical share), peak amplitude
$A$ (default range 2–8 m/s) and duration $T_g$ (0.5–3 s):

$$
w_{gust}(t) = \hat d\,\frac{A}{2}\left(1 - \cos\frac{2\pi (t - t_0)}{T_g}\right),
\quad t_0 \le t \le t_0 + T_g
$$

Overlapping gusts add up.

### 5.4 Manual disturbances

- **"Gust now"** button — trigger a gust with chosen direction/amplitude.
- **Poke** — click-and-drag on the drone to apply a short impulse.
- **Payload** — change mass mid-flight (step disturbance for the I term).

### 5.5 Reproducibility

All randomness (turbulence, gust timing/shape, sensor noise) comes from a
single **seeded PRNG** (e.g. `mulberry32`/`sfc32`). The seed is shown in
the UI and can be fixed. Same seed + same parameters ⇒ identical wind ⇒ two
controller tunings can be compared fairly on the exact same disturbance.

## 6. Sensors

The controller never reads the true state directly; it reads a **sensor
model**:

| Option | Default | Effect |
| --- | --- | --- |
| Gaussian noise σ (per signal) | off (altitude 0.02 m when on) | Shows the D-term noise problem. |
| Bias | off | Constant offset; shows that feedback can't fix a wrong sensor. |
| Delay | 0 ms | Transport delay; shows loss of stability margin. |
| Sample rate | = controller rate | Sample-and-hold between samples. |

In L3 the sensors provide position, velocity, attitude and body rates (an
idealised "perfect state estimator" + optional noise). State estimation is
explicitly out of scope.

## 7. Numerical integration

- **Fixed physics step** $\Delta t_{phys}$ = 1 ms (1 kHz). Fast enough to
  resolve motor lag (30 ms) and rotational dynamics with a simple method.
- **Semi-implicit (symplectic) Euler**: update velocity from forces, then
  position from the *new* velocity; same for $\vec\omega$ and $q$.
  Quaternion renormalised every step.
- The controller runs every $n$ physics steps
  ($\Delta t_{ctrl} = n\,\Delta t_{phys}$), holding its output constant in
  between (zero-order hold) — exactly like a real flight controller.
- Rendering is decoupled (see
  [software-architecture.md](software-architecture.md#simulation-loop)).
- Integrator choice is isolated behind an interface so RK4 can be swapped in
  and compared if ever needed.
