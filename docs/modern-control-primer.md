# Modern Control Primer

The textbook of Part II ("Beyond PID"), the sequel to [pid-primer.md](pid-primer.md).
Each chapter matches a chapter of lessons (see
[beyond-pid.md §6](beyond-pid.md#6-lessons-part-ii)); the **Lesson** notes
point to the experiment that shows it.

---

## A. From knobs to models

### A.1 State

The **state** of a system is the smallest set of numbers that, together with
the future inputs, determines its whole future. For the L1 drone — a mass
pushed by a thrust — it is altitude and vertical velocity. With the error
$e = y - r$ (note the sign: deviation from the setpoint) the state is
$x = (e, \dot y)$ and a PD controller is already a **linear state feedback**:

$$
u = -K_p\,(y - r) - K_d\,\dot y = -\begin{pmatrix}K_p & K_d\end{pmatrix} x = -K x
$$

The **phase portrait** plots $x$ as a point in the $(e, \dot e)$ plane.
A step moves the point sideways; the controller then steers it back to the
origin. The shape of the path tells the damping at a glance: a spiral
(underdamped), a straight slide (critically damped), a slow crawl along a
line (overdamped).

Everything that follows answers one of two questions: **how to choose K**
(LQR, MPC, learning) and **how to know x** when the sensors are noisy, slow
or biased (Kalman filter, observers).

> **Lesson II.1 "State, not error"** — make the phase-plane trajectory reach
> the origin in < 1.5 s without crossing the vertical axis.

### A.2 The linear–quadratic regulator (LQR)

Instead of choosing gains, choose what you care about. For the discrete
linear model $x_{k+1} = A x_k + B u_k$ minimise

$$
J = \sum_{k=0}^{\infty} \big(x_k^\top Q\, x_k + u_k^\top R\, u_k\big)\,\Delta t
$$

The optimal controller is a state feedback $u = -Kx$ with

$$
K = (R + B^\top P B)^{-1} B^\top P A
$$

where $P$ solves the **discrete algebraic Riccati equation**

$$
P = Q + A^\top P A - A^\top P B\,(R + B^\top P B)^{-1} B^\top P A .
$$

`src/math/riccati.ts` solves it by simply iterating the right-hand side until
it stops changing (fine for the small systems here). The model is the
continuous one, $\hat m\,\ddot y = u$, discretised exactly at the controller
period with a zero-order hold (`c2d` in `src/math/mat.ts`, via the matrix
exponential).

What the weights mean, with $Q = \operatorname{diag}(q_e, q_v)$, $R = r$:

| Raise… | …and the controller                                 |
| ------ | --------------------------------------------------- |
| $q_e$  | hates position error: stiffer, faster, more thrust. |
| $q_v$  | hates speed: more damping, calmer.                  |
| $r$    | hates thrust: gentler, slower, less saturation.     |

Only the ratios matter — scaling all weights by the same factor leaves $K$
unchanged. For the continuous double integrator with $\hat m = 1$ there is a
closed form, which the tests use as a check:

$$
k_1 = \sqrt{q_e / r},\qquad k_2 = \sqrt{q_v / r + 2 k_1}
$$

$k_1$ is a $K_p$ and $k_2$ a $K_d$ (on measurement): **LQR does not replace
the PD, it tunes it** — from a statement of preferences, with guaranteed
stability for the model (continuous-time LQR even guarantees at least 60° of
phase margin, something hand-tuning never promises). The same recipe designs a 12-state, 4-input quadrotor controller,
where hand-tuning is hopeless.

The info card shows the gains and the closed-loop poles (continuous
equivalents $s = \ln z / \Delta t$ of the eigenvalues of $A - BK$), and the
dominant pair as $\omega_n$ and $\zeta$.

**Motor lag as a state.** Optionally the actual thrust $T$ becomes a third
state ($\hat\tau\,\dot T = u - T$). The controller then needs $T$: from motor
telemetry if available, else from a first-order model of the motors run
inside the controller.

> **Lesson II.2 "Pay for what you want"** — beat the PD ghost's settling time
> without using more peak thrust, by stating costs instead of gains.

### A.3 Integral action, the LQR way (LQI)

LQR is optimal _for the model_. With the wrong mass
($\hat m = 0.8$ kg for a 1 kg drone) the feedforward is short by
$(m - \hat m)\,g$ and the LQR droops by $(m-\hat m)\,g/k_1$ — the P-controller
droop of [pid-primer.md §4](pid-primer.md#4-why-p-only-leaves-the-drone-hanging-low)
again.

The fix: add $\xi = \int e\,dt$ as a state ($\dot\xi = e$) with its own weight
$q_i$. The augmented system is still linear; the Riccati equation now also
returns $k_i$, an integral gain. As with the PID, the integrator is held at
zero on the ground and during take-off, and stops integrating while the
output saturates and the error pushes further (conditional integration).

Integral action on a mass **must** overshoot after a step for the same
reason as in [pid-primer.md §4.1](pid-primer.md#41-why-the-integral-always-overshoots-here);
$q_i$ trades the speed of offset removal for that overshoot.

> **Lesson II.3 "Integral, the optimal way"**.

### A.4 The Kalman filter

**Problem.** A cheap barometer: 5 cm noise, 50 fixes per second, held
between fixes. Differentiating it (the D term) produces violent thrust
jitter; low-pass filtering trades noise for delay.

**Idea.** Use a model and a second sensor. The accelerometer is fast and
smooth, but integrated twice it drifts (and it has a bias). The barometer is
noisy but does not drift. Fuse them:

- state $x = (y, v, b)$ — altitude, vertical speed, accelerometer bias;
- **predict** every controller sample with the measured specific force
  $a_m$: $\hat x^- = F\hat x + G\,(a_m - g)$ with

$$
F = \begin{pmatrix}1 & \Delta t & -\tfrac12\Delta t^2\\ 0 & 1 & -\Delta t\\ 0 & 0 & 1\end{pmatrix},\quad
G = \begin{pmatrix}\tfrac12\Delta t^2\\ \Delta t\\ 0\end{pmatrix}
$$

and covariance $P^- = F P F^\top + Q$;

- **correct** whenever a new fix $y_m$ arrives:
  $K = P^- H^\top (H P^- H^\top + R)^{-1}$,
  $\hat x = \hat x^- + K\,(y_m - H\hat x^-)$, $H = (1\ 0\ 0)$
  (covariance update in Joseph form, which stays symmetric).

$Q$ comes from the assumed accelerometer noise (entering like an acceleration
input) and a slow random walk of the bias; $R$ from the assumed barometer
noise. Their ratio is what matters: **the Kalman gain is a trust setting.**
Over-trust the barometer ($R$ small) and the estimate is as noisy as the
sensor; distrust it and the estimate is smooth but slow to notice real
changes. Set the bias drift to zero and the filter never learns the bias.

The filter also reports its own uncertainty $\sqrt{P_{11}}$, drawn as a ±2σ
band next to the truth — honest only if the assumed noise matches the real
one.

In Anemostatos the filter sits in front of any L1 controller
(`control.l1.estimator = 'kalman'`): the PID, LQR or ADRC reads the
estimate instead of the raw sensors.

> **Lesson II.4 "Trust issues"** — same PID, same gains: 12.5 cm → 1.5 cm RMS
> error, 7.5 N → 0.9 N thrust jitter.

---

## B. Estimate the disturbance, cancel it

### B.1 Active disturbance rejection control (ADRC)

The I term handles unknown loads, but only after they have caused an error,
and slowly. ADRC (Han 2009, in the bandwidth-parameterised form of Gao 2003)
takes the opposite view: keep only the crudest model,

$$
\ddot y = b_0\,(u - u_{known}) + f,\qquad b_0 = 1/\hat m,
$$

and call **everything else** — wind, drag, the wrong mass, the motor lag,
even gravity if you leave out the feedforward — the **total disturbance** $f$.

**Extended state observer (ESO).** Treat $f$ as one more state (assumed
roughly constant) and estimate $z = (\hat y, \hat v, \hat f)$ from the
altitude alone:

$$
\dot z = \begin{pmatrix}0&1&0\\0&0&1\\0&0&0\end{pmatrix} z
+ \begin{pmatrix}0\\ b_0\\ 0\end{pmatrix} u
+ L\,(y - \hat y),\qquad
L = \begin{pmatrix}3\omega_o\\ 3\omega_o^2\\ \omega_o^3\end{pmatrix}
$$

which puts all three observer poles at $-\omega_o$. `src/estimation/eso.ts`
discretises it exactly (ZOH), so high bandwidths stay numerically stable.

**Control law.** Cancel the estimate and control the remaining double
integrator with a PD whose two poles sit at $-\omega_c$:

$$
u = u_{known} + \frac{\omega_c^2\,(r - \hat y) - 2\omega_c\,\hat v - \hat f}{b_0}
$$

Two knobs, both in rad/s, both with a physical meaning: how fast the loop
should respond ($\omega_c$) and how fast the observer should believe the
sensor ($\omega_o$).

**Anti-windup.** The observer must be fed the input that was _actually
applied_ (after saturation). Otherwise it would explain the thrust it did not
get as a disturbance, and $\hat f$ would wind up.

**Estimate vs truth.** The engine records the true disturbance as the
nominal model sees it, $\vec d = \hat m\dot{\vec v} - \vec T - \hat m\vec g$
([physics-model.md §4.6](physics-model.md#46-what-the-controllers-model-does-not-explain)).
ADRC publishes $\hat m\hat f$ converted to the same definition, so the two
can be plotted together. They differ by the observer's lag, and by the motor
lag (which is part of ADRC's $f$ but not of the physical $\vec d$).

> **Lesson II.5 "Everything is a disturbance"** — a 300 g payload lands on
> the drone: the observer sees it within a fraction of a second; the PID
> ghost's I term takes seconds (max error 12 cm vs 28 cm).

### B.2 The bandwidth trade-off

A faster observer catches disturbances sooner, but the same gain amplifies
sensor noise into the motors. There is a second limit: the model ignores the
30 ms motor lag. A model can be simple, but not simpler than the bandwidth
you ask of it — beyond $\omega_o \approx 60$ rad/s the loop degrades and then
falls apart. Rule of thumb: $\omega_o \approx 3$–$10\,\omega_c$, and
$\omega_c$ well below the actuator bandwidth $1/\tau_m$.

> **Lesson II.6 "How fast should you believe?"** — the score (RMS error in cm
>
> - 10 × thrust jitter in N) is U-shaped in $\omega_o$, with its minimum
>   around 10 rad/s.

### B.3 Incremental nonlinear dynamic inversion (INDI)

**Dynamic inversion** computes the input that produces a desired
acceleration from a model: $\tau = I\,\alpha_{des} + \omega\times I\omega -
\tau_{aero} - \dots$ Every term you forget becomes an error. **INDI** avoids
the model by writing the same equation _incrementally_ around the present
moment. Over one short sample the state barely changes, but the torque can
change a lot, so

$$
I\,\dot\omega \approx I\,\dot\omega_0 + (\tau - \tau_0)
\quad\Rightarrow\quad
\tau = \tau_0 + \hat I\,(\alpha_{des} - \dot\omega_0)
$$

with $\tau_0$ the torque the motors produce _now_ and $\dot\omega_0$ the
angular acceleration the drone has _now_. Everything else — gyroscopic
coupling, aerodynamic moments, a centre of mass that moved, a damaged prop —
is contained in the measured $\dot\omega_0$ and needs no model (Smeur, Chu &
de Croon 2016). What remains of the model is the **control effectiveness**:
here, the inertia $\hat I$ and the mixer geometry. A factor-2 error in
$\hat I$ only rescales the loop gain; around 3× over-estimate the loop gets
too aggressive.

The rate controller becomes
$\alpha_{des} = K\,(\omega_{sp} - \omega)$ — one gain per axis. In steady
state $\dot\omega_0 = \alpha_{des} = 0$ and the command equals $\tau_0$: the
incremental form behaves like an integrator with no windup (it always starts
from what the actuators actually do).

**Where $\tau_0$ comes from.** From the motors' _speeds_ (RPM telemetry,
mapped to the thrust a healthy motor would make), or, without telemetry,
from a first-order model of the motors driven by the commands. It must be the
actuator _state_, not the thrust: a chipped prop spins at the commanded speed
and pushes less, and that deficit must show up in $\dot\omega_0$ — not be
hidden in $\tau_0$, where the increment could never close the gap.

> **Lesson II.7 "Don't model it, measure it"** — the prop of motor 2 drops to
> 60 % in mid-air. PID cascade: 77 cm drift; rate INDI: 21 cm.

### B.4 Cascaded INDI on acceleration

The same trick one level up (Smeur, de Croon & Chu 2018; Tal & Karaman
2021): with the thrust vector $\vec F_0$ the motors produce now and the
acceleration $\vec a_0$ the accelerometer measures now,

$$
\vec F = \vec F_0 + \hat m\,(\vec a_{sp} - \vec a_0),
\qquad \vec a_0 = R\,\vec a_{meas} + \vec g .
$$

A gust is a force: the accelerometer feels it the instant it acts, long
before it has moved the drone far enough for a position or velocity error.
The velocity loop's integral becomes redundant (the increment already
integrates) and is bypassed. As a by-product,
$\hat m\vec a_0 - \vec F_0 - \hat m\vec g$ _is_ an estimate of the disturbance
force — INDI knows the disturbance without ever computing it.

Each INDI loop rejects disturbances at its own level. Measured in the
simulator:

| Disturbance             | PID   | rate INDI | acceleration INDI | both   |
| ----------------------- | ----- | --------- | ----------------- | ------ |
| broken prop (max error) | 77 cm | 21 cm     | 86 cm             | 3.8 cm |
| gusty wind (RMS error)  | 22 cm | 22 cm     | 5.9 cm            | 5.7 cm |

This is why research stacks put INDI beneath everything else, and why Sun et
al. (2022) found it decisive for both NMPC and flatness-based control.

> **Lesson II.8 "Feel the push"** — beat the PID cascade's RMS error on the
> gust-challenge wind by 40 % (acceleration INDI achieves ≈ 73 %).

### B.5 Keep the filters in sync

$\dot\omega_0$ comes from differentiating a noisy gyro, so it is low-pass
filtered, and a filter delays. $\tau_0$, if taken raw, is not delayed. INDI
subtracts one from the other: it then compares the torque of _now_ with the
acceleration of _a few tens of milliseconds ago_, keeps correcting for a
mismatch that no longer exists, and the loop wobbles. The fix, standard in
the INDI literature, is to pass $\tau_0$ through the **same** filter, so
both describe the same moment (the same applies to $\vec F_0$ and $\vec a_0$).

With a fast filter the mismatch is small and harmless; the bug bites exactly
when noise forces a slow one. In the simulator, with 10 °/s gyro noise and a
5 Hz filter: unsynchronised 21° RMS attitude error and 0.54 N motor jitter,
synchronised 7.4° and 0.08 N. Raising the cutoff to escape instead lets the
noise through to the motors.

> **Lesson II.9 "Keep your filters in sync"**.

---

## C. Geometry and trajectories

### C.1 Attitude lives on a sphere

An attitude is a rotation, an element of the rotation group SO(3) — a curved
space, not three independent number lines. Euler angles (roll, pitch, yaw)
are a _chart_ of it: convenient, but singular. With this project's order
(yaw, then pitch, then roll) the chart folds over at pitch = ±90°: roll and
yaw stop being distinguishable (gimbal lock) and jump by 180° as the drone
passes through.

The naive attitude controller subtracts angles,
$\omega_{cmd} = K\,(\eta_{sp} - \eta)$, and sends them as body rates. For small
errors that is fine — all laws agree to first order. After a large upset it
is wrong twice: angle differences are not body rates, and near the
singularity they point the wrong way.

Rotation-based laws compute the error on the group itself. With the error
quaternion $q_e = q^{-1}\otimes q_{sp}$ (shortest way round:
$q_{e,w}\ge 0$), Part I already used $\omega_{cmd} = K\cdot 2\,\vec q_{e}$, the
error along the shortest rotation. The **tilt-prioritised** law
(Brescianini & D'Andrea 2020) splits $q_e$ into a tilt part — rotate the
thrust axis to where it must point — and a yaw part about that axis, with
separate gains: yaw is the least important degree of freedom of a
multicopter and must never steal authority from the thrust direction.
Lee et al.'s geometric controller (2010) states the same idea with rotation
matrices, $e_R = \tfrac12(R_d^\top R - R^\top R_d)^\vee$, and proves
almost-global stability.

> **Lesson II.10 "Flying on a sphere"** — a 170° flip at 6 m: the Euler law
> loses 2.6 m and needs 1.4 s to level, the rotation-based laws 1.4 m and
> 0.4 s. (Tilt-prioritised and quaternion behave alike here; the split pays
> off with large yaw errors.)

### C.2 Feedback is always late

A feedback controller acts on an error, so for a moving reference it needs an
error to act at all. For a 2 m/s figure-8 even a well-tuned loop lags and cuts
the corners: ≈ 80 cm RMS with the geometric law and no feedforward, 122 cm
for the Part I cascade.

The geometric tracking law closes position and velocity in one expression
and adds the reference's own acceleration:

$$
\vec a = \vec a_{ref} - K_p\,\vec e_p - K_v\,\vec e_v\quad(-\,K_i\!\int\vec e_p),
\qquad \vec e_p = \vec p - \vec p_{ref},\ \vec e_v = \vec v - \vec v_{ref}
$$

With the plant $\ddot{\vec p} = \vec a$ the error obeys
$\ddot{\vec e} + K_v\dot{\vec e} + K_p\vec e = 0$ whatever the trajectory — a
mass–spring–damper again, with $\omega_n = \sqrt{K_p}$ and
$\zeta = K_v/(2\sqrt{K_p})$ (shown in the info card).

### C.3 Differential flatness

A quadrotor is **differentially flat** (Mellinger & Kumar 2011): position
and yaw, with their derivatives, determine the whole state and the inputs.

- acceleration ⇒ thrust vector $\vec F = \hat m(\vec a + \vec g)$ ⇒ attitude
  (thrust axis $b = \vec F/|\vec F|$, plus yaw) and collective thrust;
- jerk ⇒ how fast the thrust axis turns: differentiating $b$,
  $$\dot b = \frac{\hat m}{T}\big(j - (j\cdot b)\,b\big),\qquad \omega_\perp = b\times\dot b$$
  — body-rate feedforward;
- snap ⇒ angular acceleration ⇒ torque feedforward (not used here: the rate
  loop, especially with INDI, absorbs it).

So a smooth reference gives the inner loops their commands _before_ any error
appears; feedback only corrects what the plan could not foresee.

> **Lesson II.11 "Feedforward from the future"** — switch the feedforward on:
> 78 cm → 2 cm RMS on the 2 m/s figure-8.

### C.4 Minimum-snap trajectories

If the reference must be smooth, which smooth path? Snap (4th derivative) is
what the motors feel — it maps to the rate of change of thrust and torque —
so Mellinger & Kumar minimise

$$
J = \int_0^T \big\|\,\ddddot{p}(t)\,\big\|^2\,dt
$$

subject to passing the waypoints at given times. The minimiser is a
piecewise 7th-order polynomial, $C^6$ at the waypoints; with position, rest
conditions and continuity the coefficients follow from **one linear solve**
(`src/math/poly.ts`). Anemostatos flies a square with one rest-to-rest
segment per side (straight edges; passing the corners without stopping
bulges far outside the square).

Honest comparison with simply stepping the setpoint to each corner, 2 s per
side and a good controller: the steps arrive about as soon (1.84 s vs
1.80 s). What planning buys is

- **smoothness** — half the peak body rate (130 vs 249 °/s), less thrust
  chatter;
- **a schedule** — the drone is where the plan says, at every moment
  (5 cm RMS), which is what coordinated flights and cameras need;
- **feasibility in advance** — the plan's peak acceleration $a$ requires a
  tilt of $\arctan(a/g)$. A 7 s lap needs 45° against a 35° limit: the info
  card flags it before take-off.

> **Lesson II.12 "Plan the motion"**.

---

## D. Optimisation in the loop

### D.1 Model predictive control

Part I handled limits after the fact: saturate, then stop the integrator from
winding up. **MPC** plans with them. At every step it solves

$$
\min_{u_0,\dots,u_{N-1}} \sum_{k=1}^{N} q_p\,(p_k - r_k)^2 + q_v\,(v_k - v_{r,k})^2 + r\,u_{k-1}^2
\quad\text{s.t.}\quad u_{min} \le u_k \le u_{max},\ \ p_k \le p_{max},\ \dots
$$

over a model, applies only $u_0$, and solves again at the next step with the
new measurement (**receding horizon**) — feedback through re-planning.

For a linear model the predictions are affine in the inputs:
$p = \bar p + G_p U$, $v = \bar v + G_v U$ with $\bar p_k = p_0 + k\,\Delta t\,v_0$
and $G_{p,kj} = b\,\Delta t^2 (k - j - \tfrac12)$ for $j < k$. The cost becomes
$\tfrac12 U^\top H U + f^\top U$ and the limits become linear inequalities in
$U$: a **quadratic program** ("condensed" MPC).

Anemostatos solves it with ADMM as in OSQP (Stellato et al. 2020): a
factorisation computed once and reused, cheap iterations, warm start from the
previous plan, and an **iteration cap** — so the computing time is bounded and
the run deterministic (the µs/step meter shows the cost).

> **Lesson II.13 "Look before you leap"** — ceiling at 4 m, setpoint 3.95 m:
> the PID spends 8.5 s above the ceiling, the MPC never touches it and
> arrives sooner.

### D.2 The horizon

The horizon must cover what the system needs to react — here, to brake from
climbing speed. Shorter plans are cheaper but **myopic**: with hard
constraints in every plan the drone stays safe, but with little to gain
within a short window the climb becomes slow (0.1 s horizon: 6.2 s to
arrive; 1 s: 1.3 s). Longer horizons cost more (the QP grows with $N$) and
soon buy nothing.

> **Lesson II.14 "How far ahead?"**

### D.3 MPC on top, INDI below

On the quadrotor, the MPC runs as the outer stage: three axis QPs on
$\ddot p = a$ with the tilt limit as a bound on horizontal acceleration,
previewing the planned trajectory. It is a _linear_ MPC on a _point-mass_
model: good on feasible trajectories (it beats geometric + flatness on a
6.5 s figure-8, 1.8 vs 3.8 cm), not beyond the drone's limits, where the
attitude dynamics it ignores dominate. Full nonlinear MPC (acados, Sun et al. 2022) models those too.

What carries over from Sun et al. is the other finding: **the inner loop
matters most.** In gusts, MPC alone tracks to 4.3 cm, MPC + INDI to 1.6 cm,
geometric + INDI to 1.3 cm.

> **Lesson II.15 "MPC on top, INDI below"**

### D.4 MPPI — planning by sampling

Some costs are not convex: "stay out of that pillar" allows passing left
_or_ right, and no QP can express the "or". **Model predictive path
integral** control (Williams et al. 2017) does not optimise, it samples:

1. perturb the current plan $U$ with Gaussian noise $K$ times;
2. roll each perturbed sequence out on a model; score it with any cost
   (here: tracking, effort, a heavy penalty inside a zone);
3. new plan $U \leftarrow U + \sum_i w_i\,\varepsilon_i$ with
   $w_i \propto e^{-S_i/\lambda}$;
4. apply the first input, shift the plan, repeat.

The **temperature** $\lambda$ sets how many samples count: small follows the
best sample (decisive), large averages many — and the average of "left of
the pillar" and "right of the pillar" is "into the pillar": the drone
hesitates in front. It costs many rollouts (256 × 30 steps per plan here,
≈ 10 % of a CPU core in real time), but they are independent, which is why
MPPI runs on GPUs in racing and off-road driving.

> **Lesson II.16 "A thousand futures"**

### D.5 Control barrier functions

A **safety filter** sits between any controller and the actuators and
changes the command as little as possible to stay in a safe set
$\{h(p) \ge 0\}$ (Ames et al. 2019). For a pillar,
$h = |p_{xz} - c|^2 - R^2$. The acceleration affects $h$ only through
$\ddot h$ (relative degree 2), so the condition is

$$
\ddot h + 2\gamma\,\dot h + \gamma^2 h \ge 0
\quad\Longleftrightarrow\quad
2\,(p - c)\cdot a \;\ge\; -2|v|^2 - 2\gamma\dot h - \gamma^2 h,
$$

linear in $a$ — and the filter is the tiny QP
$\min |a - a_{des}|^2$ subject to all such constraints. Far from the boundary
it does nothing; near it, it removes exactly the part of the command that
would approach too fast ($\gamma$ sets how fast is too fast).

Its limit is equally instructive: a CBF guarantees **safety, not progress**.
With the target behind the pillar, the filtered drone stops in front of it
and waits (a deadlock) — going around is a planning problem (MPPI), and the
filter stays as the last line of defence, whatever flies above it.

> **Lesson II.17 "The safety filter"**

---

## E. Adapting and learning

### E.1 L1 adaptive control

Adaptive control estimates what the model gets wrong while flying. Classical
model-reference adaptive control has a catch: faster adaptation means higher
gain, and high adaptation gain means poor robustness and oscillation. **L1
adaptive control** (Hovakimyan & Cao 2010) separates the two:

- a **state predictor** runs beside the drone,
  $\hat m\,\dot{\hat v} = F_{known} + \hat\sigma + \hat m\,a_s(\hat v - v)$;
- a **piecewise-constant adaptation law** sets $\hat\sigma$ every sample to
  the value that makes the predictor agree with the measurement,
  $\hat\sigma = -\hat m\,\Phi^{-1} e^{a_sT}(\hat v - v)$,
  $\Phi = (e^{a_sT} - 1)/a_s$ — as fast as the sample rate allows;
- a **low-pass filter** $C(s)$ decides how much of $\hat\sigma$ reaches the
  actuators: $u = u_{baseline} - C(s)\,\hat\sigma$.

Adaptation speed is now free; robustness (and noise) is set by $C(s)$ alone.
Anemostatos uses the velocity-predictor form of L1Quad (Wu et al. 2025), on
the altitude loop and per world axis on the quadrotor.

> **Lesson II.18 "Adapt fast, act smooth"** — a payload with a noisy velocity
> sensor: max error 3.9 cm (ADRC 12.2, PID 27.7); open the filter and the
> noise goes into the motors.

### E.2 Learning the shape of a disturbance

Integrators, observers and L1 all estimate the disturbance _as it is now_.
When it depends on the state — aerodynamic drag on a drone flying a
trajectory through wind — a **model** of how it varies can predict it
instead: $f \approx \Phi(x)\,\theta$. Neural-Fly (O'Connell et al. 2022)
meta-learns the basis $\Phi$ offline with a neural network across wind
conditions, so that online only the linear coefficients $\theta$ adapt.

Anemostatos hand-crafts $\Phi$ from the physics — per axis
$[1,\ v,\ |v|v,\ T\,v_\perp]$ (steady part, linear and quadratic drag, rotor
drag) — and fits $\theta$ by **recursive least squares** with forgetting to
the residual force measured by the accelerometer. RLS needs one practical
fix: without excitation (hovering) its covariance grows without bound
(covariance windup) and the fit becomes jumpy, so the covariance is capped.

Measured honestly: on a fast figure-8 through wind it beats the integrator
≈ 4× (14 → 3 cm); with this simulator's clean accelerometer the fast
observers (L1, INDI) do better still (≈ 2 cm). With poor sensors all three
track alike, and the learned model gives by far the smoothest thrust — a good
model lets you adapt slowly, and slow is smooth.

> **Lesson II.19 "Learn the shape of the wind"**

### E.3 Learned policies

A policy maps observations directly to commands; nobody writes its control
law. Anemostatos trains a small MLP (15 → 32 → 32 → 4, observations: errors,
attitude axes, body rates; actions: collective thrust + body rates, the
interface of Kaufmann et al. 2023) in two stages, with `make train`:

1. **Imitation learning** — fly the geometric controller in randomised
   conditions and fit the network to its commands (supervised, backprop +
   Adam).
2. **Evolution strategies** — a gradient-free form of reinforcement learning:
   perturb the weights, fly each variant, move towards the better ones (rank
   shaping, antithetic samples, common random seeds).

The result flies, reacts quickly, and shows the classic weaknesses: it hunts
around its targets in a small limit cycle (nothing rewarded perfect
stillness, and it has no memory), and it degrades outside its training
distribution. Careful comparisons (Kunapuli et al. 2025) find the same
split: learned controllers can win transients and hard-to-model problems;
classical ones keep their guarantees and steady state. Champion-level racing
policies (Kaufmann et al. 2023) are far larger and trained on vastly more
experience; differentiable simulation (Heeg et al. 2025) makes such training
much more sample-efficient.

> **Lesson II.20 "A controller nobody designed"**

### E.4 The arena

Every controller of Part II flies the same seeded scenarios; accuracy, motor
smoothness and computing cost are measured for all
([arena.md](arena.md), lesson II.21). No controller wins every column. An
INDI inner loop (under MPC or geometric control) dominates the disturbance
scenarios; the PID has the calmest motors because it is sluggish; the policy
is the cheapest to evaluate; MPPI pays for its generality. Choosing a
controller is choosing which column matters.

> **Lesson II.21 "The grand comparison"**

---

## Further reading

The references are collected in [beyond-pid.md §8](beyond-pid.md#8-references).
For this part in particular: Han 2009 and Gao 2003 (ADRC); Smeur et al. 2016
and 2018, Tal & Karaman 2021 (INDI); Lee, Leok & McClamroch 2010, Brescianini
& D'Andrea 2020, Mellinger & Kumar 2011 (geometry, flatness, min-snap);
Stellato et al. 2020, Williams et al. 2017, Ames et al. 2019 (QP, MPPI, CBF);
Hovakimyan & Cao 2010, Wu et al. 2025, O'Connell et al. 2022, Kaufmann et al. 2023,
Kunapuli et al. 2025 (adaptive and learned control); any control
textbook's chapter on LQR and Kalman filtering (e.g. Åström & Murray,
_Feedback Systems_, chapters on state feedback and output feedback, freely
available online).
