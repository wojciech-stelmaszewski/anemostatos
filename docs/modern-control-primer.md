# Modern Control Primer

The textbook of Part II ("Beyond PID"), the sequel to [pid-primer.md](pid-primer.md).
Each chapter matches a chapter of lessons (see
[beyond-pid.md §6](beyond-pid.md#6-lessons-part-ii)); the **Lesson** notes
point to the experiment that shows it. Chapters C–E are written as their
milestones land.

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

## Further reading

The references are collected in [beyond-pid.md §8](beyond-pid.md#8-references).
For this part in particular: Han 2009 and Gao 2003 (ADRC); Smeur et al. 2016
and 2018, Tal & Karaman 2021 (INDI); any control
textbook's chapter on LQR and Kalman filtering (e.g. Åström & Murray,
_Feedback Systems_, chapters on state feedback and output feedback, freely
available online).
