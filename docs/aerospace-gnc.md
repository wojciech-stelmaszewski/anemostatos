# Parts IV and V — Aerospace GNC and Engineering Practice

> **Status (2026-10-03):** all of Parts IV and V are built (M19–M26). §5
> lists what was built and where it differs from this plan. Milestones
> M19–M26 in [roadmap.md](roadmap.md) point here. Part IV depends on Part III
> ([analysis.md](analysis.md)): its lessons use the Bode, Nyquist, pole,
> dispersion and covariance views built there.

Parts I–III teach control on one vehicle, a quadrotor. That is the right
choice for learning, and the wrong one for someone who wants to work on
aircraft, launch vehicles or spacecraft. A quadrotor near hover is almost
linear and time-invariant, has no aerodynamics worth the name, no
flexible modes and no fuel. The problems that fill an aerospace control
engineer's day are missing from it.

**Part IV** brings three more vehicles into the same simulator and teaches
the theory that they need and the quadrotor does not: optimal control from
first principles, guidance, gain scheduling, flexible structures,
spacecraft attitude control and advanced estimation. **Part V** is about
the work around the mathematics: requirements, verification, software and
redundancy.

This changes one rule of the project. Until now the non-goal was "the
world stays a grid, and there is one drone". From Part IV on it is "one
simulator, one engine, several vehicles". The grid, the charts, the
lesson runner, the probe and the Monte Carlo runner stay the same.

## 1. What the first three parts cannot teach

| Topic                                         | Why the quadrotor cannot show it                                  | Vehicle that can                  | Chapter |
| --------------------------------------------- | ----------------------------------------------------------------- | --------------------------------- | ------- |
| Pontryagin's principle, bang-bang control     | Hover has no fuel to save and no deadline to meet.                | Rocket landing (1D, then 3D)      | K       |
| Dynamic programming, the value function       | Can be shown on L1, and was not planned.                          | L1 drone, rocket                  | K       |
| Trajectory optimisation, convex guidance      | Min-snap planning (II.12) ignores thrust limits and mass.         | Rocket landing                    | K       |
| Unstable airframes, conditional stability     | The drone is a chain of integrators, not an unstable pole.        | Launch vehicle in the pitch plane | L       |
| Flexible modes, slosh                         | The drone is rigid.                                               | Launch vehicle, satellite         | L, N    |
| Non-minimum-phase response                    | None in the drone beyond a Padé delay.                            | Launch vehicle, aircraft          | L, M    |
| Flight dynamics and its modes                 | No wing, no angle of attack.                                      | Aircraft, longitudinal            | M       |
| Gain scheduling                               | The drone's dynamics do not change with speed.                    | Aircraft across the envelope      | M       |
| Handling qualities, pilot-induced oscillation | There is no pilot and no standard to meet.                        | Aircraft with a pilot model       | M       |
| Attitude control without gravity              | The drone's attitude loop exists to point the thrust.             | Satellite                         | N       |
| Momentum exchange, on–off actuators           | Motors are proportional actuators.                                | Satellite                         | N       |
| Guidance laws                                 | The drone tracks a given reference; it never decides where to go. | Interceptor, rendezvous, landing  | K, O    |
| UKF, particle filters, smoothing              | The drone's measurements are close to linear.                     | Range-only and bearing-only fixes | O       |

## 2. The pedagogical arc

Numbering restarts at IV.1 and V.1.

| Chapter                               | The question it answers                                                     | New ideas                                                                                                                                                          |
| ------------------------------------- | --------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| K. The best possible path             | "LQR gave me a gain. Where did it come from, and what if the cost is fuel?" | Calculus of variations, Pontryagin's principle, bang-bang control, dynamic programming, direct trajectory optimisation, convex powered-descent guidance            |
| L. A rocket is not a drone            | "Why does a launch vehicle need a gain margin in both directions?"          | Unstable plants and minimum bandwidth, thrust-vector control, bending modes, slosh, phase and gain stabilisation                                                   |
| M. Wings                              | "Why does an airliner have hundreds of gains and the drone a dozen?"        | Trim, linearisation, short period and phugoid, stability augmentation, handling qualities, gain scheduling, dynamic inversion, rate limits and PIO, energy control |
| N. Pointing in space                  | "There is nothing to push against. How do you turn?"                        | Quaternion feedback, reaction wheels and momentum management, on–off thrusters, flexible appendages, input shaping, star-tracker filtering                         |
| O. Guidance and navigation            | "Tracking a given reference is control. Who decides the reference?"         | Proportional navigation, relative motion, inertial navigation errors, UKF, particle filter, smoothing                                                              |
| P. From whiteboard to flight (Part V) | "The design works. What else must be true before it flies?"                 | Requirements, error budgets, verification campaigns, software implementation, redundancy, mode logic, certification                                                |

## 3. Architecture changes

### 3.1 A vehicle abstraction

Today `Level = 1 | 2 | 3` selects how much of the quadrotor is simulated,
and `sim/dynamics.ts` knows about four motors. Part IV puts a vehicle
interface in front of it:

```ts
interface Vehicle<State, Input> {
  id: 'quadrotor' | 'rocket' | 'aircraft' | 'satellite';
  initial(p: Params): State;
  step(s: State, u: Input, env: Environment, dt: number, p: Params): Forces;
  measure(s: State): Measurement;
  trim(p: Params, condition: TrimCondition): { state: State; input: Input };
  linearize(p: Params, at: { state: State; input: Input }): { A: Mat; B: Mat };
}
```

```
src/sim/vehicles/
  quadrotor.ts   today's dynamics.ts, moved behind the interface; levels 1–3 unchanged
  rocket.ts      point mass with variable mass (1D and 3D); planar rigid body with a gimballed
                 engine, aerodynamic normal force, one bending mode and one slosh pendulum
  aircraft.ts    3-DoF longitudinal (speed, angle of attack, pitch rate, pitch angle) with elevator
                 and throttle; lateral-directional as a second step
  satellite.ts   rigid body in torque-free space with reaction wheels, thrusters, a magnetic field
                 and one flexible appendage
src/sim/atmosphere.ts   density and speed of sound against altitude; dynamic pressure
```

`trim` and `linearize` are what make gain scheduling and the mode analysis
of Chapter M possible, and `analysis/linearize.ts` from Part III already
provides the finite-difference fallback. The quadrotor keeps its regression
baseline: moving it behind the interface must not change one number in
`tests/fixtures/baseline.json`.

The scene gets one model per vehicle (`scene/vehicles/`), each procedural
like the drone. The grid, the camera modes, the wind particles and the
trail stay. The satellite replaces the ground with a star field, and the
rocket adds a landing pad.

### 3.2 New mathematics

```
src/math/
  socp.ts        small second-order cone programs (primal-dual interior point), for convex guidance
  nlp.ts         sequential quadratic programming on top of the existing qp.ts, for trajectory optimisation
  ode.ts         fixed-step RK4 with sensitivities, for shooting and collocation defects
  grid.ts        value iteration on a regular grid with interpolation, for dynamic programming
```

`qp.ts` from Part II (M11) stays the workhorse. `socp.ts` is the one
genuinely new solver, and the largest technical risk of Part IV (§6).

### 3.3 New controllers, guidance and estimation

```
src/control/
  bangbang.ts     time-optimal and fuel-optimal switching laws for the double integrator and the rocket
  dp.ts           a policy read off a value-function grid
  schedule.ts     gain tables over a scheduling variable, with interpolation and the hidden-coupling terms
  tvc.ts          thrust-vector control with bending filters
  sas.ts          pitch damper, C*-type control law, dynamic inversion for the aircraft
  tecs.ts         total-energy control of speed and altitude
  quatfb.ts       quaternion feedback regulator, eigenaxis slews
  wheels.ts       reaction-wheel steering, momentum dumping with thrusters or magnetorquers
  pwpf.ts         on–off thruster modulation
  shaper.ts       input shaping for flexible modes
src/guidance/
  pdg.ts          powered-descent guidance: fuel-optimal landing as a convex problem
  collocation.ts  direct collocation for general trajectory optimisation
  pronav.ts       proportional navigation and its augmented form
  cw.ts           Clohessy–Wiltshire relative motion for rendezvous
src/estimation/
  ukf.ts          unscented Kalman filter
  particle.ts     bootstrap particle filter
  smoother.ts     Rauch–Tung–Striebel smoother
  ins.ts          strapdown inertial navigation and its error model
```

### 3.4 New visualisations

- **Costate and Hamiltonian** charts next to the state, and the
  **switching curve** in the phase portrait.
- **Value function** as a heat map over the phase plane, with the optimal
  policy as arrows.
- **Planned trajectory with its constraints**: the glide-slope cone, the
  thrust bounds, the pointing limit.
- **Mode shapes**: the bending rocket and the waving solar panel drawn
  with their deflection exaggerated.
- **Envelope chart**: altitude against speed with the trim points and the
  scheduled gains, and the poles at each point.
- **Handling-qualities chart**: short-period frequency and damping against
  the level boundaries of the standard.
- **Momentum chart**: wheel speeds against their limits.
- **Engagement geometry**: line of sight, closing velocity, miss distance.

## 4. Designs (short)

Full derivations go into `docs/aerospace-primer.md`, written alongside the
code.

### Chapter K — The best possible path

**The maximum principle.** Minimise $\int L(x,u)\,dt$ subject to
$\dot x = f(x,u)$. Introduce the costate $\lambda$ and the Hamiltonian
$H = L + \lambda^\top f$. Along an optimal path $\dot\lambda = -\partial
H/\partial x$ and $u$ minimises $H$ at every instant [Bryson 1975]. For a
quadratic cost and linear dynamics this gives $u = -R^{-1}B^\top\lambda$
and $\lambda = Px$, which is the LQR of lesson II.2 derived instead of
asserted. The lesson plots $\lambda(t)$ next to the state and shows
$u = -R^{-1}B^\top\lambda$ holding at every sample.

**Bang-bang control.** When the input is bounded and the cost is time, $H$
is linear in $u$, so the minimiser sits on a bound: full thrust one way,
then full thrust the other. For the double integrator the switch happens
on the curve $x = -\tfrac{1}{2a}\,v|v|$. The L1 drone does the fastest
possible altitude change, and the phase portrait shows the state sliding
onto the switching curve.

**Dynamic programming.** The value function $V(x)$, the cost of the best
possible future from $x$, satisfies Bellman's equation, and in continuous
time the Hamilton–Jacobi–Bellman equation [Bertsekas 2017]. `grid.ts`
solves it numerically for the L1 state $(e, v)$ with thrust limits. Near
the origin the result matches the quadratic $x^\top P x$ of LQR; near the
limits it does not, and the policy bends. The value function is also a
Lyapunov function, which ties this chapter to lesson III.7.

**Landing on the last drop.** A rocket descending vertically with mass
$m(t)$, thrust bounded between a minimum and a maximum, and
$\dot m = -T/(I_{sp} g_0)$. The fuel-optimal policy is bang-bang: fall,
then burn at full thrust, the "suicide burn". The ignition altitude
follows from the maximum principle, and the lesson lets the student
predict it.

**Powered-descent guidance.** In three dimensions, with a glide-slope
constraint and a lower bound on thrust, the problem is not convex. The
lower thrust bound can be relaxed with a slack variable so that the
relaxed problem is a second-order cone program with the same solution:
_lossless convexification_ [Açıkmeşe 2007, Blackmore 2010]. Solved again
at every guidance step, it lands the rocket from dispersed initial states
and reports when no feasible landing exists.

**Direct trajectory optimisation.** For problems that are not convex
(a drone flip through a window, a rocket with aerodynamic loads), the
trajectory is discretised and the dynamics become equality constraints at
collocation points; a sparse nonlinear program finds states and inputs
together [Betts 2010]. We solve it by sequential quadratic programming on
the existing QP solver.

### Chapter L — A rocket is not a drone

**The inverted pendulum with an engine.** In the pitch plane a launch
vehicle with its centre of pressure ahead of its centre of mass is
aerodynamically unstable: $\ddot\theta = \mu_\alpha\,\alpha +
\mu_c\,\delta$ with $\mu_\alpha > 0$. An unstable pole at $p$ demands a
bandwidth of at least about $2p$, and the available bandwidth is capped
from above by the actuator and the bending modes. When the two limits
meet there is no controller [Stein 2003]. The loop is **conditionally
stable**: too little gain is as fatal as too much, so it has a lower gain
margin as well as an upper one, and the Nyquist chart shows both.

**Bending.** The vehicle bends, and the rate gyro measures rigid rotation
plus the local slope of the bending mode. If the first bending frequency is
well above crossover it is **gain-stabilised** with a notch or a roll-off;
if it is close, it must be **phase-stabilised**, and the sensor location
decides the sign [Greensite 1970]. The lesson moves the gyro along the
rocket and watches the mode's lobe on the Nyquist chart rotate.

**Slosh.** Liquid propellant behaves as a pendulum attached to the tank.
Its mode is lightly damped and sits near the control bandwidth. The lesson
adds baffles (damping) and shows the margin return.

**Non-minimum phase.** An engine gimballed to pitch the rocket up first
pushes the tail sideways, so the lateral position starts the wrong way. A
right-half-plane zero at $z$ limits the bandwidth to about $z/2$ from
above, the mirror image of the unstable pole's limit.

### Chapter M — Wings

**Trim and the longitudinal modes.** The aircraft model is three degrees
of freedom with lift, drag and pitching moment as functions of angle of
attack, elevator and dynamic pressure $\bar q = \tfrac12\rho V^2$
[Stevens 2016]. `trim` finds the steady flight condition and `linearize`
gives a fourth-order system with two pairs of poles: the fast, well-damped
**short period** and the slow, lightly damped **phugoid**. The lesson
disturbs the aircraft and watches both.

**Stability augmentation and handling qualities.** A pitch-rate feedback
raises the short-period damping. What "good" means is written in a
standard: the short-period frequency and damping must fall inside the
level-1 region [MIL-STD-1797]. The lesson tunes the damper until the
point moves into the region.

**Gain scheduling.** The elevator's effectiveness grows with $\bar q$, so
a gain that is right at low speed is four times too large at twice the
speed. Designs are made at a grid of trim points and interpolated on a
scheduling variable. The lesson flies an acceleration through the
envelope with a fixed gain, then with a scheduled one, and shows the
classical trap: scheduling on a fast variable adds hidden feedback terms
that the frozen-point designs did not see [Rugh 2000].

**Dynamic inversion at scale.** The same inversion as lesson II.7, now
with aerodynamic effectiveness that depends on the flight condition. It
replaces the gain table with a model, and its robustness is the model's
accuracy, measured with the tools of Part III.

**Rate limits and pilot-induced oscillation.** A pilot model (a gain and a
delay) closes the outer loop. When the elevator reaches its rate limit it
adds phase lag that grows with amplitude, and the pilot–aircraft loop
locks into a growing oscillation. The describing function of lesson III.8
predicts the onset [Klyde 1997].

**Energy control.** Throttle changes total energy, elevator trades kinetic
for potential. Controlling those two quantities instead of speed and
altitude separately removes the coupling that makes a naive autopilot
fight itself [Lambregts 1983].

### Chapter N — Pointing in space

**Quaternion feedback.** $\tau = -K_p\,q_{e,vec} - K_d\,\omega$ is
globally stabilising for a rigid body, with a Lyapunov proof that fits on
half a page [Wie 2008]. The sign ambiguity of the quaternion causes
unwinding, met in lesson II.10, and is handled the same way.

**Reaction wheels.** A satellite turns by spinning a wheel the other way;
total angular momentum is conserved. A constant disturbance torque
(gravity gradient, solar pressure) makes the wheels speed up until they
saturate. Momentum must be dumped with thrusters or magnetic torquers,
and the lesson does it without disturbing the pointing.

**On–off thrusters.** A thruster is on or off. With a dead band the
attitude settles into a limit cycle whose amplitude and fuel rate follow
from the phase-plane geometry. A pulse-width pulse-frequency modulator
makes the on–off actuator look proportional on average.

**Flexible appendages.** A solar panel adds a lightly damped mode. With
the sensor and the actuator on the hub (collocated) every such mode is
phase-stabilised and passive control cannot destabilise it; with a sensor
on the tip it can. **Input shaping** removes the residual vibration of a
slew by splitting the command into impulses timed to cancel the mode
[Singer 1990].

**Star tracker and gyro.** The multiplicative EKF of lesson III.23 with a
star tracker as the measurement: arc-second knowledge from a sensor that
updates at a few hertz and a gyro that drifts.

### Chapter O — Guidance and navigation

**Proportional navigation.** Command an acceleration proportional to the
rotation rate of the line of sight, $a = N V_c \dot\lambda$. With
$N$ between 3 and 5 it intercepts a constant-velocity target and is
optimal for a linearised engagement [Zarchan 2012]. The lesson flies the
drone as the pursuer and measures miss distance against $N$, target
manoeuvre and actuator lag.

**Relative motion.** Near a target in circular orbit, relative motion
obeys the linear Clohessy–Wiltshire equations. A rendezvous becomes a
linear control problem, and an LQR or MPC from Part II flies it.

**Inertial navigation.** Integrating an accelerometer and a gyro gives
position that drifts: a gyro bias produces a position error growing with
the cube of time over short intervals. The lesson shows the error growth
with no aiding and then with position fixes.

**When linearisation fails.** With a range-only measurement the EKF's
linearisation is poor far from the truth, and the filter can converge to
the wrong answer with a small covariance. The unscented Kalman filter
propagates sigma points through the true nonlinearity [Julier 2004]; the
particle filter represents a multi-modal belief directly. The
Rauch–Tung–Striebel smoother runs backwards over recorded data and gives
the best estimate after the fact, which is what flight-test
reconstruction uses [Simon 2006].

### Chapter P — From whiteboard to flight (Part V)

**Requirements and budgets.** "Hold position within 0.5 m in 10 m/s gusts,
3σ" is a requirement. It is split into a budget: so much for sensor
error, so much for wind response, so much for quantisation. Each entry is
a number the earlier parts taught how to compute.

**Verification campaigns.** The Monte Carlo runner of lesson III.29, used
as it is used in practice: a requirement table, a dispersion table, a
campaign per requirement, a pass rate with confidence, and a search for
the worst case [Hanson 2010].

**The controller is software.** The same controller written as a
fixed-step function with no allocation, in the integer and
single-precision arithmetic of a flight computer, tested back-to-back
against the floating-point reference. The lesson finds the input where
they disagree.

**Redundancy.** Three sensors and a voter: mid-value selection survives
one failure without detecting it; detection and isolation (lesson III.28)
are needed for the second. Cross-channel monitoring and the cost of a
false alarm.

**Modes and transfers.** Real controllers are a set of modes with logic
between them. A switch between controllers without matching their
internal states causes a transient; **bumpless transfer** initialises the
incoming controller's integrator so the output is continuous. Envelope
protection limits what any mode may command and ties back to the safety
filter of lesson II.17.

**Standards and tools.** A reading chapter in the book: what DO-178C and
ARP4754A ask for in civil aviation, what NASA-STD-7009 asks of a
simulation before its results are trusted, and the gain- and phase-margin
requirements that certification authorities apply. An appendix maps every
function of `src/math` and `src/analysis` to its MATLAB and
Python-control equivalent, because those are the tools the reader will
meet at work.

## 5. Milestones

| Milestone | Content                                                                                                   |
| --------- | --------------------------------------------------------------------------------------------------------- |
| M19       | Vehicle abstraction; quadrotor moved behind it with an unchanged baseline; `trim` and `linearize`         |
| M20       | Chapter K on L1 and the 1D rocket: costate charts, switching curve, value-function grid — IV.1–IV.4       |
| M21       | `socp.ts`, `nlp.ts`, 3D rocket, powered-descent guidance, collocation — IV.5–IV.6                         |
| M22       | Chapter L: planar rocket with thrust-vector control, bending, slosh — IV.7–IV.10                          |
| M23       | Chapter M: aircraft model, atmosphere, modes, augmentation, scheduling, PIO, energy control — IV.11–IV.17 |
| M24       | Chapter N: satellite, wheels, thrusters, flexible panel, star-tracker filter — IV.18–IV.23                |
| M25       | Chapter O: guidance laws, relative motion, INS, UKF, particle filter, smoother — IV.24–IV.29              |
| M26       | Part V: requirement tables, campaign reports, fixed-point controller, voter, mode logic — V.1–V.6         |

### Built (2026-10-02, by three parallel agents)

**M19 — the vehicle abstraction.**

- `sim/vehicles/types.ts`: `Vehicle<S, U>` with `initial`, `step`, state and
  input names, vector conversion and `trim`. `linearize` is not a method: it
  is generic (`analysis/linearize.ts`, central differences of the vehicle's
  own step around its trim), and sensors stay in `sim/sensors.ts`, so the
  interface has no `measure`.
- `sim/dynamics.ts` moved unchanged to `sim/vehicles/quadrotor.ts` (the old
  file re-exports it). The simulator still calls the quadrotor's step
  directly, which keeps `tests/fixtures/baseline.json` bit for bit
  identical (same checksum before and after). Its trim and linearisation
  cover the L1 vertical state (y, v, T) and match the analytic plant of
  Part III to 1e-8; L3 trim is not built.
- `sim/atmosphere.ts`: ISA to 20 km (ρ₀ 1.225 kg/m³, a₀ 340.29 m/s,
  ρ(11 km) 0.3639 kg/m³); not used yet.
- `sim/vehicles/rocket.ts`: the 1D rocket (variable mass, thrust off or in
  [Tmin, Tmax], ṁ = −T/(Isp·g₀), touchdown against 1 m/s), selected by
  `sim.vehicle`; the simulator integrates it and mirrors its state into the
  drone's, so the scene, camera and charts follow. This suits 1D and planar
  vehicles; a full 3D aircraft or satellite will want its own scene pose.
  A procedural lander and landing pad in the scene.

**M20 — Chapter K (IV.1–IV.4).**

- IV.1: the LQR design returns P; the costate λ = P·x is recorded. In
  discrete time the law uses the next sample's costate, so −R⁻¹Bᵀλ of the
  current sample and the command differ by up to 3.5 %, which the lesson
  says; the costate equation holds within 12 % along the flight (the
  two-state model leaves out the motor lag; 3 % with fast motors). The
  student predicts λ_v at the step from P_ev = m·√(q·r) (5.657, flown
  5.645).
- IV.2 (`control/bangbang.ts`): the plan's "beat the PID by 30 %" does not
  hold: a high-gain PID (Kp 60, Kd 10) also rides the thrust limits and
  settles a 1.5 m descent in 0.67 s against the minimum t* = 0.715 s. The
  lesson's goal is to predict t* and settle within 10 % of it; bang-bang
  gives the floor and the shape, not a better controller. The motor lag
  needs a 40 ms lead on the switching curve.
- IV.3 (`math/grid.ts`, `control/dp.ts`): value iteration on (e, v) with the
  real thrust limits (zero thrust among the levels). "Where LQR stops being
  optimal" became "where it stops being exact": V equals xᵀPx up to the
  saturation threshold m·g/k₁ (0.99 m) and rises beyond it (+16 % for a 4 m
  descent, +7 % for a climb), while the saturated LQR stays within 1 % of
  the grid's optimum. The student predicts the threshold.
- IV.4 (`control/rocket.ts`): ignition at 44.65 m (exact, by bisection on
  the simulator's step); the constant-mass estimate h₀·m·g/T gives 45.13 m
  (1.1 % high), which is the prediction. The law is "full thrust, then hold
  0.5 m/s", not a pure bang-bang cut at v = 0, so a small fuel margin gives
  a real window: lit 0.5 m too low it hits at 5.2 m/s, lit 10 m too high it
  runs dry. No aerodynamics yet.

**M25, first lessons (IV.24, IV.26, IV.27).**

- IV.24 (`guidance/pronav.ts`): pure pursuit and proportional navigation
  command the geometric controller's acceleration against a centred
  weaving target (2 m/s; the drone 4 m/s). Pursuit misses by 0.52 m at a
  1.5 m/s² weave and 0.88 m at 3 m/s²; PN hits (0.05 m) every time, on a
  path a third shorter. The target is drawn in a top-view engagement chart,
  not in the 3D scene; no prediction.
- IV.26 (`estimation/ins.ts`): an unaided strapdown navigator runs beside the
  flight from 5 s (the controller does not fly on it). With 0.05 °/s of roll
  bias the error is 38.5 m after 30 s against g·b·t³/6 = 38.52 m, the
  student's prediction. Aiding and the Schuler period are in the text only.
- IV.27 (`estimation/ukf.ts`): one range beacon. The plan's goal ("consistent
  NIS") does not work: both filters' NIS look healthy (the EKF's about 1),
  while the EKF ends 0.6–23 m off with a 2σ of 0.6 m (26–108 σ) and the UKF
  10 m off with a 2σ of 37 m (within 1 σ). The goal is consistency against
  the truth (error ≤ 2σ from 15 to 35 s); the lesson's point is that a good
  NIS does not prove a good estimate, and that the UKF is honest, not more
  accurate. Two beacons were dropped: both filters lock onto the mirror
  solution.

### Built (2026-10-02, second round: chapters L, M and N)

**M22 — Chapter L (IV.7–IV.10).** `sim/vehicles/tvc.ts` (`sim.vehicle =
'tvc'`): a rigid pitch-plane rocket with μα > 0, a gimbal with travel, rate
limit and lag, the first bending mode (gyro station a parameter) and a slosh
mass, frozen at one flight condition: only the motion across the flight path
is simulated, not the burn. `control/tvc.ts` is one discrete controller used
by both the simulator and `analysis/tvc.ts`, which breaks the loop at the
gimbal and gives both gain margins from closed-loop eigenvalues; a Nyquist
chart (`tvc`) shows them.

- IV.7: the student predicts the least pitch gain μα/μc = 0.48 (true 0.46;
  the drift coupling makes it depend a little on kd, which the lesson says).
  Goal ≥ 6 dB both ways; kp 1.5, kd 0.8 gives −9.7/+17.1 dB, PM 49°.
- IV.8: "10 dB of attenuation" became |L| ≤ −10 dB at the mode with PM ≥ 30°
  (a 40 dB notch: −14 dB, 39°; 30 dB is not enough). Gyros at 0.4 and 0.6 see
  the same +18 dB with opposite slopes: one sings, one is stable (in Try).
- IV.9: the goal is a number: the least baffle damping (about ζ 0.027) that
  keeps the curve 0.5 from −1.
- IV.10 flies a hovering test vehicle (T = mg), not a rocket in ascent; the
  student predicts z = √(T·l/I) = 2.02 rad/s. The pitch loop also limits the
  speed: instability comes at an outer ωn of about 0.75 rad/s, not cleanly at
  z/2, and a faster pitch loop makes it worse.

**M23 — Chapter M (IV.11–IV.13, IV.17).** `sim/vehicles/aircraft.ts`: a 5 t
light jet in the pitch plane at 110 m/s and 4000 m (ISA), elevator servo,
engine spool-up and 40 ms of delay; trim by Newton, the linear model by the
generic `linearize`. A light aircraft's short period is already well damped,
so the jet gives the damper something to fix. `control/autopilot.ts`
(attitude hold, damper, schedule, auto-throttle, TECS), `analysis/aircraft.ts`.

- IV.11: short period 1.94 s, ζ 0.24; phugoid 49.3 s, ζ 0.05 (flown within
  1–3 %). The student predicts the phugoid (Lanchester: 49.8 s).
- IV.12: the bare airframe is level 3; the least damper gain for level 1 is
  0.083 s (the short-period approximation says 0.070; most of the gap is the
  approximation, not the servo). Goal: level 1 within 1.1× that gain.
- IV.13: a gain tuned at 80 m/s has 55° there and 30° at 160 m/s, where the
  nose buzzes at 2 Hz; scheduled on q̄ it keeps 55–78°. The goal adds a
  crossover of at least 4 rad/s, because lowering a fixed gain alone would
  meet the margin; without the delay the fixed gain never drops below 68°.
- IV.17: "separate loops fight" does not hold as such (a noise-free
  auto-throttle at gain 0.4 holds 1 m/s). Goal: 0.3 m/s in a 300 m climb with
  the separate loops' throttle gain capped at 0.2: they reach 0.48 m/s, TECS
  0.15 m/s. The engine lag is what makes the difference real.
- Not built: IV.14, IV.15, IV.16; no wind on the aircraft.

**M24 — Chapter N (IV.18–IV.20, IV.22).** `sim/vehicles/satellite.ts`:
quaternion attitude, J = diag(4, 5, 3) kg·m², three wheels (0.2 N·m,
0.6 N·m·s), on–off thruster pairs, a constant disturbance.
`control/satellite.ts` (quaternion feedback, momentum dumping with optional
feedforward, the dead-band law and its limit-cycle formulas),
`estimation/startracker.ts` (the MEKF of III.23 on a biased gyro and a
two-star tracker).

- IV.18: the plan had no prediction; the command is 240° about the
  diagonal and the student predicts the angle actually turned (120°: q and −q
  are one attitude). 15.3 s the short way against about 23 s the long way.
  V falls monotonically until the gains hit the wheel torque limit, then rises
  by about a millionth; the lesson says the guarantee is lost there.
- IV.19: the z wheel fills at h_max/τ_d = 30 s (the prediction); dumping
  alone jumps 0.80°, with feedforward 0.29° (the PD offset τ_d/(kp/2)).
- IV.20: the simple ω²/db fuel formula is off 2.4×; the prediction counts the
  pulse time, and the drift rate is floored at half a minimum pulse. Goal:
  within 0.5° on at most 1.3× the least fuel.
- IV.22: the gyro alone drifts |b|·30 s = 212″ (prediction); with the bias
  estimated it stays under 8″ through a 10 s tracker outage. Without the
  bias state it is about 50″ off while its 2σ says 6″, the point of IV.27
  again.
- Not built: IV.21 (panel), IV.23 (CMG).

`Simulation.loops()` now gives the loops of whatever flies, so the charts and
the live formula follow the rocket, the TVC loop and the autopilot. Still
open: with a Part IV vehicle selected the side panel also shows the drone's
controller, setpoint and PID groups, and a crash shows the drone's "hit the
ground" message.

### Built (2026-10-02, third round: M21, chapters M and N completed, Part V)

**M21 — IV.5 and IV.6.** `math/socp.ts` is a barrier interior-point method
with a phase I that reports infeasibility (not primal-dual); `math/nlp.ts` is
an augmented Lagrangian with BFGS (not SQP: a dense SQP over all collocation
states was too slow in JavaScript). `sim/vehicles/lander.ts` is a 3D
point-mass lander with seeded dispersions, an unknown wind force and an
engine that may deliver less than commanded.

- IV.5 (`guidance/pdg.ts`, lossless convexification, final time by a scan
  and golden section): with a 5 % weak engine and an unknown push, open loop
  crashes 19 of 20 dispersed starts; re-planning every 0.5 s lands 19 at
  about 0.46 m/s within 0.5 m and reports 1 infeasible (propellant). Fuel-
  optimal re-planning alone could not absorb the weak engine: it needed a gate
  3 m above the pad with a terminal descent law, a propellant reserve and the
  engine efficiency measured in flight. The campaign is 20 cases, not 50. No
  prediction.
- IV.6 (`guidance/collocation.ts`): a dash of 6 m past a pillar, not a flip
  through a window. The states are eliminated exactly, so the program's only
  variables are 40 jerks and T. The student predicts the bang-bang bound
  2√(D/a) = 1.97 s; the transcription flies 2.48 s without saturating, while
  min-snap at 2.5 s asks 7.47 m/s² against 6.87 allowed. Min-snap flies
  cleanly from about 2.6 s, so the gain is about 10 %, from using the limits
  fully; the lesson says so instead of "min-snap cannot respect limits".

**M23 and M24, completed (IV.15, IV.16, IV.21).**

- IV.15: NDI of the pitch dynamics; a model error e divides the commanded
  acceleration by 1 + e, a pure gain error. The student predicts the
  bandwidth the aircraft really gets, k_q/(1 + e) = 11.4 rad/s. k_q 6, k_a 1
  meet PM ≥ 45° and crossover ≥ 4 rad/s from 80 to 160 m/s with ±30 % error;
  the q̄ schedule with the same error misses the crossover at 80 m/s. The
  goal became "survive ±30 % across the envelope"; the lesson says the
  schedule could be retuned too.
- IV.16 (`analysis/pio.ts`): pilot gain 1, 150 ms (250 ms gave no PIO),
  elevator 20°/s. A step does not trigger it; a startle burst does. The
  describing function of the simulator's servo puts the onset at 7.65° of
  command (flown: 5.6° dies out, 8.5° locks in). The student predicts the
  slowest PIO-free rate, 52.3°/s (bisection 52.4°/s).
- IV.21: a torsional panel on z (off by default). Shaping a step does
  nothing, because the wheels saturate, so the slew is a profiled bang-bang
  with feedforward and the shaper shapes the profile. The student predicts
  the panel's free-hub frequency 0.524 Hz (the clamped 0.5 Hz fails the 5 %
  goal with ZV; ZVD is robust). A tip sensor makes the swing grow at every
  gain; a hub sensor never.

**M26 — Part V (V.1–V.6).** `analysis/budget.ts`, `campaign.ts`,
`software.ts`, `review.ts`; `control/part5/` (fixed point, shadow, voter,
transfer); panels in `ui/part5/`.

- V.1, V.4 and V.5 fly L1, not L3: one axis makes "biases add, random errors
  in squares" exact and keeps campaigns fast. V.1 and V.2 use turbulence
  only: gusts are heavy-tailed, and no 3σ requirement survives them.
- V.1: the student predicts the wind's allocation (4.24 cm); the budget
  predicts the all-sources flight within 5 %.
- V.2: 20 clean flights prove 86.1 % (the prediction); the V.1 design fails
  overshoot on slow-motor drones. The settling band is ±10 cm (±5 cm is
  unmeetable with a ±3 cm altimeter bias).
- V.3: signals are 32-bit; only the integrator's word and scaling are the
  student's. The prediction is the integrator dead zone, 1.22 m.
- V.4: B is isolated 3.2 s after its drift starts (flown within 0.15 s);
  mid-value selection ends 50 cm off after the second failure, the monitor
  holds 9 cm.
- V.5: PID → LQI on L1; the kick Δm·g = 2.45 N (flown 2.43 N); bumpless
  moves the drone under 1 cm against a 21 cm sag.
- V.6: no PID design found passes the campaign with 45° of phase margin; the
  way out is architectural (the altitude Kalman filter), and the review is
  built around that. Margins are measured by probe; the model check compares
  the measured plant with the controller's own m̂ and τ̂.

### Built (2026-10-03, fourth round: the last five lessons and the UI)

- IV.14 `hidden`: scheduling on α reads the trim column at the measured α,
  which adds δe = −(C_mα/C_mδe)·α and here cancels the static stability
  exactly. The hidden path changes the loop rather than destabilising it:
  the flown phase margin is 85° against the table's 55°, the crossover drops
  to 3.8 rad/s (IV.13's requirement fails) and the short period slows from
  1.7 to 3.9 s. The student predicts the trim slope (−1 °/°). The aircraft
  has no probe, so "measured in flight" is the full-law linearisation,
  checked by flying steps. Scheduling on q̄ removes the gap; a 1 s filter on
  α leaves 2.9°.
- IV.23 `cmg`: a pyramid of four single-gimbal CMGs (h₀ 1 N·m·s, β 54.74°)
  as a satellite actuator. From δ = 0 a 90° slew needs √(Θτ J) = 1.77 N·m·s
  (the prediction) but stalls at 2h₀cos β = 1.155; pseudoinverse, SR and GSR
  all stall, because the command points along the singular direction, and
  null-motion gradient steering does nothing on that symmetric path. What
  works is null-motion pre-positioning: parked gimbals settle in 6.9 s.
- IV.25 `rendezvous` (`sim/vehicles/chaser.ts`, exact discrete CW model;
  `control/rendezvous.ts`, a condensed QP on `qp.ts`): a 5 cm/s forward burn
  leaves the chaser 6πΔv/n = 833 m behind after one orbit (the prediction).
  Goal: dock below 5 cm/s within an orbit on 0.3 m/s; in-plane only, time
  warped 120×, the target drawn at 1:10.
- IV.28 `particle`: the filter is told the drone's velocity; with one beacon
  and an unknown velocity any rotation of the path about the beacon fits, and
  no motion could break the symmetry. Calm air (turbulence alone breaks the
  ring). A straight leg leaves the mirror image (the prediction: its x,
  20 m); only a turn resolves it.
- IV.29 `smoother`: mid-record the smoother's σ is half the filter's for a
  slow model (the prediction); "halve the RMS" holds for any reasonable q, so
  the student tunes q down from a deliberately bad 100.
- UI: a group or field now names the vehicles it belongs to (`vehicles`);
  untagged groups and `drone.*` fields are the drone's and are hidden while a
  Part IV vehicle flies. The end-of-flight message and the chart beside the
  main one follow the vehicle.

**Build order.** M19 first: nothing else in Part IV can start without the
vehicle abstraction. Then the **must** lessons across M20–M25, then Part V,
then the rest. Each vehicle is usable after its own milestone.

## 6. Lessons

Priority: **must**, **should**, **could**.

### Chapter K — The best possible path

| #   | id            | Title                          | Vehicle    | What the student sees / goal                                                                                                                              | Prio   |
| --- | ------------- | ------------------------------ | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 1   | `costate`     | Where LQR comes from           | L1 drone   | The costate plotted beside the state; $u = -R^{-1}B^\top\lambda$ holds at every sample, and $\lambda = Px$. Goal: predict $\lambda(0)$ for a given step.  | must   |
| 2   | `bangbang`    | Full throttle, then full brake | L1 drone   | The fastest altitude change rides the thrust limits and switches once, on the curve drawn in the phase portrait. Goal: beat the PID's time by 30 %.       | must   |
| 3   | `bellman`     | The map of all futures         | L1 drone   | The value function as a heat map; quadratic near the origin, bent near the thrust limits; the policy as arrows. Goal: find where LQR stops being optimal. | should |
| 4   | `suicideburn` | Land on the last drop          | Rocket, 1D | Bang-bang again, now with mass that burns away. Goal: predict the ignition altitude within 5 % and land below 1 m/s.                                      | must   |
| 5   | `pdg`         | Convex landing                 | Rocket, 3D | A fuel-optimal landing inside a glide-slope cone, re-solved every step; dispersed starts; an infeasible start is reported. Goal: land 50 dispersed cases. | should |
| 6   | `collocation` | Plan the whole flight          | L3 drone   | A flip through a window planned by direct collocation with thrust limits, which min-snap (II.12) could not respect. Goal: a plan that never saturates.    | should |

### Chapter L — A rocket is not a drone

| #   | id         | Title                  | Vehicle        | What the student sees / goal                                                                                                                             | Prio   |
| --- | ---------- | ---------------------- | -------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 7   | `tvc`      | Balancing on a flame   | Rocket, planar | An unstable airframe held by gimballing the engine; the Nyquist chart shows a lower and an upper gain margin. Goal: at least 6 dB both ways.             | must   |
| 8   | `bending`  | The rocket is a noodle | Rocket, planar | The gyro picks up the first bending mode and the loop sings; a notch, then a better gyro location. Goal: 10 dB of attenuation at the mode with PM ≥ 30°. | must   |
| 9   | `slosh`    | Fuel that moves        | Rocket, planar | A slosh pendulum near crossover eats the margin; baffles restore it. Goal: keep the slosh mode's lobe clear of the critical point.                       | could  |
| 10  | `wrongway` | First the wrong way    | Rocket, planar | Lateral position starts opposite to the command; pushing the bandwidth past half the zero destabilises. Goal: the fastest stable lateral step.           | should |

### Chapter M — Wings

| #   | id            | Title                     | Vehicle  | What the student sees / goal                                                                                                                                | Prio   |
| --- | ------------- | ------------------------- | -------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 11  | `modes`       | Two ways to wobble        | Aircraft | An elevator pulse excites the short period (seconds) and the phugoid (a minute); both appear as pole pairs. Goal: predict both periods from the pole map.   | must   |
| 12  | `pitchdamper` | Level 1                   | Aircraft | A pitch damper moves the short-period point into the level-1 region of the handling-qualities chart. Goal: level 1 with the least gain.                     | must   |
| 13  | `schedule`    | One gain does not fit all | Aircraft | A fixed gain is sluggish when slow and oscillates when fast; a schedule on dynamic pressure is right everywhere. Goal: PM ≥ 45° across the envelope.        | must   |
| 14  | `hidden`      | The trap in the table     | Aircraft | Scheduling on angle of attack adds a feedback path the point designs never saw; margins measured in flight differ from the table. Goal: find and remove it. | could  |
| 15  | `ndi`         | Invert the aeroplane      | Aircraft | Dynamic inversion replaces the table; a 30 % error in the aerodynamic model costs margin, measured with the probe. Goal: compare with the schedule.         | should |
| 16  | `pio`         | The pilot in the loop     | Aircraft | A pilot model and a rate-limited elevator lock into a growing oscillation at the amplitude the describing function predicts. Goal: predict the onset.       | should |
| 17  | `energy`      | Throttle is energy        | Aircraft | Separate speed and altitude loops fight; total-energy control does not. Goal: a climb with speed held within 1 m/s.                                         | should |

### Chapter N — Pointing in space

| #   | id            | Title                  | Vehicle   | What the student sees / goal                                                                                                                                   | Prio   |
| --- | ------------- | ---------------------- | --------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 18  | `slew`        | Point the telescope    | Satellite | A 120° slew with quaternion feedback follows the shortest rotation; the Lyapunov function falls all the way. Goal: settle within 0.1° in the least time.       | must   |
| 19  | `wheels`      | Spin to turn           | Satellite | Wheel speeds mirror the body's motion; a constant disturbance winds them up to saturation and pointing is lost. Goal: dump momentum without losing the target. | must   |
| 20  | `thrusters`   | On or off              | Satellite | A dead-band limit cycle in the phase plane; its width sets the fuel rate. Goal: pointing within 0.5° at the lowest fuel rate.                                  | should |
| 21  | `panel`       | Panels that wave       | Satellite | A fast slew leaves the panel ringing; input shaping cancels it. A tip sensor destabilises what a hub sensor cannot. Goal: residual vibration below 5 %.        | should |
| 22  | `startracker` | Stars and gyros        | Satellite | The MEKF fuses a slow, accurate star tracker with a fast, drifting gyro; bias and covariance converge. Goal: knowledge within 10 arc-seconds.                  | should |
| 23  | `cmg`         | The singular direction | Satellite | Control-moment gyros give large torque until their axes align and one direction is lost. Goal: a steering law that avoids the singularity.                     | could  |

### Chapter O — Guidance and navigation

| #   | id           | Title                   | Vehicle   | What the student sees / goal                                                                                                                              | Prio   |
| --- | ------------ | ----------------------- | --------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 24  | `pronav`     | Aim where it will be    | L3 drone  | Pure pursuit tail-chases; proportional navigation holds the line of sight still and intercepts. Goal: miss distance below 0.2 m against a weaving target. | must   |
| 25  | `rendezvous` | Falling around together | Satellite | Thrusting forward makes the chaser drop behind; the Clohessy–Wiltshire model explains it and an MPC flies the approach. Goal: dock below 5 cm/s.          | could  |
| 26  | `ins`        | Drift                   | L3 drone  | With no position fix the inertial solution leaves the truth, slowly then fast. Goal: predict the error after 30 s from the gyro bias.                     | should |
| 27  | `ukf`        | When the tangent lies   | L3 drone  | Range-only beacons: the EKF converges to the wrong place with a confident covariance; the UKF does not. Goal: consistent NIS.                             | should |
| 28  | `particle`   | Many guesses at once    | L3 drone  | One beacon gives a ring of possible positions; the particle cloud holds the ring until motion breaks the symmetry. Goal: see the cloud collapse.          | could  |
| 29  | `smoother`   | Hindsight               | L3 drone  | The smoothed estimate of a recorded flight is better than the filter's at every instant. Goal: halve the RMS error of the filter.                         | could  |

### Chapter P — From whiteboard to flight (Part V)

| #   | id           | Title                        | Vehicle  | What the student sees / goal                                                                                                                             | Prio   |
| --- | ------------ | ---------------------------- | -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ |
| 1   | `budget`     | What does "good" mean?       | L3 drone | A requirement split into an error budget whose entries are measured one by one. Goal: a budget that closes with 20 % left over.                          | must   |
| 2   | `campaign`   | The verification campaign    | Any      | A requirement table, a dispersion table and one campaign per requirement; the report lists pass rates and the worst case. Goal: every requirement green. | must   |
| 3   | `fixedpoint` | The controller is code       | L1 drone | The PID in 16-bit arithmetic against the reference; they disagree when the integrator is scaled badly. Goal: back-to-back error below 1 %.               | should |
| 4   | `voter`      | Three sensors, one truth     | L3 drone | Mid-value selection hides the first failure; the second needs detection. Goal: survive two altimeter failures.                                           | should |
| 5   | `bumpless`   | Change controllers in flight | L3 drone | Switching from PID to LQR with unmatched states kicks the drone; a bumpless transfer does not. Goal: a transient below 5 cm.                             | should |
| 6   | `review`     | The design review            | All      | The capstone: one vehicle, a requirement set, and a checklist (model, margins, campaign, faults, software). The app produces the review report.          | must   |

## 7. Risks and open questions

| Risk                                                                                             | Mitigation                                                                                                                                                                |
| ------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| The vehicle abstraction breaks the quadrotor's regression baseline.                              | M19 moves the code without changing behaviour and is accepted only when `tests/fixtures/baseline.json` passes unchanged.                                                  |
| A second-order cone solver in TypeScript, fast enough to re-solve a landing every guidance step. | Small problems (about 50 time steps); a fixed iteration budget; if it is too slow, IV.5 solves once per second and tracks the plan in between, as flight software does.   |
| Aerodynamic data for the aircraft.                                                               | A published generic model with constant stability derivatives per flight condition [Stevens 2016], not a real aircraft. The lessons need the structure, not the fidelity. |
| Four vehicles multiply the UI: parameter panels, scene models, cameras.                          | The parameter panel is already generated from the schema; each vehicle adds a schema block. Scene models stay procedural and simple.                                      |
| Scope: 35 more lessons.                                                                          | The **must** set is 14 lessons (IV.1, 2, 4, 7, 8, 11–13, 18, 19, 24 and V.1, 2, 6) and covers one lesson or more on every vehicle. The rest is the second pass.           |
| The handling-qualities boundaries come from a military standard and are often misquoted.         | The book reproduces the boundaries from the standard itself and states the flight phase category they apply to.                                                           |
| Part V's standards chapter can only describe certification, not teach it.                        | It is a reading chapter and says so. [curriculum.md](curriculum.md) §7 lists what the programme does not provide.                                                         |

## 8. References

- [Bryson 1975] A. E. Bryson, Y.-C. Ho, _Applied Optimal Control_, revised printing, Hemisphere, 1975.
- [Bertsekas 2017] D. P. Bertsekas, _Dynamic Programming and Optimal Control_, vol. I, 4th ed., Athena Scientific, 2017.
- [Betts 2010] J. T. Betts, _Practical Methods for Optimal Control and Estimation Using Nonlinear Programming_, 2nd ed., SIAM, 2010.
- [Açıkmeşe 2007] B. Açıkmeşe, S. R. Ploen, "Convex programming approach to powered descent guidance for Mars landing", _Journal of Guidance, Control, and Dynamics_ 30(5), 2007.
- [Blackmore 2010] L. Blackmore, B. Açıkmeşe, D. P. Scharf, "Minimum-landing-error powered-descent guidance for Mars landing using convex optimization", _Journal of Guidance, Control, and Dynamics_ 33(4), 2010.
- [Stein 2003] G. Stein, "Respect the unstable", _IEEE Control Systems Magazine_ 23(4), 2003.
- [Greensite 1970] A. L. Greensite, _Analysis and Design of Space Vehicle Flight Control Systems_, Spartan Books, 1970.
- [Stevens 2016] B. L. Stevens, F. L. Lewis, E. N. Johnson, _Aircraft Control and Simulation_, 3rd ed., Wiley, 2016.
- [MIL-STD-1797] U.S. Department of Defense, _Flying Qualities of Piloted Aircraft_, MIL-STD-1797A, 1990.
- [Rugh 2000] W. J. Rugh, J. S. Shamma, "Research on gain scheduling", _Automatica_ 36(10), 2000.
- [Klyde 1997] D. H. Klyde, D. T. McRuer, T. T. Myers, "Pilot-induced oscillation analysis and prediction with actuator rate limiting", _Journal of Guidance, Control, and Dynamics_ 20(1), 1997.
- [Lambregts 1983] A. A. Lambregts, "Vertical flight path and speed control autopilot design using total energy principles", AIAA paper 83-2239, 1983.
- [Arulampalam 2002] M. S. Arulampalam, S. Maskell, N. Gordon, T. Clapp, "A tutorial on particle filters for online nonlinear/non-Gaussian Bayesian tracking", _IEEE Transactions on Signal Processing_ 50(2), 2002.
- [Clohessy 1960] W. H. Clohessy, R. S. Wiltshire, "Terminal guidance system for satellite rendezvous", _Journal of the Aerospace Sciences_ 27(9), 1960.
- [Rauch 1965] H. E. Rauch, F. Tung, C. T. Striebel, "Maximum likelihood estimates of linear dynamic systems", _AIAA Journal_ 3(8), 1965.
- [Wie 2008] B. Wie, _Space Vehicle Dynamics and Control_, 2nd ed., AIAA, 2008.
- [Markley 2014] F. L. Markley, J. L. Crassidis, _Fundamentals of Spacecraft Attitude Determination and Control_, Springer, 2014.
- [Singer 1990] N. C. Singer, W. P. Seering, "Preshaping command inputs to reduce system vibration", _Journal of Dynamic Systems, Measurement, and Control_ 112(1), 1990.
- [Zarchan 2012] P. Zarchan, _Tactical and Strategic Missile Guidance_, 6th ed., AIAA, 2012.
- [Julier 2004] S. J. Julier, J. K. Uhlmann, "Unscented filtering and nonlinear estimation", _Proceedings of the IEEE_ 92(3), 2004.
- [Simon 2006] D. Simon, _Optimal State Estimation: Kalman, H∞, and Nonlinear Approaches_, Wiley, 2006.
- [Hanson 2010] J. M. Hanson, B. B. Beard, _Applying Monte Carlo Simulation to Launch Vehicle Design and Requirements Analysis_, NASA/TP-2010-216447, 2010.
