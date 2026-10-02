# The Curriculum

> **Status (2026-09-30):** Parts I and II are implemented in the app. Part I
> is written in the book. Parts III–V are planned. This document is the
> map of the whole programme; the part plans hold the detail.

Anemostatos is one course in two forms: the simulator, and the book
_Anemostatos_. This page says what the course is meant to teach, what it covers
today, what is still missing, and in which order the missing pieces get
built.

## 1. The goal

A reader who works through all five parts, with the simulator open and the
exercises done, should have the knowledge of a graduate of a good master's
programme in guidance, navigation and control: able to model a vehicle,
analyse a loop, design a controller by several methods, estimate a state,
verify the result statistically, and explain every step.

That is the knowledge an aerospace employer expects of a junior control
engineer. It is not everything such an employer expects. Section 7 lists
what no course of this kind can provide.

Three principles hold throughout:

1. **Watch first, then explain.** Every idea starts from something that
   flies, in the simulator.
2. **Prediction next to measurement.** Every formula is checked against
   the simulator on the same page or the same screen.
3. **No more mathematics than the answer needs, and no less.** The book
   derives each result in full; the app shows it.

## 2. The five parts

| Part                         | Lessons      | Vehicles                               | App                         | Book            | Plan                                 |
| ---------------------------- | ------------ | -------------------------------------- | --------------------------- | --------------- | ------------------------------------ |
| I. The PID controller        | 1–14         | Quadrotor, levels L1–L3                | done (M0–M6)                | written; see §5 | [roadmap.md](roadmap.md)             |
| II. Beyond PID               | II.1–II.21   | Quadrotor                              | done (M7–M12)               | not written     | [beyond-pid.md](beyond-pid.md)       |
| III. Why it works            | III.1–III.30 | Quadrotor                              | 18 of 30 done; rest planned | not written     | [analysis.md](analysis.md)           |
| IV. Aerospace GNC            | IV.1–IV.29   | Quadrotor, rocket, aircraft, satellite | planned (M19–M25)           | not written     | [aerospace-gnc.md](aerospace-gnc.md) |
| V. From whiteboard to flight | V.1–V.6      | All                                    | planned (M26)               | not written     | [aerospace-gnc.md](aerospace-gnc.md) |

One hundred lessons. Fifty-three exist in the app today.

Each part asks a different question:

- **I** — how does feedback work, and how do I make one loop behave?
- **II** — which controllers exist beyond PID, and what does each cost?
- **III** — how do I know a loop will work before it flies, and when the
  model is wrong?
- **IV** — what changes when the vehicle has wings, fuel, flexibility or
  no gravity, and who decides where it should go?
- **V** — what else must be true before a design is allowed to fly?

## 3. Competence map

The body of knowledge of a graduate GNC curriculum, and where this course
teaches each item. "App" says whether the lessons exist in the simulator.

| Area                          | Topics                                                                                           | Lessons                           | App     |
| ----------------------------- | ------------------------------------------------------------------------------------------------ | --------------------------------- | ------- |
| Feedback fundamentals         | Open and closed loop, error, disturbance rejection, sensitivity to model error                   | 1–5                               | done    |
| PID in practice               | Windup, derivative kick, noise and filtering, sampling, delay, actuator lag, tuning              | 6–11                              | done    |
| Cascaded loops                | Time-scale separation, inner and outer loops, mixing                                             | 12–14                             | done    |
| Classical analysis            | Transfer functions, Bode, Nyquist, root locus, Routh–Hurwitz, margins, system type               | III.1–III.4                       | planned |
| Fundamental limits            | Bode integral, delay, right-half-plane zeros and poles                                           | III.4, III.5, IV.7, IV.10         | planned |
| Loop shaping                  | Lead, lag, notch, target loop shapes                                                             | III.6                             | planned |
| State space                   | State feedback, integral action, pole placement                                                  | II.1–II.3                         | done    |
| Optimal control, linear       | LQR, LQI, LQG, loop-transfer recovery                                                            | II.2, II.3, III.10                | partly  |
| Optimal control, theory       | Calculus of variations, Pontryagin, bang-bang, dynamic programming, HJB                          | IV.1–IV.4                         | planned |
| Trajectory optimisation       | Minimum snap, direct collocation, convex powered-descent guidance                                | II.12, IV.5, IV.6                 | partly  |
| Predictive control            | QP, linear MPC, sampling-based MPC, constraints                                                  | II.13–II.16                       | done    |
| Safety filters                | Control barrier functions, envelope protection                                                   | II.17, V.5                        | partly  |
| Estimation, linear            | Kalman filter, observability, detectability, separation                                          | II.4, III.9                       | partly  |
| Estimation, nonlinear         | Complementary and Mahony filters, EKF, multiplicative EKF, UKF, particle filter, smoothing       | III.22–III.24, IV.22, IV.27–IV.29 | partly  |
| Disturbance observers         | ADRC, extended state observer, incremental control (INDI)                                        | II.5–II.9                         | done    |
| Nonlinear stability           | Lyapunov, LaSalle, region of attraction, describing functions, circle criterion                  | III.7, III.8                      | planned |
| Nonlinear design              | Dynamic inversion, geometric control on SO(3), differential flatness, sliding mode, backstepping | II.7–II.12, III.15, III.16, IV.15 | partly  |
| Robust control                | Uncertainty models, small gain, singular values, disk margins, H∞ loop shaping, μ analysis       | III.11–III.14                     | partly  |
| Adaptive control              | MRAC, robust modifications, L1 adaptive control                                                  | II.18, III.17                     | partly  |
| Learning-based control        | Learned residual models, neural-network policies, reinforcement learning                         | II.19–II.21                       | done    |
| Digital implementation        | Sampling, z-plane, aliasing, quantisation, rate limits, jitter, fixed-point code                 | 9, III.18–III.20, V.3             | partly  |
| Filtering of real sensors     | Vibration, notch and RPM-tracking filters, anti-alias filters                                    | 8, III.20, III.21                 | partly  |
| System identification         | Least squares, relay tuning, frequency-domain identification with coherence                      | III.25, III.26                    | planned |
| Fault detection and tolerance | Residual tests, redundancy and voting, control after actuator loss                               | II.7, III.27, III.28, V.4         | partly  |
| Verification                  | Monte Carlo campaigns, pass rates with confidence, worst cases, requirements and budgets         | III.29, III.30, V.1, V.2, V.6     | partly  |
| Flight dynamics               | Trim, linearisation, longitudinal modes, stability augmentation, handling qualities              | IV.11, IV.12                      | planned |
| Gain scheduling               | Point designs, interpolation, hidden coupling                                                    | IV.13, IV.14                      | planned |
| Launch vehicles               | Thrust-vector control, conditional stability, bending, slosh                                     | IV.7–IV.10                        | planned |
| Spacecraft attitude           | Quaternion feedback, reaction wheels, momentum management, thrusters, flexible appendages        | IV.18–IV.23                       | planned |
| Guidance and navigation       | Proportional navigation, relative orbital motion, inertial navigation                            | IV.24–IV.26                       | planned |
| Engineering practice          | Mode logic, bumpless transfer, standards, the design review                                      | V.5, V.6                          | planned |

Left out on purpose, with the reason in the part plans: multi-agent and
formation control, μ-synthesis, stochastic and dual control, hybrid-systems
theory, and orbital mechanics beyond relative motion. The book's reading
list points to a text for each.

## 4. The standard of a lesson

A topic counts as taught only when both halves exist.

**In the app**, a lesson has: a set-up that starts in the interesting
state; scripted events that replay on reset; the chart that shows the idea
already selected; a goal with a check the tests prove reachable
(`solution` in `src/lessons/types.ts`); and text short enough to read
beside a running simulation.

**In the book**, a chapter has:

1. a lead that starts from something the reader has just seen fly;
2. the derivation in full, from what a bachelor's degree provides, with
   every tool that goes beyond it explained where it is used and collected
   in the mathematical toolbox;
3. at least **three worked examples** with the vehicle's real numbers,
   solved step by step;
4. a figure with the prediction and the simulator's measurement together,
   and an explanation wherever they differ;
5. an _In Anemostatos_ box with the parameter paths and source files;
6. a _Try this_ experiment with a question to answer before running it;
7. an _Out in the World_ page;
8. a section **The engineer's view**, which restates the result in the
   language used at work (state, poles, energy, steady-state errors). It
   ends with a subsection **In formal terms**: every term the lesson relied
   on is defined as the literature defines it, and every general claim is
   stated as a numbered theorem with its hypotheses, followed by a proof or
   by the source of one. The text then says which hypothesis fails where;
9. a **Where this leads** box naming the later lessons, in Parts II–V,
   where each idea returns, and one worked example on another vehicle;
10. a summary, and at least **six exercises** in three grades, including
    one derivation (`derive`), one experiment in the simulator (`fly`) and
    one task on the simulator's source (`code`), each labelled as such.

Solutions are published in a companion volume, not in the book.

**Notation is binding.** The Prelude's section _Notation and conventions_
fixes one meaning per symbol for the whole book: bold for state vectors,
arrows for vectors in space, italic capitals for matrices, $I$ for the
integral term and therefore $J$ for inertia and a separate symbol for the
identity matrix, $d$ positive upwards, radians in every formula. It also
fixes the kinds of statement: a _Definition_ fixes a term, a _Theorem_ is
general and proven or cited, a _Result_ is a formula for our vehicle. A
chapter that needs a new symbol adds it to that table.

## 5. The book

### 5.1 Volumes

| Volume | Parts | Title                                                    | State                                   |
| ------ | ----- | -------------------------------------------------------- | --------------------------------------- |
| 1      | I–II  | _Anemostatos_ — from PID to learning                     | Part I written, Part II not yet         |
| 2      | III   | analysis, robustness and the real loop                   | after the app's Part III                |
| 3      | IV–V  | aerospace guidance, navigation and control, and practice | after the app's Parts IV–V              |
| —      | all   | solutions to the exercises                               | collected by `\answer` during the build |

The app leads and the book follows: a chapter is written when its lesson
runs, because every figure in the book is flown by the simulator
(`scripts/book-data.ts`).

### 5.2 Mathematical toolbox

The preface promises that every tool beyond a bachelor's degree is
collected at the end. The toolbox (`book/back/math.tex`, one file per
appendix in `book/back/toolbox/`) is a short course, not a formula sheet:
each appendix runs to about ten pages, states its theorems with a proof or
a source, works two or three examples with the drone's numbers, and lists
the lessons that use it. It has no exercises.

The letters are fixed for the whole course and follow the order of first
use. An appendix is written in full for the parts its volume contains;
material that only later parts need is added now only where it is short
and completes the subject, and otherwise with the volume that needs it.

| Appendix | Content                                                                             | First needed | Volume | Grows with                                |
| -------- | ----------------------------------------------------------------------------------- | ------------ | ------ | ----------------------------------------- |
| A        | Linear differential equations, state space, the matrix exponential                  | lesson I.2   | 1      | —                                         |
| B        | Polynomials and stability: Routh–Hurwitz, root-locus rules                          | lesson I.5   | 1      | —                                         |
| C        | Complex numbers, Fourier series, the Laplace transform, transfer functions, Nyquist | lesson I.8   | 1      | Bode relations (III.5)                    |
| D        | Sampling and the z-transform                                                        | lesson I.9   | 1      | multirate loops (III.18)                  |
| E        | Probability, random processes, spectra, least squares                               | lesson I.8   | 1      | Bayesian estimation (III.24, IV.27–IV.29) |
| F        | Numerical methods                                                                   | lesson I.1   | 1      | collocation (IV.6), fixed point (V.3)     |
| G        | Stability of nonlinear systems: Lyapunov, invariance, time scales                   | lesson I.2   | 1      | describing function (III.8)               |
| H        | Rotations and quaternions                                                           | lesson I.12  | 1      | the error state of the MEKF (III.23)      |
| I        | Linear algebra: singular values, positive definiteness, the Riccati equation        | II.2         | 1      | — (written with Part II)                  |
| J        | Optimisation: convexity, KKT conditions, quadratic and cone programs                | II.13        | 1      | — (written with Part II)                  |
| K        | Norms of signals and systems                                                        | III.11       | 2      | —                                         |
| L        | Calculus of variations and dynamic programming                                      | IV.1         | 3      | —                                         |

### 5.3 State of volume 1, verified 2026-09-30

First checked on the build of commit `8e1df9c` (127 pages): the sources,
the build log and the rendered pages. Lessons I.1–I.4 were then rebuilt to the
full standard of §4, and lessons I.5–I.14 after them; Part I is complete and
the book now has 288 pages with the toolbox.

**What is good.** The design system is consistent and carries the app's
colours. Chapter 0 and the Prelude are complete. Lessons I.1–I.4 meet the
whole standard of §4 and are the model for the rest: five or six worked
examples and eleven to thirteen exercises each (labelled `derive`, `fly`,
`code`), a figure that draws each prediction over the simulator's flight,
an engineer's-view section with twelve theorems between them, stated with
hypotheses and proofs or sources, and a where-this-leads box. Every number they
quote from the simulator comes from an experiment in
`scripts/book-data.ts`. The book builds from a clean checkout in
eleven seconds.

**What is missing or wrong.**

| #   | Finding                                                                                                                                                                                                                                     | State                                                   |
| --- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------- |
| 1   | Lessons I.5–I.14 had no worked example and three to five exercises each, over five to seven pages. They did not meet §4.                                                                                                                    | fixed (rewritten to the level of 1–4)                   |
| 2   | Part II is a title page followed by nothing; its five chapter files are stubs. The cover promises thirty-five lessons.                                                                                                                      | open: 21 chapters to write                              |
| 3   | The mathematical toolbox and the app map were stubs. Appendices A–H are written (about 80 pages, `book/back/toolbox/`); I and J follow with Part II, and the app map is still a stub. The remaining "??" are references to Part II lessons. | open: appendices I, J and the app map                   |
| 4   | The check marks of the _What to remember_ boxes were missing: the math font has no such glyph.                                                                                                                                              | fixed (drawn with TikZ)                                 |
| 5   | `main.tex` and `scripts/book-data.ts` referred to `make book`, `make book-data` and `book/README.md`, none of which existed.                                                                                                                | fixed (three make targets, the README)                  |
| 6   | The imprint named a script `book/tools/figures` that does not exist.                                                                                                                                                                        | fixed                                                   |
| 7   | No author, editor or credit anywhere in the book.                                                                                                                                                                                           | fixed (title page, imprint, colophon)                   |
| 8   | The field map of Chapter 0 listed Bode, Nyquist, root locus, Lyapunov, Pontryagin, dynamic programming and H∞ against Parts I–II, which do not teach them.                                                                                  | fixed (the map names the part that does)                |
| 9   | Chapter 0 and the preface described a two-part course.                                                                                                                                                                                      | fixed (they describe the five parts)                    |
| 10  | Four lines were too wide by 3–11 pt (lessons I.2 and I.4, the motor figure).                                                                                                                                                                | fixed in lessons I.1–I.4; three lines under 3 pt remain |

Findings 1–3 are writing work and belong to the book's own schedule.

## 6. Order of work

1. **M13 and the must lessons of Part III** in the app
   ([analysis.md](analysis.md) §5). Everything later uses its charts and
   its Monte Carlo runner.
2. **Volume 1 of the book to the standard of §4**: lessons I.5–I.14, Part II,
   appendices A–J (A–H first, I and J with Part II).
3. **The rest of Part III**, then volume 2.
4. **M19, the vehicle abstraction**, then the must lessons of Part IV,
   then Part V ([aerospace-gnc.md](aerospace-gnc.md) §5).
5. **The rest of Part IV**, then volume 3.

Steps 1 and 2 are independent and can run side by side.

## 7. What the course does not provide

Stated plainly, so that nobody is misled by the goal in §1:

- **The tools of the trade.** Industry works in MATLAB and Simulink, in C
  and in Python. The course's code is TypeScript. Volume 3 maps every
  function to its equivalent, which helps, and is not the same as having
  used them.
- **Hardware.** Nothing here replaces flying a real vehicle, debugging a
  real sensor or running a hardware-in-the-loop bench.
- **Process.** Certification standards are described in Part V. Working
  under them is learnt on a programme.
- **Depth in one speciality.** A working engineer goes far deeper into one
  area (flight dynamics, attitude determination, trajectory optimisation)
  than any survey can. The reading list names the book to continue with.
- **A degree.** Employers in this field ask for one.

## 8. Reference texts

The standard texts that cover the same ground, for readers who want a
second explanation or more depth.

| Part | Texts                                                                                                                                                                                                                                                                                    |
| ---- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| I    | Åström & Murray, _Feedback Systems_; Åström & Hägglund, _Advanced PID Control_                                                                                                                                                                                                           |
| II   | Anderson & Moore, _Optimal Control: Linear Quadratic Methods_; Rawlings, Mayne & Diehl, _Model Predictive Control_                                                                                                                                                                       |
| III  | Skogestad & Postlethwaite, _Multivariable Feedback Control_; Khalil, _Nonlinear Systems_; Franklin, Powell & Workman, _Digital Control of Dynamic Systems_; Simon, _Optimal State Estimation_                                                                                            |
| IV   | Bryson & Ho, _Applied Optimal Control_; Stevens, Lewis & Johnson, _Aircraft Control and Simulation_; Wie, _Space Vehicle Dynamics and Control_; Markley & Crassidis, _Fundamentals of Spacecraft Attitude Determination and Control_; Zarchan, _Tactical and Strategic Missile Guidance_ |
| V    | Hanson & Beard, _Applying Monte Carlo Simulation to Launch Vehicle Design and Requirements Analysis_ (NASA/TP-2010-216447)                                                                                                                                                               |

## 9. Credits

Idea and editing: Wojciech Stelmaszewski. Text, figures and code: Claude
Opus 5.5 (Anthropic).
