# The Curriculum

> **Status (2026-10-02):** Parts I–III are implemented in the app and Part IV
> is under way (19 of 29 lessons). Parts I and II, volume 1 of the book, are
> written; its back matter is being finished (§5.3). Part V is planned.
> This document is the map of the whole programme; the part plans hold the
> detail.

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

| Part                         | Lessons      | Vehicles                               | App                     | Book            | Plan                                 |
| ---------------------------- | ------------ | -------------------------------------- | ----------------------- | --------------- | ------------------------------------ |
| I. The PID controller        | I.1–I.14     | Quadrotor, levels L1–L3                | done (M0–M6)            | written; see §5 | [roadmap.md](roadmap.md)             |
| II. Beyond PID               | II.1–II.21   | Quadrotor                              | done (M7–M12)           | written; see §5 | [beyond-pid.md](beyond-pid.md)       |
| III. Why it works            | III.1–III.30 | Quadrotor                              | done (M13–M18)          | not written     | [analysis.md](analysis.md)           |
| IV. Aerospace GNC            | IV.1–IV.29   | Quadrotor, rocket, aircraft, satellite | 24 of 29 (M19–M22 done) | not written     | [aerospace-gnc.md](aerospace-gnc.md) |
| V. From whiteboard to flight | V.1–V.6      | All                                    | done (M26)              | not written     | [aerospace-gnc.md](aerospace-gnc.md) |

One hundred lessons. Ninety-five exist in the app today: Parts I–III and V complete, Part IV all but five.

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
| Feedback fundamentals         | Open and closed loop, error, disturbance rejection, sensitivity to model error                   | I.1–I.5                           | done    |
| PID in practice               | Windup, derivative kick, noise and filtering, sampling, delay, actuator lag, tuning              | I.6–I.11                          | done    |
| Cascaded loops                | Time-scale separation, inner and outer loops, mixing                                             | I.12–I.14                         | done    |
| Classical analysis            | Transfer functions, Bode, Nyquist, root locus, Routh–Hurwitz, margins, system type               | III.1–III.4                       | done    |
| Fundamental limits            | Bode integral, delay, right-half-plane zeros and poles                                           | III.4, III.5, IV.7, IV.10         | done    |
| Loop shaping                  | Lead, lag, notch, target loop shapes                                                             | III.6                             | done    |
| State space                   | State feedback, integral action, pole placement                                                  | II.1–II.3                         | done    |
| Optimal control, linear       | LQR, LQI, LQG, loop-transfer recovery                                                            | II.2, II.3, III.10                | done    |
| Optimal control, theory       | Calculus of variations, Pontryagin, bang-bang, dynamic programming, HJB                          | IV.1–IV.4                         | done    |
| Trajectory optimisation       | Minimum snap, direct collocation, convex powered-descent guidance                                | II.12, IV.5, IV.6                 | partly  |
| Predictive control            | QP, linear MPC, sampling-based MPC, constraints                                                  | II.13–II.16                       | done    |
| Safety filters                | Control barrier functions, envelope protection                                                   | II.17, V.5                        | partly  |
| Estimation, linear            | Kalman filter, observability, detectability, separation                                          | II.4, III.9                       | done    |
| Estimation, nonlinear         | Complementary and Mahony filters, EKF, multiplicative EKF, UKF, particle filter, smoothing       | III.22–III.24, IV.22, IV.27–IV.29 | partly  |
| Disturbance observers         | ADRC, extended state observer, incremental control (INDI)                                        | II.5–II.9                         | done    |
| Nonlinear stability           | Lyapunov, LaSalle, region of attraction, describing functions, circle criterion                  | III.7, III.8                      | done    |
| Nonlinear design              | Dynamic inversion, geometric control on SO(3), differential flatness, sliding mode, backstepping | II.7–II.12, III.15, III.16, IV.15 | partly  |
| Robust control                | Uncertainty models, small gain, singular values, disk margins, H∞ loop shaping, μ analysis       | III.11–III.14                     | done    |
| Adaptive control              | MRAC, robust modifications, L1 adaptive control                                                  | II.18, III.17                     | done    |
| Learning-based control        | Learned residual models, neural-network policies, reinforcement learning                         | II.19–II.21                       | done    |
| Digital implementation        | Sampling, z-plane, aliasing, quantisation, rate limits, jitter, fixed-point code                 | I.9, III.18–III.20, V.3           | partly  |
| Filtering of real sensors     | Vibration, notch and RPM-tracking filters, anti-alias filters                                    | I.8, III.20, III.21               | done    |
| System identification         | Least squares, relay tuning, frequency-domain identification with coherence                      | III.25, III.26                    | done    |
| Fault detection and tolerance | Residual tests, redundancy and voting, control after actuator loss                               | II.7, III.27, III.28, V.4         | partly  |
| Verification                  | Monte Carlo campaigns, pass rates with confidence, worst cases, requirements and budgets         | III.29, III.30, V.1, V.2, V.6     | partly  |
| Flight dynamics               | Trim, linearisation, longitudinal modes, stability augmentation, handling qualities              | IV.11, IV.12                      | done    |
| Gain scheduling               | Point designs, interpolation, hidden coupling                                                    | IV.13, IV.14                      | partly  |
| Launch vehicles               | Thrust-vector control, conditional stability, bending, slosh                                     | IV.7–IV.10                        | done    |
| Spacecraft attitude           | Quaternion feedback, reaction wheels, momentum management, thrusters, flexible appendages        | IV.18–IV.23                       | partly  |
| Guidance and navigation       | Proportional navigation, relative orbital motion, inertial navigation                            | IV.24–IV.26                       | partly  |
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

Short answers to every exercise are printed in the book's Answers appendix (collected by `\answer` during the build); complete worked solutions are left for a companion volume.

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

| Volume | Parts | Title                                                    | State                                        |
| ------ | ----- | -------------------------------------------------------- | -------------------------------------------- |
| 1      | I–II  | _Anemostatos_ — from PID to learning                     | written; back matter being finished          |
| 2      | III   | analysis, robustness and the real loop                   | not begun; the app's Part III is done        |
| 3      | IV–V  | aerospace guidance, navigation and control, and practice | after the app's Parts IV–V                   |
| —      | all   | worked solutions to the exercises                        | short answers are in each volume (`\answer`) |

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
| I        | Linear algebra: singular values, positive definiteness, the Riccati equation        | II.2         | 1      | —                                         |
| J        | Optimisation: convexity, KKT conditions, quadratic and cone programs                | II.13        | 1      | —                                         |
| K        | Norms of signals and systems                                                        | III.11       | 2      | —                                         |
| L        | Calculus of variations and dynamic programming                                      | IV.1         | 3      | —                                         |

### 5.3 State of volume 1, verified 2026-10-02

Checked on the build of commit `84078df`: the sources, the build log, the
rendered pages and `book/tools/check_margins.py`. The book has 535 pages
(i–xi, then 1–524): Chapter 0 and the Prelude, Part I on pages 19–201,
Part II on pages 202–415, the toolbox on pages 416–511, then the
references, photo credits and colophon. The first check, on commit
`8e1df9c`, found 127 pages and Part I below the standard.

**What is good.** All thirty-five lessons are written: I.1–I.14, and
II.1–II.21 in the five chapters A–E of Part II. Every lesson meets the list
of §4: at least three worked examples (164 in all), at least eight
exercises (335, every one with an `\answer`), each lesson with at least one
`derive`, one `fly` and one `code` exercise, an _In Anemostatos_ box, a
_Try this_ experiment, an _Out in the World_ page, an engineer's view
ending in _In formal terms_ (76 theorems across the two parts), a
where-this-leads box and a figure from the simulator's flights. Every
number quoted from the simulator comes from an experiment in
`scripts/book-data.ts`. The toolbox has all ten appendices of volume 1,
A–J, and the PDF has no "??" left and no undefined reference. Every
chapter from Chapter 0 to II.21, and the toolbox's opener and appendices
A–H, open with a quotation kept in `book/front/quotes.tex`, each checked
against its source. The front matter is new since the first check: a
black cover with an emblem of a feedback loop standing in the wind, a page
that explains the name, a technical preface on how the book is built and
how it goes with the simulator, and a Chapter 0 that opens with the story
of Ingenuity on Mars. `check_margins.py` reports, besides colliding margin
notes, any text, drawing or image below the foot of the text block; on
this build it reports nothing (299 notes). The build takes about ninety
seconds for its two passes.

**What is missing or wrong.**

| #   | Finding                                                                                                                                                                                                                                                 | State                                                                                         |
| --- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| 1   | Lessons I.5–I.14 had no worked example and three to five exercises each, over five to seven pages. They did not meet §4.                                                                                                                                | fixed (rewritten to the level of I.1–I.4)                                                     |
| 2   | Part II was a title page followed by nothing; its five chapter files were stubs, while the title page promises thirty-five lessons.                                                                                                                     | fixed (II.1–II.21, chapters A–E)                                                              |
| 3   | The mathematical toolbox and the app map were stubs.                                                                                                                                                                                                    | toolbox fixed (A–J, 96 pages); app map: 15                                                    |
| 4   | The check marks of the _What to remember_ boxes were missing: the math font has no such glyph.                                                                                                                                                          | fixed (drawn with TikZ)                                                                       |
| 5   | `main.tex` and `scripts/book-data.ts` referred to `make book`, `make book-data` and `book/README.md`, none of which existed.                                                                                                                            | fixed (three make targets, the README)                                                        |
| 6   | The imprint named a script `book/tools/figures` that does not exist.                                                                                                                                                                                    | fixed                                                                                         |
| 7   | No author, editor or credit anywhere in the book.                                                                                                                                                                                                       | fixed (title page, imprint, colophon)                                                         |
| 8   | The field map of Chapter 0 listed Bode, Nyquist, root locus, Lyapunov, Pontryagin, dynamic programming and H∞ against Parts I–II, which do not teach them.                                                                                              | fixed (the map names the part that does)                                                      |
| 9   | Chapter 0 and the preface described a two-part course.                                                                                                                                                                                                  | fixed (they describe the five parts)                                                          |
| 10  | Lines too wide. Now four: lesson I.11 by 9.7 pt (a paragraph), I.12 by 4.8 pt (the chain equation), II.2 by 3.7 pt (a list item), the contents by 1.3 pt.                                                                                               | open                                                                                          |
| 11  | Chapters opened without a quotation.                                                                                                                                                                                                                    | fixed, except appendices I and J                                                              |
| 12  | Margin notes, margin figures and boxes ran below the foot of the page after the quotations and the new Chapter 0 moved them; nothing checked for it.                                                                                                    | fixed (`check_margins.py` checks the foot)                                                    |
| 13  | MPPI shifted its warm start one step per re-plan, 2.5 times faster than time. Fixed in `src/control/mppi.ts`; the MPPI flights of II.16 and II.21 re-flown, their figures and numbers updated. The ranking and the conclusions did not change.          | fixed                                                                                         |
| 14  | The answers to the 335 exercises are collected into `build/main.ans` and printed nowhere; `book/back/answers.tex` is a stub. When they are printed, §4 and §5.1, which send the solutions to a companion volume, change with them.                      | fixed: the Answers appendix (16 pages); §4 and §5.1 updated                                   |
| 15  | The app map (`book/back/appmap.tex`) is a stub and prints nothing.                                                                                                                                                                                      | fixed: the map of the application (10 pages)                                                  |
| 16  | No index.                                                                                                                                                                                                                                               | fixed: an index of 338 entries; `make book` sorts it with upmendex                            |
| 17  | No back cover.                                                                                                                                                                                                                                          | fixed: a back cover in the cover's style                                                      |
| 18  | Underfull pages in lessons II.15–II.20.                                                                                                                                                                                                                 | fixed in II.15 and II.17; II.16 and II.20 had none left                                       |
| 19  | Lesson II.21 has no screenshot of the app; the screenshot of II.16 dates from before the MPPI fix (finding 13).                                                                                                                                         | fixed: II.16 re-shot, II.21 shows the arena's results table                                   |
| 20  | Lessons I.5 and I.11 show no screenshot, although `book/screens/integral.jpg` and `challenge.jpg` are committed.                                                                                                                                        | open                                                                                          |
| 21  | After the two passes of `make book` the log still asks for a rerun ("Label(s) may have changed"), and the count it prints of "lines that mention undefined references" counts font-shape warnings: there is no undefined reference.                     | fixed: three passes (the third settles the labels); the count is of undefined references only |
| 22  | Ten lessons named parts of the screen that do not exist ("Controller › L1 controller", "Outer loops", "Physics › Mass", feedforward, assumed mass and controller rate under "Controller").                                                              | fixed against `src/engine/schema.ts` (the app map lists the real names)                       |
| 23  | The II.21 screenshot (the arena's results table, two crops of a scrolled panel) was taken with a one-off script; `book/tools/shots.mjs` has no entry that can retake it.                                                                                | open                                                                                          |
| 24  | `book/tools/check_fill.py` (page fill per lesson) finds about ten pages under 80 % outside II.15–II.20, in Part I and II.1–II.5 (e.g. PDF pages 38, 113, 250); it also counts a part or chapter title page that follows a lesson as that lesson's page. | open                                                                                          |

## 6. Order of work

1. **M13 and the must lessons of Part III** in the app
   ([analysis.md](analysis.md) §5). Everything later uses its charts and
   its Monte Carlo runner. Done.
2. **Volume 1 of the book to the standard of §4**: lessons I.5–I.14, Part II,
   appendices A–J (A–H first, I and J with Part II). Written; the back
   matter is being finished (§5.3).
3. **The rest of Part III**, then volume 2. The app's part is done; volume
   2 is not begun.
4. **M19, the vehicle abstraction**, then the must lessons of Part IV,
   then Part V ([aerospace-gnc.md](aerospace-gnc.md) §5). Under way: M19,
   M20 and M22 are done, M23–M25 begun.
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
