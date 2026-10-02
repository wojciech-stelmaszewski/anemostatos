import {
  BIAS_ALLOCATION,
  finishedBudget,
  MARGIN,
  NOISE_ALLOCATION,
  REQUIREMENT,
  windAllocation,
} from '@/analysis/budget';
import { CAMPAIGN_RUNS, finishedVerification, turbulence } from '@/analysis/campaign';
import { finishedReview, REVIEW } from '@/analysis/review';
import {
  FIXED,
  fixedDisagreement,
  fixedScenario,
  TRANSFER,
  trackingPeak,
  transferJump,
  VOTER,
  voterError,
  voterScenario,
} from '@/analysis/software';
import { integratorDeadZone, integratorRange } from '@/control/part5/fixedpoint';
import { PHYS_DT } from '@/engine/simulation';
import type { Params } from '@/sim/params';
import { BudgetPanel } from '@/ui/part5/BudgetPanel';
import { CampaignPanel } from '@/ui/part5/CampaignPanel';
import { ReviewPanel } from '@/ui/part5/ReviewPanel';
import { Notice, Try } from './Bits';
import { M } from './Math';
import type { Lesson } from './types';

/**
 * Part V — from whiteboard to flight (docs/aerospace-gnc.md, Chapter P): what turns a control law
 * into flight software that can be signed off. Numbering restarts at V.1 in the lesson panel.
 */

export const P = 'P · From whiteboard to flight';

/** The environment of lessons V.1 and V.2: turbulence, a noisy altimeter with a bias. */
const ALTIMETER = { noise: 0.01, bias: 0.03 };
const environment = (p: Params) => {
  turbulence(p);
  p.sensors.posNoise = ALTIMETER.noise;
  p.sensors.altBias = ALTIMETER.bias;
};

/** The design lesson V.1 ends with: more gain for the wind, a slower D filter for the noise. */
const budgetDesign = (p: Params) => {
  Object.assign(p.control.alt, { kp: 80, ki: 8, kd: 18, dFilterHz: 4 });
};

/** 95 % lower bound on the pass rate after `n` clean flights, %: 0.05^(1/n). */
const provenRate = (n: number): number => 100 * 0.05 ** (1 / n);

const cm = (v: number) => (v * 100).toFixed(1);

/** The controller's sample period, s (an integer number of physics steps). */
const controlPeriod = (p: Params): number =>
  Math.max(1, Math.round(1 / (p.control.rateHz * PHYS_DT))) * PHYS_DT;
/** Lesson V.3: the dead zone of the integrator as the code has it (ki rounded to Q8.8), m. */
const deadZone = (p: Params): number =>
  integratorDeadZone(
    Math.round(p.control.alt.ki * 256) / 256,
    controlPeriod(p),
    p.part5.fixed.intFrac,
  );

/** Lesson V.4: B is isolated once its drift passes the threshold and has stayed there, s. */
const isolationDelay = (p: Params): number =>
  p.part5.voter.threshold / Math.abs(p.part5.voter.driftRate) + p.part5.voter.persistMs / 1000;

/** Lesson V.5: the PID that flies first, tuned to take up a payload in a few seconds. */
const handOver = (p: Params) => {
  p.wind.enabled = false;
  p.drone.mass = TRANSFER.mass;
  Object.assign(p.control.alt, { kp: 20, ki: 6, kd: 9 });
  p.control.l1.kind = 'transfer';
  p.control.lqr.integral = true;
  p.part5.transfer.at = 15;
};

/** Lesson V.6: what the review is handed — the design of V.2 and the software as delivered. */
const delivered = (p: Params) => {
  environment(p);
  Object.assign(p.control.alt, { kp: 75, ki: 7.5, kd: 22, dFilterHz: 4 });
  // The model the controller was tuned on dates from before the larger battery.
  p.control.model.mass = 0.8;
  p.part5.voter.mode = 'mid';
};

/** Lessons of Part V, in plan order (by `n`). */
export const PART_FIVE: Lesson[] = [
  {
    id: 'budget',
    n: 1,
    part: 5,
    chapter: P,
    title: 'What does "good" mean?',
    level: 1,
    setup: environment,
    predict: {
      label: 'Allocation left for the wind',
      unit: 'cm',
      truth: () => windAllocation() * 100,
      tolerance: 0.05,
    },
    goal: {
      text: `Predict what the wind may take. Then tune the altitude PID until every source stays within its allocation and the confirmation flight stays within ${cm(REQUIREMENT * (1 - MARGIN))} cm: the budget closes with 20 % left over.`,
      check: ({ sim, prediction }) => {
        const truth = windAllocation() * 100;
        if (prediction == null)
          return 'first the prediction: what is left for the wind once the margin, the bias and the noise have their share?';
        if (Math.abs(prediction - truth) / truth > 0.05)
          return 'not within 5 %: the biases add, and what is left is shared in squares';
        const r = finishedBudget(sim.params);
        if (!r) return 'press Measure the budget with these settings';
        const over = r.entries.filter((e) => !e.within).map((e) => e.source.label.toLowerCase());
        if (over.length) return `over its allocation: ${over.join(', ')}`;
        return (
          r.closes ||
          `every entry fits, but all of them together give ${cm(r.flown.total)} cm: the sources are not independent here`
        );
      },
    },
    solution: budgetDesign,
    body: (
      <>
        <p>
          Every part so far ended with "it flies well". A customer does not buy "well". They write a{' '}
          <b>requirement</b>: <i>hold the altitude within 10 cm, 3σ</i>. A Gaussian error stays
          inside three standard deviations 99.7 % of the time. The requirement is written on{' '}
          <M>{'|\\bar e| + 3\\sigma'}</M>, so that an error which is always there, a bias, counts in
          full.
        </p>
        <p>
          Three things move the drone off its setpoint here: turbulence of 1.5 m/s, an altimeter
          with 1 cm of noise, and the same altimeter reading 3 cm high. A systems engineer splits
          the requirement into a <b>budget</b> before anything flies. First a margin is kept back
          for what nobody has thought of, 20 % here. The bias gets {cm(BIAS_ALLOCATION)} cm (the
          altimeter is specified to 3 cm), the noise {cm(NOISE_ALLOCATION)} cm, and the wind gets
          what is left.
        </p>
        <p>
          The shares do not simply add. A bias is the same in every flight, and two biases can line
          up, so means add: <M>{'\\bar e = \\sum |\\bar e_i|'}</M>. Independent random errors partly
          cancel, and their variances add: <M>{'\\sigma^2 = \\sum \\sigma_i^2'}</M>. The budget is
          therefore
        </p>
        <M display>
          {
            '\\sum_i |\\bar e_i| + 3\\sqrt{\\sum_i \\sigma_i^2} \\;\\le\\; (1 - 0.2)\\cdot 10\\ \\text{cm}'
          }
        </M>
        <p>
          Each entry is a number that the earlier parts taught you to measure. The panel below flies
          the drone once per source, with all the others switched off, then once with all of them
          together to check that the budget adds up.
        </p>
        <BudgetPanel />
        <Try>
          Work out how many centimetres the wind may take and enter it. Measure the budget with the
          Part I gains: two rows are red. The wind wants more gain. The noise reaches the motors
          through the D term, which differentiates it, and more gain makes that worse. Find the knob
          in the altitude PID that the noise row responds to, and measure again after each change.
        </Try>
        <Notice>
          The confirmation flight agrees with the budget within about 5 %. That is not automatic: it
          holds because the three sources are independent and the loop is close to linear. With the
          D filter at 20 Hz and high gain, the D term turns the noise into thrust jitter large
          enough to shift the mean, not only the spread. The loop is then no longer linear, and
          entries measured one at a time need not add. Only the confirmation flight can tell you.
          Budgets like this one are how landing accuracy, telescope pointing and navigation errors
          are specified in industry: every line has an owner, and the margin is spent only by a
          decision.
        </Notice>
      </>
    ),
  },
  {
    id: 'campaign',
    n: 2,
    part: 5,
    chapter: P,
    title: 'The verification campaign',
    level: 1,
    setup: (p) => {
      environment(p);
      budgetDesign(p);
    },
    predict: {
      label: `Pass rate ${CAMPAIGN_RUNS} clean flights prove`,
      unit: '%',
      truth: () => provenRate(CAMPAIGN_RUNS),
      tolerance: 0.02,
    },
    goal: {
      text: `Predict what ${CAMPAIGN_RUNS} clean flights prove. Then make every requirement green: ${CAMPAIGN_RUNS} of ${CAMPAIGN_RUNS} flights in each campaign.`,
      check: ({ sim, prediction }) => {
        if (prediction == null)
          return `first the prediction: the rate p for which ${CAMPAIGN_RUNS} passes in a row still happen 5 % of the time`;
        if (Math.abs(prediction / provenRate(CAMPAIGN_RUNS) - 1) > 0.02)
          return 'not within 2 %: solve p^N = 0.05';
        const r = finishedVerification(sim.params);
        if (!r) return 'press Fly the campaign with this design';
        const red = r.filter((x) => !x.green);
        return (
          red.length === 0 ||
          red.map((x) => `${x.requirement.id}: ${x.passed}/${x.runs.length}`).join(', ')
        );
      },
    },
    solution: (p) => {
      Object.assign(p.control.alt, { kp: 75, ki: 7.5, kd: 22, dFilterHz: 4 });
    },
    body: (
      <>
        <p>
          The budget of V.1 closed for one drone, the nominal one, in one wind. The drones that
          leave the factory differ. Their mass is known to 20 %, motors age and slow down, and one
          altimeter is noisier than the next. The design is signed off when it meets its
          requirements for all of them, and the evidence is a <b>verification campaign</b>
          [Hanson 2010].
        </p>
        <ul className="list-disc space-y-0.5 pl-4">
          <li>
            A <b>requirement table</b>. Each line is testable and has its own scenario: R1 is the
            budget of V.1 in turbulence, R2 and R3 are a 1 m step in still air.
          </li>
          <li>
            A <b>dispersion table</b>: every uncertain parameter and its range. Each flight draws
            all of them at random.
          </li>
          <li>
            One <b>campaign per requirement</b>: {CAMPAIGN_RUNS} flights. All campaigns use the same
            draws, so a change of design is compared on the same drones.
          </li>
          <li>
            A <b>report</b>: the flights that passed, the pass rate that proves with 95 %
            confidence, and the worst case with the values that produced it.
          </li>
        </ul>
        <CampaignPanel />
        <Try>
          How high a pass rate do {CAMPAIGN_RUNS} clean flights prove? Enter it, then fly the
          campaign with the design V.1 ended with. Read the worst case of the red line: which
          dispersion is at its end of the range? Retune and fly again. The requirements pull in
          different directions: R1 wants gain, and R2 wants damping on drones whose motors are slow.
        </Try>
        <Notice>
          Twenty clean flights prove only 86 %. A launch vehicle is cleared with thousands, because
          a requirement is a probability and a campaign is how it is measured. Gusts are not in the
          table. They are not Gaussian: with the default gusts, two wind seeds in nine give the
          design of V.1 an |ē| + 3σ of 30 to 60 cm, and no 3σ requirement survives that. In practice
          a gust gets its own requirement, on the peak error and the recovery. A green report also
          proves nothing outside the dispersion table. Choosing its ranges, and defending them, is
          part of the design.
        </Notice>
      </>
    ),
  },
  {
    id: 'fixedpoint',
    n: 3,
    part: 5,
    chapter: P,
    title: 'The controller is code',
    level: 1,
    chart: 'backtoback',
    setup: (p) => {
      fixedScenario(p);
      // This version of the code has no feedforward: the integrator carries the weight.
      p.control.feedforward = false;
    },
    predict: {
      label: 'Smallest error the integrator can see',
      unit: 'cm',
      truth: (p) => deadZone(p) * 100,
      tolerance: 0.05,
    },
    goal: {
      text: `Predict the integrator's dead zone. Then choose its word and scaling so that the fixed-point PID agrees with the reference within ${FIXED.limit} % of the hover thrust over ${FIXED.end} s.`,
      check: ({ sim, prediction }) => {
        if (prediction == null)
          return 'first the prediction: below which error does kᵢ·e·T round to zero steps of the integrator?';
        if (Math.abs(prediction / (deadZone(sim.params) * 100) - 1) > 0.05)
          return 'not within 5 %: one step of the integrator is 2^−bits N, and kᵢ·e·T must reach it';
        if (!sim.params.part5.fixed.shadow) return 'switch the fixed-point shadow on';
        const worst = fixedDisagreement(sim);
        if (worst > FIXED.limit)
          return `they disagree by ${worst.toFixed(1)} % of the hover thrust`;
        if (sim.t < FIXED.end - 1e-6)
          return `${worst.toFixed(2)} % so far; keep flying to ${FIXED.end} s`;
        return true;
      },
    },
    solution: (p) => {
      p.part5.fixed.intBits = 32;
      p.part5.fixed.intFrac = 16;
    },
    body: (
      <>
        <p>
          The PID of Part I is three lines of floating-point arithmetic. The computer that flies a
          small drone, or the actuator controller of an airliner, may have no floating point at all.
          The same law is then written in integers. Each signal gets a unit (micrometres,
          micronewtons) and each gain a fixed binary point (here Q8.8: steps of 1/256). The integers
          must neither overflow nor lose what matters below their last bit.
        </p>
        <p>
          The panel keeps the drone flying on the floating-point PID and runs an integer copy beside
          it on the same sensor readings. Comparing the two outputs sample by sample is called a{' '}
          <b>back-to-back test</b>. It is how flight code is checked against the model it was
          generated from. Everything here is 32-bit except the integrator, whose word length and
          binary point are yours to choose.
        </p>
        <p>
          With <M>{'F'}</M> fractional bits the integrator moves in steps of <M>{'2^{-F}'}</M> N.
          Each sample it adds <M>{'k_i\\,e\\,T'}</M>, rounded towards zero, so an error smaller than{' '}
          <M>{'2^{-F}/(k_i T)'}</M> adds nothing at all. In a 16-bit word it can hold at most{' '}
          <M>{'2^{15-F}'}</M> N before it wraps round to a large negative number.
        </p>
        <Try>
          Compute the dead zone for 8 fractional bits, kᵢ = 0.8 and T = 4 ms, and enter it. Watch
          the two integrators on the chart: the reference climbs to the drone's weight, the copy
          stops early. Try 11 and 12 bits ({integratorRange(16, 11)} N and {integratorRange(16, 12)}{' '}
          N of range), then a 32-bit word. Then try 16 bits with the feedforward on: the integrator
          then holds only what the model gets wrong.
        </Try>
        <Notice>
          With no feedforward the integrator must hold 9.8 N. A 16-bit word cannot hold that and
          still resolve the centimetre errors of a hover. At 11 bits the dead zone is 15 cm, and the
          copy falls 14 % behind. At 12 bits the word wraps, and the output jumps by the whole
          range. There are two fixes, and both are used in flight code: a 32-bit accumulator, or
          less for the integrator to hold, which here means the feedforward. Neither problem shows
          in a simulation run in floating point. That is why the code itself is tested against the
          model, and why tools that generate code from a model check its scaling.
        </Notice>
      </>
    ),
  },
  {
    id: 'voter',
    n: 4,
    part: 5,
    chapter: P,
    title: 'Three sensors, one truth',
    level: 1,
    chart: 'altimeters',
    setup: (p) => {
      voterScenario(p);
      p.part5.voter.mode = 'mid';
    },
    predict: {
      label: 'From the start of B’s drift to its isolation',
      unit: 's',
      truth: isolationDelay,
      tolerance: 0.1,
    },
    goal: {
      text: `Predict how long the monitor takes to isolate the drifting altimeter. Then choose a voter that keeps the altitude it flies on within ${cm(VOTER.limit)} cm of the truth through both failures, to ${VOTER.end} s.`,
      check: ({ sim, prediction }) => {
        if (prediction == null)
          return 'first the prediction: how long until B is past the threshold, and then how long must it stay there?';
        if (Math.abs(prediction / isolationDelay(sim.params) - 1) > 0.1)
          return 'not within 10 %: the threshold over the drift rate, plus the persistence';
        const e = voterError(sim);
        if (!Number.isFinite(e)) return 'it crashed';
        if (e > VOTER.limit) return `the voted altitude was ${cm(e)} cm off the truth`;
        if (sim.t < VOTER.end - 1e-6)
          return `worst so far ${cm(e)} cm; keep flying to ${VOTER.end} s`;
        return true;
      },
    },
    solution: (p) => {
      p.part5.voter.mode = 'monitor';
    },
    body: (
      <>
        <p>
          A sensor that fails silently is the most dangerous kind. An altimeter that freezes still
          reports a plausible altitude. One that drifts reports one that is a little worse each
          second. Flight control systems carry several sensors of each kind and a <b>voter</b> that
          decides which reading the control law gets.
        </p>
        <p>
          Here there are three altimeters. At 10 s altimeter A freezes; from 20 s altimeter B drifts
          down at 5 cm/s. The setpoint steps up and down by half a metre, so a frozen reading shows.
        </p>
        <ul className="list-disc space-y-0.5 pl-4">
          <li>
            <b>A alone</b>: no redundancy. The drone flies on whatever A says.
          </li>
          <li>
            <b>Mid-value selection</b>: the middle of three readings. One wrong channel can never be
            the middle one, so the first failure is masked without being noticed.
          </li>
          <li>
            <b>Mid-value with a monitor</b>: a channel that stays more than 15 cm from the middle
            value for 200 ms is isolated for good. With two channels left there is no middle, and a
            disagreement does not say which one is wrong. The monitor then compares each with the
            altitude carried forward by the measured vertical speed: a second, dissimilar source.
          </li>
        </ul>
        <Try>
          Fly A alone first and watch what a frozen altimeter does to a drone told to climb. Then
          fly mid-value selection: the first failure goes unnoticed, and B's drift is followed once
          A is out of the way. Predict when the monitor will isolate B, then fly it.
        </Try>
        <Notice>
          Mid-value selection survives one failure only because nothing has failed yet as far as it
          knows. It keeps no record, so the second failure meets a system that has already used up
          its redundancy. Detection and isolation, as in lesson III.28, are what make two failures
          survivable. They also cost something: a threshold low enough to catch B early isolates a
          healthy channel on a noisy day, and a false alarm uses up the redundancy as surely as a
          fault. Real systems add the dissimilar source (an inertial or GPS altitude) for exactly
          the case where two channels disagree.
        </Notice>
      </>
    ),
  },
  {
    id: 'bumpless',
    n: 5,
    part: 5,
    chapter: P,
    title: 'Change controllers in flight',
    level: 1,
    setup: handOver,
    predict: {
      label: 'Jump in thrust at an unprepared hand-over',
      unit: 'N',
      truth: transferJump,
      tolerance: 0.05,
    },
    goal: {
      text: `Predict the kick. Then hand over from the PID to the LQI so that the drone moves less than ${cm(TRANSFER.limit)} cm in the ${TRANSFER.window} s that follow.`,
      check: ({ sim, prediction }) => {
        if (prediction == null)
          return 'first the prediction: what was the PID’s integrator holding when it was switched off?';
        if (Math.abs(prediction / transferJump(sim.params) - 1) > 0.05)
          return 'not within 5 %: the payload’s weight, which the model does not know about';
        const at = sim.params.part5.transfer.at;
        if (sim.t < at + TRANSFER.window - 1e-6)
          return `wait for the hand-over at ${at} s and after`;
        const peak = trackingPeak(sim, at, at + TRANSFER.window);
        return peak <= TRANSFER.limit || `the drone moved ${cm(peak)} cm after the hand-over`;
      },
    },
    solution: (p) => {
      p.part5.transfer.bumpless = true;
    },
    body: (
      <>
        <p>
          A real flight control system is a set of <b>modes</b>: take-off, hover, cruise, landing, a
          degraded mode for when a sensor is lost. Logic switches between them, and each switch
          hands the vehicle from one control law to another in flight. Here the drone carries a
          payload of {(TRANSFER.mass - 1).toFixed(2)} kg that the model does not know about. It
          flies the PID, and at 15 s it changes over to the LQI of Part II.
        </p>
        <p>
          The PID has had fifteen seconds to learn the payload: its integrator holds the extra
          thrust. The LQI starts with its own integrator at zero, so its first command lacks exactly
          that much. A <b>bumpless transfer</b> presets the incoming controller's state so that its
          first command equals the last one of the outgoing controller. Here that means the LQI's
          integrator:
        </p>
        <M display>{'-k_i\\,\\xi_0 = u_{PID} - u_{LQI}(\\xi=0)'}</M>
        <Try>
          Predict the jump in thrust, then fly the hand-over as it is and watch the altitude when
          the LQI takes over. Then tick <b>Bumpless transfer</b> and fly it again.
        </Try>
        <Notice>
          The kick is the integrator's content, the part of the command that encodes what the model
          does not know. Any state that one controller has learned and the next has not causes this:
          integrators, filters, an estimator's bias. Mode logic in flight software is mostly about
          such transfers, about which commands are allowed in which mode, and about the envelope
          that no mode may leave. The safety filter of lesson II.17 is one such limit.
        </Notice>
      </>
    ),
  },
  {
    id: 'review',
    n: 6,
    part: 5,
    chapter: P,
    title: 'The design review',
    level: 1,
    setup: delivered,
    goal: {
      text: 'Hold the design review and close every item: model, margins, campaign, faults and software.',
      check: ({ sim }) => {
        const items = finishedReview(sim.params);
        if (!items) return 'press Hold the review with these settings';
        const open = items.filter((i) => !i.passed).map((i) => i.title.toLowerCase());
        return open.length === 0 || `open: ${open.join(', ')}`;
      },
    },
    solution: (p) => {
      Object.assign(p.control.alt, { kp: 65, ki: 6.5, kd: 18, dFilterHz: 40 });
      p.control.l1.estimator = 'kalman';
      p.control.model.mass = 1;
      p.part5.voter.mode = 'monitor';
      p.part5.fixed.intBits = 32;
      p.part5.fixed.intFrac = 16;
    },
    body: (
      <>
        <p>
          The last lesson is the one every design goes through before it flies: a{' '}
          <b>design review</b>. The design and its software are handed over with a requirement set,
          and a reviewer asks for evidence item by item. The app plays the reviewer. It flies the
          design you have now through every check of this part:
        </p>
        <ul className="list-disc space-y-0.5 pl-4">
          <li>
            <b>Model</b>: a probe sweep measures the plant. Near crossover it must agree with the
            plant the controller was designed on (its <M>{'\\hat m'}</M> and <M>{'\\hat\\tau'}</M>)
            within {REVIEW.modelDb} dB and {REVIEW.modelDeg}°.
          </li>
          <li>
            <b>Margins</b>: from the same sweep, {REVIEW.gmDb} dB of gain margin both ways and{' '}
            {REVIEW.pmDeg}° of phase margin, the classical rule of lesson III.30.
          </li>
          <li>
            <b>Campaign</b>: the requirement table of V.2, every line green.
          </li>
          <li>
            <b>Faults</b>: the two altimeter failures of V.4.
          </li>
          <li>
            <b>Software</b>: the fixed-point copy of V.3, within {FIXED.limit} %.
          </li>
        </ul>
        <ReviewPanel />
        <Try>
          Hold the review on the design as it was handed over. Four items are open. Close them one
          at a time: find which parameter each check points at, and re-run the review after each
          change. The margins are the hard one. Read the campaign's limits first, then ask which
          part of the loop costs the phase.
        </Try>
        <Notice>
          In a search over kp, kd and the D filter, no PID on this noisy altimeter both passed the
          campaign and kept 45° of phase margin: the low D filter that tames the noise is what costs
          the phase. The way out is not a gain but an architectural change, a Kalman filter that
          blends in the accelerometer, and that is the usual outcome of a review. A review does not
          judge a controller. It judges the evidence that a whole system (model, law, software,
          redundancy) meets what was asked of it, and it says what is still missing. In industry the
          same items, in far more detail, are what DO-178C and ARP4754A ask an aircraft's flight
          control system to show.
        </Notice>
      </>
    ),
  },
];
