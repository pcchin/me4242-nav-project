"""
Collect training data on the real robot: drive the 4 motors through many angle combinations and record where the tip
marker ends up, seen by the depth camera.

Before running:
    1. Calibrate the camera (Emio Labs, camera calibration module) and put one coloured marker on the tip.
    2. Mount the cables taut with all the motors at 0 rad, and check MOTOR_DIRECTIONS in config.py.
    3. Close Emio Labs / SOFA (they would hold the motors' port).

Usage (from this folder, in the emio-ml environment):
    python collect_data.py --n 200 --max-theta 1.0 --out data/stage1.csv    # a first stage, up to 1 rad
    python collect_data.py --max-theta 2.5 --out data/stage2.csv            # the full range
    python collect_data.py --simulate --out data/sim.csv                     # no hardware: made-up robot
Ctrl+C stops cleanly (motors released). Run the same command again to resume where it stopped.
Each file holds one plan (seed, n, max angle): use a new --out for a different plan.

Tracking: the tip is looked for near where the calibrated physics model (models/sim_calibration.json, from
calibrate_sim.py) predicts it, so a stray blob elsewhere is never taken for the tip. If the measured tip stops moving
while the predicted tip moves a lot, the tracker is stuck: the samples are marked `tracking_lost` and the tip is found
again at rest.
"""
import argparse
import csv
import json
import os
import time

import numpy as np
from scipy.stats import qmc

import config
from robot_io import EmioRobot, FakeRobot, OverCurrentError, rampTo

COLUMNS = (["sample_id", "kind", "status"]
           + [f"theta{i}_cmd" for i in range(4)] + [f"theta{i}_meas" for i in range(4)]
           + [f"current{i}" for i in range(4)]
           + ["x", "y", "z", "std_x", "std_y", "std_z", "n_frames", "timestamp", "max_theta", "plan"])
REACH_MARGIN = 30.  # mm, added to the robot's length for the reach check (the fitted length and base are estimates)


def planSamples(n, nRepeats, seed, maxTheta=config.MAX_THETA):
    """
    Safe angle samples covering the 4D space within +-maxTheta (Sobol sequence), visited in a short path order,
    plus repeats of some samples at the end (reached from another direction: repeatability and hysteresis)
    and a return to rest every 100 samples (drift). Returns a list of (sample_id, kind, theta).
    """
    sobol = qmc.Sobol(d=4, scramble=True, seed=seed)
    candidates = qmc.scale(sobol.random(2 ** int(np.ceil(np.log2(4 * n)))), [-maxTheta] * 4, [maxTheta] * 4)
    candidates = candidates[config.isSafe(candidates, maxTheta=maxTheta)][:n]

    # Greedy nearest-neighbour path from rest, so the robot moves little between samples
    order, current, remaining = [], np.zeros(4), list(range(len(candidates)))
    while remaining:
        nearest = remaining[int(np.argmin(np.linalg.norm(candidates[remaining] - current, axis=1)))]
        order.append(nearest)
        remaining.remove(nearest)
        current = candidates[nearest]

    plan = []
    for k, index in enumerate(order):
        if k % 100 == 0:
            plan.append((-1, "rest", np.zeros(4)))
        plan.append((int(index), "sample", candidates[index]))
    rng = np.random.default_rng(seed)
    for index in rng.choice(len(candidates), size=min(nRepeats, len(candidates)), replace=False):
        plan.append((int(index), "repeat", candidates[index]))
    plan.append((-1, "rest", np.zeros(4)))
    return plan


def tipPredictor(simulate, path):
    """
    (predict, validUpTo, reach): `predict` maps motor angles (4,) to the predicted tip (3,), or is None if no prediction
    is available; `validUpTo` is the largest angle (rad) the prediction can be trusted at (the calibration's data range);
    `reach` is (base, radius): the tip can't be farther than the robot's length from its base.
    """
    if simulate:
        return (lambda theta: FakeRobot.tipFromAngles(theta)[0]), np.inf, (np.zeros(3), FakeRobot.LENGTH + REACH_MARGIN)
    if path is None or not os.path.exists(path):
        print("No simulation calibration found: the tip is tracked without prediction (run calibrate_sim.py).")
        return None, 0., None
    import calibrate_sim
    with open(path) as f:
        c = json.load(f)
    params = {"slack": np.array(c["slack_rad"]), "gain": np.array(c["gain"]), "length": c["length"],
              "base": np.array(c["base"]), "cable_angles": np.array(c["cable_angles_rad"])}
    validUpTo = c.get("max_theta", 0.45) + 0.1
    print(f"Tracking the tip near its predicted position (calibrated physics, {c['error_mean_mm']:.1f} mm on the "
          f"data, gate {config.PREDICTION_GATE:.0f} mm) for angles up to {validUpTo:.2f} rad. Beyond, the prediction "
          f"is only used to detect a stuck tracker: recalibrate (calibrate_sim.py) after each stage.")
    reach = (np.array(c["base"]), c["length"] + REACH_MARGIN)
    return (lambda theta: calibrate_sim.tipPositions(np.asarray(theta), params)[0]), validUpTo, reach


class StuckDetector:
    """The measured tip barely moves over several samples while the predicted tip moves a lot: the tracker is stuck."""

    def __init__(self):
        self.measured, self.predicted = [], []

    def update(self, tip, prediction):
        if tip is None or prediction is None:
            return False
        self.measured = (self.measured + [np.asarray(tip)])[-config.STUCK_SAMPLES:]
        self.predicted = (self.predicted + [np.asarray(prediction)])[-config.STUCK_SAMPLES:]
        if len(self.measured) < config.STUCK_SAMPLES:
            return False
        spread = lambda points: np.max(np.linalg.norm(np.array(points) - np.mean(points, axis=0), axis=1))
        return spread(self.measured) < config.STUCK_MEASURED_SPREAD and \
            spread(self.predicted) > config.STUCK_PREDICTED_SPREAD

    def reset(self):
        self.measured, self.predicted = [], []


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--n", type=int, default=1200, help="number of samples")
    parser.add_argument("--repeats", type=int, default=30, help="number of samples measured a second time")
    parser.add_argument("--seed", type=int, default=0, help="seed of the sample plan (same seed = resume)")
    parser.add_argument("--max-theta", type=float, default=config.MAX_THETA,
                        help=f"maximum motor angle of this collection (rad, at most {config.MAX_THETA})")
    parser.add_argument("--out", default=os.path.join(here, "data", "samples.csv"))
    parser.add_argument("--calibration", default=os.path.join(here, "models", "sim_calibration.json"),
                        help="simulation calibration used to predict the tip for tracking ('none' to disable)")
    parser.add_argument("--simulate", action="store_true", help="use a made-up robot instead of the hardware")
    parser.add_argument("--simulate-trap", action="store_true",
                        help="with --simulate: the camera loses the tip at large bends and sees a stray blob")
    args = parser.parse_args()

    maxTheta = min(args.max_theta, config.MAX_THETA)
    plan = planSamples(args.n, args.repeats, args.seed, maxTheta)
    planId = f"seed{args.seed}_n{args.n}_r{args.repeats}_max{maxTheta:g}"

    # Resume: only in a file made with the same columns and the same plan
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    done = 0
    if os.path.exists(args.out) and os.path.getsize(args.out) > 0:
        with open(args.out, newline="") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            if reader.fieldnames != COLUMNS or any(row.get("plan") != planId for row in rows):
                raise SystemExit(f"{args.out} holds data from another collection (other columns or plan). "
                                 f"Use a new --out for this plan ({planId}).")
            done = len(rows)
        print(f"{args.out} already has {done} rows of this plan: resuming at step {done + 1}/{len(plan)}.")
    newFile = not os.path.exists(args.out) or os.path.getsize(args.out) == 0

    predict, validUpTo, reach = tipPredictor(args.simulate, None if args.calibration == "none" else args.calibration)
    if args.simulate:
        trap = {"hideBeyond": 1.5, "strayBlob": [-34., -55., 163.]} if args.simulate_trap else {}
        robot = FakeRobot(seed=args.seed, **trap)
    else:
        robot = EmioRobot()
    if reach is not None:
        robot.setReach(*reach)
    # Is the calibration still right for this robot? The tip found at rest must be where it predicts the rest tip
    if predict is not None and validUpTo > 0 and not args.simulate:
        restError = np.linalg.norm(np.asarray(robot._lastTip) - predict(np.zeros(4)))
        if restError > config.PREDICTION_GATE / 2:
            print(f"WARNING: the tip at rest is {restError:.0f} mm from where the calibration predicts it: the "
                  f"calibration is out of date (robot or marker changed?). Tracking without the prediction gate; "
                  f"run calibrate_sim.py on new data.")
            validUpTo = -1.
    print(f"Connected to the {'made-up' if args.simulate else 'real'} robot. {len(plan) - done} steps to go, "
          f"angles up to {maxTheta:g} rad.")

    stuck = StuckDetector()
    trackingLost = False
    commanded = robot.angles
    start = time.time()
    try:
        with open(args.out, "a", newline="") as f:
            writer = csv.writer(f)
            if newFile:
                writer.writerow(COLUMNS)
            for step, (sampleId, kind, theta) in enumerate(plan[done:], start=done + 1):
                status = "ok"
                prediction = predict(theta) if predict is not None else None
                try:
                    commanded = rampTo(robot, theta, start=commanded)
                    if kind == "rest":
                        # Find the tip again on the axis and refresh the background blobs
                        robot.setPrediction(None)
                        robot.waitSettled()
                        try:
                            robot.refreshTracking()
                            stuck.reset()
                            trackingLost = False
                        except RuntimeError as e:
                            print(f"  step {step}: tip not found again at rest ({e}). Is the robot caught on "
                                  f"something? Samples stay marked tracking_lost.")
                    # Gate the tracking on the prediction only where the calibration can be trusted
                    robot.setPrediction(prediction if prediction is not None and np.abs(theta).max() <= validUpTo
                                        else None)
                    if not robot.waitSettled():
                        status = "not_settled"
                    tip, std, nFrames = robot.readTip()
                    if tip is None:
                        status, tip, std = "no_marker", [np.nan] * 3, [np.nan] * 3
                    elif trackingLost or stuck.update(tip, prediction):
                        if not trackingLost:
                            print(f"  step {step}: the measured tip stopped moving at {np.round(tip, 1)} while the "
                                  f"predicted tip moves: tracking lost until the next rest pose")
                        trackingLost = True
                        status = "tracking_lost"
                except OverCurrentError as e:
                    print(f"  step {step}: {e}, skipped")
                    status, tip, std, nFrames = "overcurrent", [np.nan] * 3, [np.nan] * 3, 0
                    commanded = robot.angles

                writer.writerow([sampleId, kind, status] + list(np.round(theta, 6))
                                + list(np.round(robot.angles, 6)) + list(np.round(robot.currents, 1))
                                + list(np.round(tip, 3)) + list(np.round(std, 3))
                                + [nFrames, round(time.time(), 2), maxTheta, planId])
                f.flush()

                if step % 20 == 0 or step == len(plan):
                    elapsed = time.time() - start
                    remaining = elapsed / (step - done) * (len(plan) - step)
                    print(f"  {step}/{len(plan)}  tip={np.round(tip, 1)}  ({status})  ~{remaining / 60:.0f} min left")
    except KeyboardInterrupt:
        print("\nStopped. Run the same command again to resume.")
    finally:
        try:
            rampTo(robot, np.zeros(4), start=commanded, maxCurrent=np.inf)
        finally:
            robot.close()
    print(f"Data in {os.path.abspath(args.out)}")


if __name__ == "__main__":
    main()
