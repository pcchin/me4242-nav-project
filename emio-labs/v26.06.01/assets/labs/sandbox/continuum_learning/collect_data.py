"""
Collect training data on the real robot: drive the 4 motors through many angle combinations and record where the tip
marker ends up, seen by the depth camera.

Before running:
    1. Calibrate the camera (Emio Labs, camera calibration module) and put one coloured marker on the tip.
    2. Mount the cables taut with all the motors at 0 rad, and check MOTOR_DIRECTIONS in config.py.
    3. Close Emio Labs / SOFA (they would hold the motors' port).

Usage (from this folder, in the emio-ml environment):
    python collect_data.py                       # 1200 samples + 30 repeats, appends to data/samples.csv
    python collect_data.py --n 200               # a first, short session
    python collect_data.py --simulate            # no hardware: made-up robot, to test the scripts
Ctrl+C stops cleanly (motors released). Run again with the same --seed to resume where it stopped.
"""
import argparse
import csv
import os
import time

import numpy as np
from scipy.stats import qmc

import config
from robot_io import EmioRobot, FakeRobot, OverCurrentError, rampTo

COLUMNS = (["sample_id", "kind", "status"]
           + [f"theta{i}_cmd" for i in range(4)] + [f"theta{i}_meas" for i in range(4)]
           + [f"current{i}" for i in range(4)]
           + ["x", "y", "z", "std_x", "std_y", "std_z", "n_frames", "timestamp"])


def planSamples(n, nRepeats, seed):
    """
    Safe angle samples covering the 4D space (Sobol sequence), visited in a short path order,
    plus repeats of some samples at the end (reached from another direction: repeatability and hysteresis)
    and a return to rest every 100 samples (drift). Returns a list of (sample_id, kind, theta).
    """
    sobol = qmc.Sobol(d=4, scramble=True, seed=seed)
    candidates = qmc.scale(sobol.random(2 ** int(np.ceil(np.log2(4 * n)))), [-config.MAX_THETA] * 4,
                           [config.MAX_THETA] * 4)
    candidates = candidates[config.isSafe(candidates)][:n]

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


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--n", type=int, default=1200, help="number of samples")
    parser.add_argument("--repeats", type=int, default=30, help="number of samples measured a second time")
    parser.add_argument("--seed", type=int, default=0, help="seed of the sample plan (same seed = resume)")
    parser.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "data", "samples.csv"))
    parser.add_argument("--simulate", action="store_true", help="use a made-up robot instead of the hardware")
    args = parser.parse_args()

    plan = planSamples(args.n, args.repeats, args.seed)

    # Resume: skip the plan steps already in the file for this seed
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    done = 0
    if os.path.exists(args.out):
        with open(args.out, newline="") as f:
            done = sum(1 for row in csv.DictReader(f) if row.get("timestamp"))
        print(f"{args.out} already has {done} rows: resuming at step {done + 1}/{len(plan)}.")
    newFile = not os.path.exists(args.out) or os.path.getsize(args.out) == 0

    robot = FakeRobot(seed=args.seed) if args.simulate else EmioRobot()
    print(f"Connected to the {'made-up' if args.simulate else 'real'} robot. {len(plan) - done} steps to go.")

    commanded = robot.angles
    start = time.time()
    try:
        with open(args.out, "a", newline="") as f:
            writer = csv.writer(f)
            if newFile:
                writer.writerow(COLUMNS)
            for step, (sampleId, kind, theta) in enumerate(plan[done:], start=done + 1):
                status = "ok"
                try:
                    commanded = rampTo(robot, theta, start=commanded)
                    if not robot.waitSettled():
                        status = "not_settled"
                    tip, std, nFrames = robot.readTip()
                    if tip is None:
                        status, tip, std = "no_marker", [np.nan] * 3, [np.nan] * 3
                except OverCurrentError as e:
                    print(f"  step {step}: {e}, skipped")
                    status, tip, std, nFrames = "overcurrent", [np.nan] * 3, [np.nan] * 3, 0
                    commanded = robot.angles

                writer.writerow([sampleId, kind, status] + list(np.round(theta, 6))
                                + list(np.round(robot.angles, 6)) + list(np.round(robot.currents, 1))
                                + list(np.round(tip, 3)) + list(np.round(std, 3)) + [nFrames, round(time.time(), 2)])
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
