"""
Clean a data file: keep only the good samples, and drop readings where the camera tracker was stuck on a stray blob.

A stuck tracker is recognised with the calibrated physics model (models/sim_calibration.json): over several consecutive
samples the measured tip stands still while the predicted tip moves a lot. Every reading at such a stuck point that is
far from its predicted tip is dropped. (Slack cables also leave the tip still for different angles, but then the
physics model predicts it too, so those genuine samples are kept.)
The original file is not changed; the cleaned copy is written next to it (`<name>_clean.csv`).

Usage (from this folder, in the emio-ml environment):
    python clean_data.py data/samples.csv
"""
import argparse
import json
import os

import numpy as np
import pandas as pd

import calibrate_sim
import config

XYZ = ["x", "y", "z"]
THETA = [f"theta{i}_meas" for i in range(4)]
STUCK_RADIUS = 10.  # mm, readings this close to a stuck point...
FAR_FROM_PREDICTION = config.PREDICTION_GATE  # mm, ...and this far from their predicted tip are stuck readings


REACH_MARGIN = 30.  # mm: a reading farther than the robot's length (+ margin) from its base can't be the tip


def loadPredictor(path):
    """(predict, base, length) from a simulation calibration."""
    with open(path) as f:
        c = json.load(f)
    params = {"slack": np.array(c["slack_rad"]), "gain": np.array(c["gain"]), "length": c["length"],
              "base": np.array(c["base"]), "cable_angles": np.array(c["cable_angles_rad"])}
    return (lambda theta: calibrate_sim.tipPositions(theta, params)), np.array(c["base"]), c["length"]


def findStuckPoints(data, predicted):
    """Rows (boolean mask) whose reading is a stuck tracker, and the stuck points found."""
    valid = data[XYZ].notna().all(axis=1).values
    xyz = data[XYZ].values
    rows = np.where(valid)[0]
    n = config.STUCK_SAMPLES
    spread = lambda points: np.max(np.linalg.norm(points - points.mean(axis=0), axis=1))
    centers = []
    for k in range(len(rows) - n + 1):
        window = rows[k:k + n]
        if spread(xyz[window]) < config.STUCK_MEASURED_SPREAD and \
                spread(predicted[window]) > config.STUCK_PREDICTED_SPREAD:
            center = xyz[window].mean(axis=0)
            if not centers or np.min(np.linalg.norm(np.array(centers) - center, axis=1)) > STUCK_RADIUS:
                centers.append(center)
    stuck = np.zeros(len(data), dtype=bool)
    for center in centers:
        stuck |= valid & (np.linalg.norm(xyz - center, axis=1) < STUCK_RADIUS) & \
            (np.linalg.norm(xyz - predicted, axis=1) > FAR_FROM_PREDICTION)
    return stuck, centers


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("data")
    parser.add_argument("--out", default=None)
    parser.add_argument("--calibration", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "models",
                                                              "sim_calibration.json"))
    args = parser.parse_args()

    data = pd.read_csv(args.data)
    predict, base, length = loadPredictor(args.calibration)
    predicted = predict(data[THETA].values)
    stuck, centers = findStuckPoints(data, predicted)
    noisy = np.linalg.norm(data[["std_x", "std_y", "std_z"]].values, axis=1) > config.MAX_TIP_STD
    unreachable = np.linalg.norm(data[XYZ].values - base, axis=1) > length + REACH_MARGIN
    keep = (data["status"] == "ok").values & ~stuck & ~noisy & ~unreachable & data[XYZ].notna().all(axis=1).values

    print(f"{len(data)} rows: {data['status'].value_counts().to_dict()}")
    for center in centers:
        print(f"Stuck point at {np.round(center, 1)} mm")
    if stuck.any():
        print(f"Stuck readings: {int(stuck.sum())} rows, from row {int(np.argmax(stuck))} "
              f"({int((stuck & (data['status'] == 'ok').values).sum())} of them marked ok)")
    print(f"Out of the robot's reach (> {length + REACH_MARGIN:.0f} mm from its base): {int(unreachable.sum())} rows "
          f"({int((unreachable & (data['status'] == 'ok').values).sum())} of them marked ok)")
    print(f"Kept {int(keep.sum())} good samples (ok, not stuck, reachable, camera std <= {config.MAX_TIP_STD} mm)")

    out = args.out or os.path.splitext(args.data)[0] + "_clean.csv"
    data[keep].to_csv(out, index=False)
    print(f"Saved {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
