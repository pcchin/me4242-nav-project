"""
Calibrate the SOFA simulation of the continuum robot from the collected data, so that the simulated tip goes where
the real tip went for the same motor angles.

Each motor's angle is mapped to the length its cable shortens the robot by, with 2 parameters per motor:
    shortening_i = gain_i * PULLEY_RADIUS * (direction_i * angle_i - slack_i)   (released, no pull, if <= 0)
    - slack_i (rad): angle the motor turns before its cable starts pulling (slack cable),
    - gain_i: how strongly the cable bends the robot once taut (1 = ideal cable).
Plus the robot's length, its base position (the marker is at the tip) and the angle of each cable around the robot's
axis (the cable holes need not be aligned with the camera's axes).

The fit uses a constant-curvature model of the robot (fast, close to what SOFA computes for cable-driven bending);
then check it in SOFA (the continuum scene loads the result automatically).

Usage (from this folder, in the emio-ml environment):
    python calibrate_sim.py          # data/samples.csv -> models/sim_calibration.json + models/sim_calibration.png
"""
import argparse
import json
import os

import numpy as np
from scipy.optimize import least_squares

import config
import train_model

CABLE_OFFSET = 6.  # mm, distance from the backbone axis to the cable holes (as in continuum.py)
NOMINAL_CABLE_ANGLES = np.array([0., 0.5, 1., 1.5]) * np.pi  # rad around y, 0 = +z, pi/2 = +x (cables 0..3)


def unpack(p):
    return {"slack": p[0:4], "gain": p[4:8], "length": p[8], "base": p[9:12], "cable_angles": p[12:16]}


def shortenings(theta, params, directions=config.MOTOR_DIRECTIONS, pulleyRadius=config.PULLEY_RADIUS):
    """Length (mm) each cable shortens the robot by, for motor angles (n, 4). Released cables: 0."""
    wound = np.atleast_2d(theta) * directions
    return np.clip(params["gain"] * pulleyRadius * (wound - params["slack"]), 0., None)


def tipPositions(theta, params):
    """Constant-curvature tip positions (n, 3) for motor angles (n, 4)."""
    a = shortenings(theta, params)
    angles = params["cable_angles"]
    bend = a @ np.stack([np.sin(angles), np.cos(angles)], axis=1)  # bending vector (x, z), mm
    phi = np.linalg.norm(bend, axis=1) / CABLE_OFFSET  # bending angle (rad)
    psi = np.arctan2(bend[:, 0], bend[:, 1])  # bending direction around y
    length = params["length"]
    safe = np.where(phi < 1e-9, 1., phi)
    rho = np.where(phi < 1e-9, 0., length / safe * (1. - np.cos(phi)))
    drop = np.where(phi < 1e-9, length, length / safe * np.sin(phi))
    return params["base"] + np.stack([rho * np.sin(psi), -drop, rho * np.cos(psi)], axis=1)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", nargs="+", default=[os.path.join(here, "data", "samples.csv")])
    parser.add_argument("--out", default=os.path.join(here, "models", "sim_calibration.json"))
    args = parser.parse_args()

    data = train_model.loadData(args.data)
    theta, xyz = data[train_model.THETA].values, data[train_model.XYZ].values
    rest = xyz[np.argmin(np.abs(theta).sum(axis=1))]

    # Initial guess: small slack, half-efficient cables, robot hanging from y = 0 down to the rest position
    p0 = np.concatenate([[0.1] * 4, [0.5] * 4, [-rest[1]], [rest[0], 0., rest[2]], NOMINAL_CABLE_ANGLES])
    low = np.concatenate([[-0.2] * 4, [0.02] * 4, [60.], [-30., -100., -30.], NOMINAL_CABLE_ANGLES - np.pi / 2])
    high = np.concatenate([[2.5] * 4, [3.] * 4, [300.], [30., 100., 30.], NOMINAL_CABLE_ANGLES + np.pi / 2])
    p0 = np.clip(p0, low + 1e-6, high - 1e-6)  # the "rest" sample may be a bent pose when no rest pose was measured

    def residuals(p):
        return (tipPositions(theta, unpack(p)) - xyz).ravel()

    best = None
    for gain in (0.3, 0.6, 1.0):  # a few starts: the slack/gain trade-off has local minima
        p0[4:8] = gain
        result = least_squares(residuals, p0, bounds=(low, high), loss="soft_l1", f_scale=3.)
        if best is None or result.cost < best.cost:
            best = result
    params = unpack(best.x)
    errors = np.linalg.norm(tipPositions(theta, params) - xyz, axis=1)

    print(f"Calibrated on {len(xyz)} samples: tip error mean {errors.mean():.2f} mm, 95% "
          f"{np.percentile(errors, 95):.2f} mm (the learned model: see train_model.py)")
    print("motor   slack (rad)   slack (mm of cable)   gain   cable angle (deg, 0 = +z, 90 = +x)")
    for i in range(4):
        print(f"  {i}      {params['slack'][i]:+.3f}        {params['slack'][i] * config.PULLEY_RADIUS:+6.2f}"
              f"           {params['gain'][i]:.2f}   {np.degrees(params['cable_angles'][i]):+6.1f}")
    print(f"robot length {params['length']:.1f} mm, base at {np.round(params['base'], 1)} mm "
          f"(rest tip at {np.round(params['base'] - [0, params['length'], 0], 1)})")
    print("Negative slack: the cable is already pulling at 0 rad (pre-tension).")

    calibration = {"slack_rad": params["slack"].tolist(), "gain": params["gain"].tolist(),
                   "length": float(params["length"]), "base": params["base"].tolist(),
                   "cable_angles_rad": params["cable_angles"].tolist(),
                   "cable_offset": CABLE_OFFSET, "pulley_radius": config.PULLEY_RADIUS,
                   "directions": config.MOTOR_DIRECTIONS.tolist(),
                   "error_mean_mm": float(errors.mean()), "n_samples": int(len(xyz)),
                   "max_theta": float(np.abs(theta).max())}  # angle range of the data: trust the fit up to there
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(calibration, f, indent=2)
    print(f"Saved {os.path.abspath(args.out)}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fitted = tipPositions(theta, params)
        fig, axes = plt.subplots(1, 2, figsize=(13, 6))
        axes[0].scatter(xyz[:, 0], xyz[:, 2], s=12, label="measured tip")
        axes[0].scatter(fitted[:, 0], fitted[:, 2], s=12, marker="x", label="calibrated simulation")
        for m, f_ in zip(xyz, fitted):
            axes[0].plot([m[0], f_[0]], [m[2], f_[2]], color="gray", lw=0.5, alpha=0.6)
        axes[0].set(xlabel="x (mm)", ylabel="z (mm)", aspect="equal", title="Seen from above, same motor angles")
        axes[0].legend()
        axes[0].grid(alpha=0.3)
        axes[1].hist(errors, bins=30)
        axes[1].set(xlabel="tip error (mm)", ylabel="samples", title=f"Calibrated simulation error, "
                                                                      f"mean {errors.mean():.2f} mm")
        fig.tight_layout()
        plot = os.path.splitext(args.out)[0] + ".png"
        fig.savefig(plot, dpi=120)
        print(f"Plot: {plot}")
    except ImportError:
        pass


if __name__ == "__main__":
    main()
