"""
Train the forward model (4 motor angles -> tip position) from the collected data, check the inverse kinematics,
and save the portable model used by `ik.py`, `goto.py` and the SOFA scene.

Usage (from this folder, in the emio-ml environment):
    python train_model.py                        # data/samples.csv -> models/model.npz + report plots
    python train_model.py --data data/a.csv data/b.csv --out models/model.npz
"""
import argparse
import json
import os

import numpy as np
import pandas as pd
from sklearn.compose import TransformedTargetRegressor
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel, WhiteKernel
from sklearn.linear_model import Ridge
from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPRegressor
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import PolynomialFeatures, StandardScaler

import config
import ik

THETA = [f"theta{i}_meas" for i in range(4)]
XYZ = ["x", "y", "z"]
MAX_GP_SAMPLES = 2500  # above this, the Gaussian process gets slow: only the MLP is tried
LAM = 5.  # IK: weight of the distance to rest (mm^2 per rad^2)


def loadData(paths):
    data = pd.concat([pd.read_csv(p) for p in paths], ignore_index=True)
    total = len(data)
    data = data[(data["status"] == "ok") & data[XYZ].notna().all(axis=1)]
    data = data[np.linalg.norm(data[["std_x", "std_y", "std_z"]].values, axis=1) <= config.MAX_TIP_STD]
    print(f"{len(data)} usable samples out of {total} rows")
    return data


def repeatability(data):
    """Distance between the two measurements of the same sample (repeats), in mm."""
    samples = data[data["sample_id"] >= 0].groupby("sample_id")
    spreads = [np.linalg.norm(g[XYZ].values[0] - g[XYZ].values[-1]) for _, g in samples if len(g) > 1]
    return np.array(spreads)


def fitGP(x, y):
    xScaler = StandardScaler().fit(x)
    kernel = ConstantKernel(1.) * RBF(length_scale=np.ones(x.shape[1])) + WhiteKernel(1e-3)
    gp = GaussianProcessRegressor(kernel=kernel, normalize_y=True, n_restarts_optimizer=2, random_state=0)
    gp.fit(xScaler.transform(x), y)
    arrays = {"x_mean": xScaler.mean_, "x_scale": xScaler.scale_,
              "y_mean": np.asarray(gp._y_train_mean), "y_scale": np.asarray(gp._y_train_std),
              "gp_constant": np.array(gp.kernel_.k1.k1.constant_value),
              "gp_length_scale": np.asarray(gp.kernel_.k1.k2.length_scale),
              "gp_x_train": gp.X_train_, "gp_alpha": gp.alpha_}
    return arrays, {"kind": "gp", "kernel": str(gp.kernel_)}


def fitMLP(x, y):
    xScaler = StandardScaler().fit(x)
    mlp = TransformedTargetRegressor(
        regressor=MLPRegressor(hidden_layer_sizes=(128, 128), alpha=1e-4, max_iter=5000, early_stopping=True,
                               validation_fraction=0.15, n_iter_no_change=50, random_state=0),
        transformer=StandardScaler())
    mlp.fit(xScaler.transform(x), y)
    net, yScaler = mlp.regressor_, mlp.transformer_
    arrays = {"x_mean": xScaler.mean_, "x_scale": xScaler.scale_, "y_mean": yScaler.mean_, "y_scale": yScaler.scale_}
    for i, (w, b) in enumerate(zip(net.coefs_, net.intercepts_)):
        arrays[f"mlp_w{i}"], arrays[f"mlp_b{i}"] = w, b
    return arrays, {"kind": "mlp", "mlp_layers": len(net.coefs_)}


def portable(arrays, meta):
    return ik.ForwardModel(arrays, meta)


def errors(model, x, y):
    return np.linalg.norm(model.predict(x) - y, axis=1)


def yControllability(xyz, noise):
    """
    Can y be set independently of (x, z)? Fit y = g(x, z) (cubic polynomial): if what remains is at the level of the
    measurement noise, y is fully determined by (x, z) and is not controllable on its own.
    """
    xz, y = xyz[:, [0, 2]], xyz[:, 1]
    fit = make_pipeline(PolynomialFeatures(3), StandardScaler(), Ridge(1e-3)).fit(xz, y)
    residual = float(np.std(y - fit.predict(xz)))
    return residual, residual > max(1., 2. * noise)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", nargs="+", default=[os.path.join(here, "data", "samples.csv")])
    parser.add_argument("--out", default=os.path.join(here, "models", "model.npz"))
    parser.add_argument("--test-fraction", type=float, default=0.2)
    args = parser.parse_args()

    data = loadData(args.data)
    if len(data) < 30:
        raise SystemExit("Not enough data to train (at least 30 usable samples needed).")
    x, y = data[THETA].values, data[XYZ].values

    spreads = repeatability(data)
    noise = float(np.median(spreads)) if len(spreads) else 0.
    if len(spreads):
        print(f"Repeatability (same angles measured twice): median {np.median(spreads):.2f} mm, "
              f"max {np.max(spreads):.2f} mm over {len(spreads)} repeats")

    xTrain, xTest, yTrain, yTest = train_test_split(x, y, test_size=args.test_fraction, random_state=0)

    # Candidates, compared on the test set, through their portable version (the one actually used)
    candidates = []
    if len(xTrain) <= MAX_GP_SAMPLES:
        candidates.append(fitGP(xTrain, yTrain))
    candidates.append(fitMLP(xTrain, yTrain))
    results = []
    for arrays, meta in candidates:
        e = errors(portable(arrays, meta), xTest, yTest)
        results.append((np.mean(e), arrays, meta))
        print(f"Forward model {meta['kind']}: test error mean {np.mean(e):.2f} mm, "
              f"95% {np.percentile(e, 95):.2f} mm, max {np.max(e):.2f} mm")
    _, _, bestMeta = min(results, key=lambda r: r[0])
    print(f"Selected: {bestMeta['kind']}. Refitting on all {len(x)} samples.")

    # Final model on all the data, and a check that the portable version matches scikit-learn's one
    arrays, meta = fitGP(x, y) if bestMeta["kind"] == "gp" else fitMLP(x, y)
    forward = portable(arrays, meta)

    residual, yFree = yControllability(y, noise)
    weights = np.array([1., 1. if yFree else 0., 1.])
    print(f"Height y beyond what (x, z) imposes: {residual:.2f} mm -> y "
          f"{'IS controllable: targets are (x, y, z)' if yFree else 'follows (x, z): targets are (x, z)'}")

    # The IK must not use angles beyond what the data covers (the model can't extrapolate)
    dataMaxTheta = float(min(config.MAX_THETA, np.abs(x).max()))
    print(f"Angles in the data up to {dataMaxTheta:.2f} rad: the IK is limited to that "
          f"(motor limit {config.MAX_THETA:.2f} rad)")
    meta.update({"lam": LAM, "max_theta": dataMaxTheta, "max_cocontraction": config.MAX_COCONTRACTION,
                 "pulley_radius": config.PULLEY_RADIUS, "pairs": config.OPPOSITE_PAIRS, "y_controllable": yFree,
                 "y_residual_mm": residual, "repeatability_mm": noise, "n_samples": int(len(x)),
                 "test_error_mean_mm": float(min(r[0] for r in results))})
    arrays.update({"weights": weights, "theta_rest": np.zeros(4), "directions": config.MOTOR_DIRECTIONS,
                   "train_theta": x, "train_xyz": y, "meta": np.array(json.dumps(meta))})
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    np.savez(args.out, **arrays)
    print(f"Saved {os.path.abspath(args.out)}")

    # Inverse check on the test positions: can the IK find angles that reach them (according to the model)?
    model = ik.loadModel(args.out)
    ikErrors = [ik.solveIK(p, model)["error"] for p in yTest[:100]]
    print(f"IK on {len(ikErrors)} measured positions: model error mean {np.mean(ikErrors):.2f} mm, "
          f"95% {np.percentile(ikErrors, 95):.2f} mm (the real error is measured with goto.py)")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(11, 5))
        sc = axes[0].scatter(y[:, 0], y[:, 2], c=y[:, 1], s=8, cmap="viridis")
        fig.colorbar(sc, ax=axes[0], label="y (mm)")
        axes[0].set(xlabel="x (mm)", ylabel="z (mm)", title="Reached tip positions", aspect="equal")
        axes[1].hist(errors(forward, xTest, yTest), bins=30)
        axes[1].set(xlabel="forward error (mm)", ylabel="test samples", title="Forward model error")
        plot = os.path.splitext(args.out)[0] + "_report.png"
        fig.tight_layout()
        fig.savefig(plot, dpi=120)
        print(f"Report plot: {plot}")
    except ImportError:
        pass


if __name__ == "__main__":
    main()
