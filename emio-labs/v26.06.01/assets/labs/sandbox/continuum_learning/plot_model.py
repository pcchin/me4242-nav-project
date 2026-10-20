"""
Plot the collected data (motor angles -> tip position) with the trained model.

The model has 4 inputs (motor angles) and 3 outputs (tip x, y, z), so it is shown in three views:
    1. Effect of each motor: tip coordinate vs the angle of one motor. Dots: the data. Line: the model's average
       prediction when that motor's angle sweeps its range, the other motors as in the data (partial dependence).
       Dashed: the model with the other motors at rest (0 rad).
    2. Predicted vs measured, on held-out samples: the same 80/20 split as train_model.py, the model refitted on the
       80% only, so the points were not seen during fitting.
    3. Workspace: measured tip positions seen from above (x, z), with the model's predicted tip positions for a grid
       of bends (each opposite pair wound/released by the same amount).

Usage (from this folder, in the emio-ml environment):
    python plot_model.py                       # data/samples.csv + models/model.npz -> models/plot_*.png
    python plot_model.py --data data/a.csv --model models/model.npz --show
"""
import argparse
import os

import numpy as np

import config
import ik
import train_model

COORDINATES = ["x", "y", "z"]


def plotMotorEffects(plt, data, model, path):
    theta, xyz = data[train_model.THETA].values, data[train_model.XYZ].values
    forward = model["forward"]
    sweep = np.linspace(-model["max_theta"], model["max_theta"], 41)
    fig, axes = plt.subplots(3, 4, figsize=(15, 9), sharex=True)
    for motor in range(4):
        # Partial dependence: the motor's angle set to each sweep value, the other angles as in the data
        average, rest = [], []
        for value in sweep:
            varied = theta.copy()
            varied[:, motor] = value
            average.append(forward.predict(varied).mean(axis=0))
            alone = np.zeros(4)
            alone[motor] = value
            rest.append(forward.predict(alone)[0])
        average, rest = np.array(average), np.array(rest)
        for c, name in enumerate(COORDINATES):
            ax = axes[c, motor]
            ax.scatter(theta[:, motor], xyz[:, c], s=8, alpha=0.35, color="tab:blue", label="data")
            ax.plot(sweep, average[:, c], color="tab:red", lw=2, label="model (average)")
            ax.plot(sweep, rest[:, c], color="tab:red", lw=1.2, ls="--", label="model (others at 0)")
            ax.grid(alpha=0.3)
            if c == 0:
                ax.set_title(f"motor {motor}")
            if c == 2:
                ax.set_xlabel("angle (rad)")
            if motor == 0:
                ax.set_ylabel(f"tip {name} (mm)")
    axes[0, 0].legend(fontsize=8, loc="best")
    fig.suptitle("Effect of each motor on the tip position: data and trained model")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    return fig


def plotPredictedVsMeasured(plt, data, model, path, testFraction=0.2):
    from sklearn.model_selection import train_test_split
    x, y = data[train_model.THETA].values, data[train_model.XYZ].values
    xTrain, xTest, yTrain, yTest = train_test_split(x, y, test_size=testFraction, random_state=0)
    fit = train_model.fitGP if model["kind"] == "gp" else train_model.fitMLP
    arrays, meta = fit(xTrain, yTrain)
    predicted = ik.ForwardModel(arrays, meta).predict(xTest)
    error = np.linalg.norm(predicted - yTest, axis=1)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for c, name in enumerate(COORDINATES):
        ax = axes[c]
        low, high = min(yTest[:, c].min(), predicted[:, c].min()), max(yTest[:, c].max(), predicted[:, c].max())
        pad = 0.05 * (high - low + 1e-9)
        ax.plot([low - pad, high + pad], [low - pad, high + pad], color="gray", lw=1, label="perfect")
        ax.scatter(yTest[:, c], predicted[:, c], s=18, color="tab:purple", alpha=0.8, label="held-out samples")
        rmse = np.sqrt(np.mean((predicted[:, c] - yTest[:, c]) ** 2))
        ax.set(xlabel=f"measured {name} (mm)", ylabel=f"predicted {name} (mm)", title=f"{name}: RMSE {rmse:.2f} mm",
               aspect="equal")
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.suptitle(f"Model on {len(yTest)} held-out samples: mean 3D error {error.mean():.2f} mm, "
                 f"95% {np.percentile(error, 95):.2f} mm")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    return fig


def plotWorkspace(plt, data, model, path, nbLines=9):
    xyz = data[train_model.XYZ].values
    forward = model["forward"]
    limit = model["max_theta"]
    values = np.linspace(-limit, limit, nbLines)
    fine = np.linspace(-limit, limit, 60)

    def bend(a, b):
        # Pair 0/2 driven by a, pair 1/3 by b: one cable wound, the opposite released by the same length
        directions = model["directions"]
        return np.array([a, b, -a * directions[0] / directions[2], -b * directions[1] / directions[3]])

    fig, ax = plt.subplots(figsize=(8, 8))
    sc = ax.scatter(xyz[:, 0], xyz[:, 2], c=xyz[:, 1], s=14, cmap="viridis", label="measured tip", zorder=3)
    fig.colorbar(sc, ax=ax, label="tip y (mm)", shrink=0.8)
    for k, a in enumerate(values):
        line = forward.predict(np.array([bend(a, b) for b in fine]))
        ax.plot(line[:, 0], line[:, 2], color="tab:red", lw=0.8, alpha=0.7, label="model grid" if k == 0 else None)
        line = forward.predict(np.array([bend(b, a) for b in fine]))
        ax.plot(line[:, 0], line[:, 2], color="tab:red", lw=0.8, alpha=0.7)
    restTip = forward.predict(np.zeros(4))[0]
    ax.scatter([restTip[0]], [restTip[2]], marker="x", s=120, color="black", label="model at rest", zorder=4)
    ax.set(xlabel="x (mm)", ylabel="z (mm)", aspect="equal",
           title="Workspace seen from above: measured tip and model's predicted bends\n"
                 f"(grid: each opposite pair wound/released by the same amount, up to ±{limit:.2f} rad)")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    return fig


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", nargs="+", default=[os.path.join(here, "data", "samples.csv")])
    parser.add_argument("--model", default=os.path.join(here, "models", "model.npz"))
    parser.add_argument("--show", action="store_true", help="open the plots in windows")
    args = parser.parse_args()

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = train_model.loadData(args.data)
    model = ik.loadModel(args.model)
    out = os.path.dirname(os.path.abspath(args.model))
    paths = [os.path.join(out, name) for name in ["plot_motor_effects.png", "plot_predicted_vs_measured.png",
                                                  "plot_workspace.png"]]
    plotMotorEffects(plt, data, model, paths[0])
    plotPredictedVsMeasured(plt, data, model, paths[1])
    plotWorkspace(plt, data, model, paths[2])
    for p in paths:
        print(f"Saved {p}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
