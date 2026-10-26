"""
Learned inverse kinematics of the continuum robot.

The forward model f: motor angles (4, rad) -> tip position (x, y, z, mm) is learned from data (`train_model.py`)
and saved as `model.npz`, a portable format that only needs numpy and scipy (so it also runs in SOFA's Python).
The inverse is solved by optimisation:

    theta* = argmin ||W (f(theta) - p*)||^2 + lam ||theta - theta_rest||^2 + co-contraction penalty
             with |theta_i| <= maxTheta

The 4 angles are redundant (many angles reach the same position): the `lam` term picks the solution closest to rest,
and the penalty keeps opposite cables from being both wound (motors fighting).

Usage:
    model = loadModel("models/model.npz")
    result = solveIK([x, y, z], model)  # use y = nan to leave y free
    result["theta"], result["predicted"], result["error"], result["reachable"]
"""
import json
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.distance import cdist

COCONTRACTION_PENALTY = 1e6  # weight of the co-contraction penalty (per mm^2 beyond the limit)


class ForwardModel:
    """
    Portable forward model: a Gaussian process (RBF kernel) or an MLP (ReLU), with input and output scaling.
    Built from the arrays saved by `train_model.py`.
    """

    def __init__(self, arrays, meta):
        self.kind = meta["kind"]
        self.xMean, self.xScale = arrays["x_mean"], arrays["x_scale"]
        self.yMean, self.yScale = arrays["y_mean"], arrays["y_scale"]
        if self.kind == "gp":
            self.constant = float(arrays["gp_constant"])
            self.lengthScale = arrays["gp_length_scale"]
            self.xTrain = arrays["gp_x_train"] / self.lengthScale
            self.alpha = arrays["gp_alpha"]
        elif self.kind == "mlp":
            self.weights = [arrays[f"mlp_w{i}"] for i in range(meta["mlp_layers"])]
            self.biases = [arrays[f"mlp_b{i}"] for i in range(meta["mlp_layers"])]
        else:
            raise ValueError(f"Unknown model kind: {self.kind}")

    def predict(self, theta):
        """Tip positions (n, 3) in mm for angles (n, 4) or (4,) in rad."""
        x = (np.atleast_2d(theta) - self.xMean) / self.xScale
        if self.kind == "gp":
            k = self.constant * np.exp(-0.5 * cdist(x / self.lengthScale, self.xTrain, "sqeuclidean"))
            y = k @ self.alpha
        else:
            y = x
            for i, (w, b) in enumerate(zip(self.weights, self.biases)):
                y = y @ w + b
                if i < len(self.weights) - 1:
                    y = np.maximum(y, 0.)
        return y * self.yScale + self.yMean


def loadModel(path):
    """
    Load a model saved by `train_model.py`. Returns a dict with the forward model ("forward") and the IK settings:
    "weights" (W), "lam", "theta_rest", "max_theta", "max_cocontraction", "directions", "pulley_radius",
    "y_controllable", and the training data ("train_theta", "train_xyz") used for initial guesses.
    """
    data = np.load(path)
    meta = json.loads(str(data["meta"]))
    arrays = {k: data[k] for k in data.files}
    model = dict(meta)
    model["forward"] = ForwardModel(arrays, meta)
    for key in ["weights", "theta_rest", "directions", "train_theta", "train_xyz"]:
        model[key] = arrays[key]
    return model


def cocontraction(theta, model):
    """Length (mm) wound on both cables of each opposite pair."""
    w = np.asarray(theta) * model["directions"] * model["pulley_radius"]
    return np.array([max(0., min(w[i], w[j])) for i, j in model["pairs"]])


def solveIK(target, model, initialGuess=None, tolerance=2.):
    """
    Motor angles (rad) that bring the tip to `target` = [x, y, z] (mm). Give y = nan to leave it free.
    If the model says y is not controllable on its own, y is ignored anyway.

    Returns a dict:
        - "theta": the 4 motor angles (rad)
        - "predicted": tip position predicted for these angles (mm)
        - "error": distance between the predicted and the target position, on the controlled coordinates (mm)
        - "reachable": error below `tolerance` mm
    """
    target = np.asarray(target, dtype=float)
    weights = np.array(model["weights"], dtype=float)
    weights[np.isnan(target)] = 0.
    target = np.nan_to_num(target)
    forward = model["forward"]
    lam, rest = model["lam"], model["theta_rest"]
    maxCocontraction = model["max_cocontraction"]

    def cost(theta):
        e = weights * (forward.predict(theta)[0] - target)
        excess = np.clip(cocontraction(theta, model) - maxCocontraction, 0., None)
        return e @ e + lam * np.sum((theta - rest) ** 2) + COCONTRACTION_PENALTY * excess @ excess

    # Starting points: the previous solution, and the training samples closest to the target
    starts = [] if initialGuess is None else [np.asarray(initialGuess, dtype=float)]
    distances = np.linalg.norm(weights * (model["train_xyz"] - target), axis=1)
    starts += list(model["train_theta"][np.argsort(distances)[:2]])

    bounds = [(-model["max_theta"], model["max_theta"])] * 4
    best = None
    for start in starts:
        result = minimize(cost, np.clip(start, -model["max_theta"], model["max_theta"]), method="L-BFGS-B",
                          bounds=bounds)
        if best is None or result.fun < best.fun:
            best = result

    theta = best.x
    predicted = forward.predict(theta)[0]
    error = float(np.linalg.norm((weights > 0) * (predicted - target)))
    return {"theta": theta, "predicted": predicted, "error": error, "reachable": error <= tolerance}
