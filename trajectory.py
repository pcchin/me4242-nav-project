"""
Run a hardcoded trajectory on the real continuum robot: no camera, no model, no SOFA.

The trajectory is a path of bends (x, z) in rad, the same bends as teleop.py's W/S (x) and D/A (z): each opposite pair
winds one cable and releases the other by the same angle (x = motors 1/3, z = motors 0/2). The robot moves along the
path at a constant speed, starting and ending straight. Press Q (or Ctrl+C) to stop: it straightens and releases.

Shapes (scaled by --amplitude, rad):
    circle    a circle around the axis
    square    the 4 corners (+-A, +-A)
    figure8   a figure of eight through the center
    cross     out and back along +x, -x, +z, -z
Saved trajectories (SAVED below, fixed points and speed, --amplitude ignored):
    trajectory1   5 points from the user, 0.6 rad/s, ends straight
    trajectory2   bend toward -x (motor 1 at -2.27 rad) and back, 0.6 rad/s
    trajectory3   10 points from the user, up to 2.57 rad, 0.6 rad/s, ends straight
    (add your own to SAVED as motor angles)

This variant is meant to be used as a library to main.py and should not be ran independently.
"""

import numpy as np

from robot_io import EmioMotorsOnly, rampTo
from teleop import LOOP_PERIOD, MAX_CURRENT, MAX_THETA, SPEED, anglesFromBend, bendFromAngles, reset_bend,get_common_zero

TRAJECTORY_IS_ACTIVE = False
CURRENT_STAGE = 0 # Stage of angles to be at
loops = 1 # Default loop count
CURRENT_LOOP = 0 # Current loop count
CURRENT_LOOP_I = 0 # i in current loop
points = []

def get_trajectory_is_active():
    global TRAJECTORY_IS_ACTIVE
    return TRAJECTORY_IS_ACTIVE

# Saved trajectories: waypoints as the 4 motor angles (rad, motors 0 1 2 3, as teleop.py shows them) and speed (rad/s).
# Opposite motors must be opposite (motor 2 = -motor 0, motor 3 = -motor 1). To add one, copy a block and rename it.
# Variant: Instead of using a speed constant interval between ticks is used instead
SAVED = {
    "trajectory1": {
        "angles": [
            [+0.73, -1.80, -0.73, +1.80],
            [+0.80, -0.50, -0.80, +0.50],
            [+2.37, +2.03, -2.37, -2.03],
            [+0.93, -1.07, -0.93, +1.07],
            [+0.00, +0.00, +0.00, +0.00],
        ],
        "speed": 0.6,
    },
    "trajectory2": {
        "angles": [
            [+0.00, -2.27, -0.00, +2.27],
            [+0.00, +0.00, +0.00, +0.00],
        ],
        "speed": 0.6,
    },
    "trajectory3": {
        "angles": [
            [-0.20, -0.83, +0.20, +0.83],
            [+1.50, -0.83, -1.50, +0.83],
            [+1.30, +2.50, -1.30, -2.50],
            [-1.63, +2.03, +1.63, -2.03],
            [+1.33, -1.83, -1.33, +1.83],
            [-1.03, -1.53, +1.03, +1.53],
            [-1.07, +0.03, +1.07, -0.03],
            [-1.27, +2.57, +1.27, -2.57],
            [+1.07, +2.57, -1.07, -2.57],
            [+0.00, +0.00, +0.00, +0.00],
        ],
        "speed": 0.6,
    },
}
# SHAPES = ["circle", "square", "figure8", "cross", "custom"] + list(SAVED)
SHAPES = ["circle", "square", "figure8", "cross"] + list(SAVED)
CURVE_POINTS = 200  # points per loop of the smooth shapes
robot = EmioMotorsOnly()
commanded = None

def savedWaypoints(name):
    """A saved trajectory's motor angles as bends (n, 2); refuses angles whose opposite motors aren't opposite."""
    angles = np.array(SAVED[name]["angles"], dtype=float)
    unpaired = np.abs(angles[:, 0] + angles[:, 2]) + np.abs(angles[:, 1] + angles[:, 3]) > 0.011
    if unpaired.any():
        raise SystemExit(f"{name}: waypoint(s) {(np.where(unpaired)[0] + 1).tolist()} need motor 2 = -motor 0 and "
                         f"motor 3 = -motor 1")
    bends = bendFromAngles(angles.T)
    return np.stack([bends["x"], bends["z"]], axis=1)


def waypoints(shape, amplitude, custom=None):
    """The path's points (n, 2): bend toward +x and +z (rad), one loop."""
    t = np.linspace(0., 2. * np.pi, CURVE_POINTS + 1)
    if shape == "circle":
        return amplitude * np.stack([np.cos(t), np.sin(t)], axis=1)
    if shape == "figure8":
        return amplitude * np.stack([np.sin(t), np.sin(t) * np.cos(t)], axis=1)
    if shape == "square":
        return amplitude * np.array([[1, 1], [-1, 1], [-1, -1], [1, -1], [1, 1]], dtype=float)
    if shape == "cross":
        return amplitude * np.array([[0, 0], [1, 0], [0, 0], [-1, 0], [0, 0], [0, 1], [0, 0], [0, -1], [0, 0]],
                                    dtype=float)
    if shape in SAVED:
        return savedWaypoints(shape)
    if shape == "custom":
        if not custom:
            raise SystemExit('custom needs --waypoints, e.g. --waypoints "1,0; 0,1; -1,0"')
        return np.array([[float(v) for v in point.split(",")] for point in custom.split(";") if point.strip()])
    raise SystemExit(f"unknown shape {shape}, choose from {SHAPES}")

def resample(points, step):
    """Points along the polyline every `step` rad (constant speed), with the indices of the original corners."""
    out, corners = [points[0]], [0]
    for a, b in zip(points[:-1], points[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
        corners.append(len(out) - 1)
    return np.array(out), corners

def terminate_current(emio_motors, ignore_speed_limit = False):
    global robot
    global commanded
    global TRAJECTORY_IS_ACTIVE
    global CURRENT_LOOP
    global CURRENT_LOOP_I

    if commanded is None:
        commanded = robot.angles(emio_motors)
    if ignore_speed_limit:
        robot.set_angles(get_common_zero(), emio_motors)
    else:
        rampTo(robot, get_common_zero(), emio_motors, start=commanded, maxCurrent=np.inf)
    CURRENT_LOOP = 0
    CURRENT_LOOP_I = 0
    print("Trajectory completed / terminated.")
    reset_bend()
    TRAJECTORY_IS_ACTIVE = False

def update(emio_motors):
    global robot
    if not TRAJECTORY_IS_ACTIVE:
        return

    global CURRENT_LOOP
    global CURRENT_LOOP_I
    global points
    global loops
    if CURRENT_LOOP == loops:
        terminate_current(emio_motors)
        return

    (x,z) = points[CURRENT_LOOP_I]
    commanded = anglesFromBend({"x": x, "z": z})
    robot.set_angles(commanded, emio_motors)
    currents = robot.currents(emio_motors)
    if np.any(currents > MAX_CURRENT):
        print(f"\nMotor current {np.round(currents).tolist()} mA above {MAX_CURRENT:.0f} mA: stopping")
        terminate_current(emio_motors)
        return
    if CURRENT_LOOP_I % 5 == 0:
        print(f"\r loop {CURRENT_LOOP + 1}/{CURRENT_LOOP} {100 * CURRENT_LOOP_I / (len(points) - 1):3.0f}% | bend x {x:+.2f}  "
                f"z {z:+.2f} rad | currents {' '.join(f'{c:4.0f}' for c in currents)} mA   ",
                end="", flush=True)

    CURRENT_LOOP_I = CURRENT_LOOP_I + 1
    if CURRENT_LOOP_I >= len(points):
        CURRENT_LOOP_I = 0
        CURRENT_LOOP = CURRENT_LOOP + 1


def main(args_amplitude, args_loops, args_shape, emio_motors):
    global robot
    global commanded
    global TRAJECTORY_IS_ACTIVE
    global loops
    global points
    global CURRENT_LOOP
    global CURRENT_LOOP_I
    loops = args_loops

    args_pause = 0
    args_max_theta = MAX_THETA
    args_speed = SAVED[args_shape]["speed"] if args_shape in SAVED else SPEED

    path = waypoints(args_shape, args_amplitude)

    if np.max(np.abs(path)) > args_max_theta:
        print(f"the path needs {np.max(np.abs(path)):.2f} rad on a motor, above the limit "
                         f"{args_max_theta:g} rad (--max-theta)")
        terminate_current(emio_motors)
        return

    points, corners = resample(path, args_speed * LOOP_PERIOD)
    holdAt = set(corners) if args_pause > 0 and len(path) < CURVE_POINTS else set()
    loopTime = len(points) * LOOP_PERIOD + len(holdAt) * args_pause

    print(f"{args_shape}: {len(path)} waypoints, up to {np.max(np.abs(path)):.2f} rad, {args_speed:g} rad/s, "
          f"{args_loops} loop(s) of about {loopTime:.0f} s. Press Q (or Ctrl+C) to stop.")

    commanded = robot.angles(emio_motors)
    commanded = rampTo(robot, np.zeros(4), emio_motors, start=commanded, maxCurrent=np.inf)
    print("Moving to the start of the path...")
    commanded = rampTo(robot, anglesFromBend({"x": points[0][0], "z": points[0][1]}), emio_motors, start=commanded,
                        maxStep=args_speed * LOOP_PERIOD, period=LOOP_PERIOD, maxCurrent=MAX_CURRENT)
    TRAJECTORY_IS_ACTIVE = True # Locks motor control away from keyboard
    CURRENT_LOOP = 0
    CURRENT_LOOP_I = 0
