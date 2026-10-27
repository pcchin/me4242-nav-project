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
    custom    your own points: --waypoints "1,0; 0,1; -1,0; 0,-1" (x,z pairs in rad, not scaled)
Saved trajectories (SAVED below, fixed points and speed, --amplitude ignored):
    trajectory1   5 points from the user, 0.6 rad/s, ends straight
    trajectory2   bend toward -x (motor 1 at -2.27 rad) and back, 0.6 rad/s
    trajectory3   10 points from the user, up to 2.57 rad, 0.6 rad/s, ends straight
    (add your own to SAVED as motor angles)

Usage (from this folder, in the emio-ml environment; close Emio Labs / SOFA first, they'd hold the port):
    python trajectory.py circle --port COM12
    python trajectory.py square --amplitude 1.5 --loops 3 --pause 1
    python trajectory.py custom --waypoints "1,0; 1,1; 0,1" --speed 0.3
    python trajectory.py figure8 --simulate                 # no hardware (test)
    python trajectory.py trajectory1 --port COM3
"""
import argparse
import time

import numpy as np

from robot_io import EmioMotorsOnly, FakeRobot, rampTo
from teleop import LOOP_PERIOD, MAX_CURRENT, MAX_THETA, SINGLE_TURN_LIMIT, SPEED, TerminalKeys, anglesFromBend, bendFromAngles

# Saved trajectories: waypoints as the 4 motor angles (rad, motors 0 1 2 3, as teleop.py shows them) and speed (rad/s).
# Opposite motors must be opposite (motor 2 = -motor 0, motor 3 = -motor 1). To add one, copy a block and rename it.
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
SHAPES = ["circle", "square", "figure8", "cross", "custom"] + list(SAVED)
CURVE_POINTS = 200  # points per loop of the smooth shapes


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


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("shape", choices=SHAPES)
    parser.add_argument("--amplitude", type=float, default=1.0, help="size of the shape (rad of bend)")
    parser.add_argument("--waypoints", default=None, help='custom points "x,z; x,z; ..." (rad)')
    parser.add_argument("--loops", type=int, default=1, help="number of times the path is run")
    parser.add_argument("--speed", type=float, default=None,
                        help=f"speed along the path (rad/s; default: the saved trajectory's, else {SPEED:g})")
    parser.add_argument("--pause", type=float, default=0., help="s to hold at each waypoint (square, cross, custom)")
    parser.add_argument("--port", default=None, help="serial port of the robot, e.g. COM12 (default: first found)")
    parser.add_argument("--max-theta", type=float, default=MAX_THETA, help="angle limit of each motor (rad)")
    parser.add_argument("--max-current", type=float, default=MAX_CURRENT,
                        help="mA, stop when a motor goes above (default: no limit)")
    parser.add_argument("--simulate", action="store_true", help="made-up robot instead of the hardware")
    args = parser.parse_args()

    if args.speed is None:
        args.speed = SAVED[args.shape]["speed"] if args.shape in SAVED else SPEED
    path = waypoints(args.shape, args.amplitude, args.waypoints)
    if np.max(np.abs(path)) > args.max_theta:
        raise SystemExit(f"the path needs {np.max(np.abs(path)):.2f} rad on a motor, above the limit "
                         f"{args.max_theta:g} rad (--max-theta)")
    points, corners = resample(path, args.speed * LOOP_PERIOD)
    holdAt = set(corners) if args.pause > 0 and len(path) < CURVE_POINTS else set()
    loopTime = len(points) * LOOP_PERIOD + len(holdAt) * args.pause

    multiTurn = args.max_theta > SINGLE_TURN_LIMIT
    robot = FakeRobot() if args.simulate else EmioMotorsOnly(args.port, multiTurn=multiTurn)
    keys = TerminalKeys()
    print(f"{args.shape}: {len(path)} waypoints, up to {np.max(np.abs(path)):.2f} rad, {args.speed:g} rad/s, "
          f"{args.loops} loop(s) of about {loopTime:.0f} s. Press Q (or Ctrl+C) to stop.")

    commanded = robot.angles
    try:
        commanded = rampTo(robot, np.zeros(4), start=commanded, maxCurrent=np.inf)
        print("Moving to the start of the path...")
        commanded = rampTo(robot, anglesFromBend({"x": points[0][0], "z": points[0][1]}), start=commanded,
                           maxStep=args.speed * LOOP_PERIOD, period=LOOP_PERIOD, maxCurrent=args.max_current)
        stop = False
        for loop in range(args.loops):
            for i, (x, z) in enumerate(points):
                if "Q" in keys.read():
                    stop = True
                    break
                commanded = anglesFromBend({"x": x, "z": z})
                robot.angles = commanded
                currents = robot.currents
                if np.any(currents > args.max_current):
                    print(f"\nMotor current {np.round(currents).tolist()} mA above {args.max_current:.0f} mA: stopping")
                    stop = True
                    break
                if i % 5 == 0:
                    print(f"\r loop {loop + 1}/{args.loops} {100 * i / (len(points) - 1):3.0f}% | bend x {x:+.2f}  "
                          f"z {z:+.2f} rad | currents {' '.join(f'{c:4.0f}' for c in currents)} mA   ",
                          end="", flush=True)
                if not args.simulate:
                    time.sleep(LOOP_PERIOD)
                    if i in holdAt:
                        time.sleep(args.pause)
            if stop:
                break
        print("\nTrajectory " + ("stopped." if stop else "done."))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        print("Straightening and releasing the motors...")
        try:
            rampTo(robot, np.zeros(4), start=commanded, maxCurrent=np.inf)
        finally:
            robot.close()
        print("Done.")


if __name__ == "__main__":
    main()
