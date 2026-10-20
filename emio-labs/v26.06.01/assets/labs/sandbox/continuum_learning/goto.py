"""
Move the tip of the real robot to a position (mm, scene frame of Emio) with the learned inverse kinematics,
then measure where it actually went with the camera.

Usage (from this folder, in the emio-ml environment; close Emio Labs / SOFA first):
    python goto.py 40 -180 0          # x y z
    python goto.py 40 nan 0           # leave y free
    python goto.py 40 -180 0 --dry-run    # only print the angles
    python goto.py 40 -180 0 --simulate   # made-up robot
"""
import argparse
import os

import numpy as np

import config
import ik
from robot_io import EmioRobot, FakeRobot, rampTo


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float, help="use nan to leave y free")
    parser.add_argument("z", type=float)
    parser.add_argument("--model", default=os.path.join(here, "models", "model.npz"))
    parser.add_argument("--dry-run", action="store_true", help="compute the angles without moving the robot")
    parser.add_argument("--simulate", action="store_true", help="use a made-up robot instead of the hardware")
    parser.add_argument("--stay", action="store_true", help="stay at the position (default: return to rest)")
    args = parser.parse_args()

    model = ik.loadModel(args.model)
    target = np.array([args.x, args.y, args.z])
    result = ik.solveIK(target, model)
    theta = result["theta"]
    print(f"Target    {target}  (y {'controlled' if model['y_controllable'] and not np.isnan(args.y) else 'free'})")
    print(f"Angles    {np.round(theta, 4)} rad   wound {np.round(config.windLengths(theta), 2)} mm")
    print(f"Predicted {np.round(result['predicted'], 1)}  error {result['error']:.2f} mm"
          + ("" if result["reachable"] else "  -> OUT OF REACH, going to the closest position"))
    if args.dry_run:
        return

    robot = FakeRobot() if args.simulate else EmioRobot()
    try:
        rampTo(robot, theta)
        if hasattr(robot, "_lastTip"):
            robot._lastTip = result["predicted"]  # look for the marker where the model expects the tip
        robot.waitSettled()
        tip, std, nFrames = robot.readTip()
        if tip is None:
            print("Measured  no marker seen by the camera")
        else:
            mask = model["weights"] > 0
            mask[np.isnan(target)] = False
            error = np.linalg.norm(mask * (tip - np.nan_to_num(target)))
            modelError = np.linalg.norm(mask * (tip - result["predicted"]))
            print(f"Measured  {np.round(tip, 1)}  error {error:.2f} mm to the target, {modelError:.2f} mm to the "
                  f"prediction  (camera std {np.round(std, 2)} mm)")
        if args.stay:
            input("At the target. Press Enter to go back to rest and release the motors...")
    finally:
        try:
            rampTo(robot, np.zeros(4), maxCurrent=np.inf)
        finally:
            robot.close()


if __name__ == "__main__":
    main()
