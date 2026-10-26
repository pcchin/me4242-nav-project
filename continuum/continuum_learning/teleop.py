"""
Steer the real continuum robot by eye with the keyboard, from this terminal: no camera, no model, no SOFA.

Hold (while this terminal has the focus):
    W / S   bend toward +x / -x   (motor 1 winds and motor 3 releases, or the reverse)
    D / A   bend toward +z / -z   (motor 0 winds and motor 2 releases, or the reverse)
    R       back to straight (all motors at 0)
    Q       quit (back to straight, motors released); Ctrl+C does the same
Opposite cables always work in pairs (one winds, the other releases), so they never fight. Each motor stays within
+-2.5 rad (--max-theta). A move that makes a motor draw more than 1200 mA (--max-current) is undone.

Usage (from this folder, in the emio-ml environment; close Emio Labs / SOFA first, they'd hold the port):
    python teleop.py --port COM12
    python teleop.py --speed 0.3 --max-theta 1.0  # slower, smaller range
    python teleop.py --simulate --script "W*30,D*60,R,Q"   # no hardware, scripted keys (test)
"""
import argparse
import sys
import time

import numpy as np

import config
from robot_io import EmioMotorsOnly, FakeRobot, OverCurrentError, rampTo

PAIRS = {"x": (1, 3), "z": (0, 2)}  # (motor winding toward +, motor winding toward -)
KEYS = {"W": ("x", 1.), "S": ("x", -1.), "D": ("z", 1.), "A": ("z", -1.)}
KEY_RATE = 30.  # key-repeat events per second while a key is held (Windows default, about 30)
LOOP_PERIOD = 0.02  # s
RELEASED_AFTER = 0.6  # s without key events: the keys are released (longer than the first key-repeat delay)
SPEED = 0.5  # rad/s, bending speed while a key is held
MAX_THETA = 2.5  # rad, angle limit of each motor for steering by eye (data collection keeps config.MAX_THETA)
MAX_CURRENT = 1200.  # mA, a move above is undone (XM430-W210 stalls around 1300 mA; data collection keeps 600 mA)


def bendFromAngles(theta):
    """Bend toward +x and +z (rad) from the 4 motor angles."""
    return {axis: (theta[plus] - theta[minus]) / 2. for axis, (plus, minus) in PAIRS.items()}


def anglesFromBend(bend):
    """The 4 motor angles for a bend: each pair winds one cable and releases the other by the same angle."""
    theta = np.zeros(4)
    for axis, (plus, minus) in PAIRS.items():
        theta[plus], theta[minus] = bend[axis], -bend[axis]
    return theta


def applyKey(bend, key, step, maxTheta):
    """The bend after one W/S/A/D key event (unchanged for other keys), clamped to +-maxTheta."""
    bend = dict(bend)
    if key in KEYS:
        axis, direction = KEYS[key]
        bend[axis] = float(np.clip(bend[axis] + direction * step, -maxTheta, maxTheta))
    return bend


class TerminalKeys:
    """Key presses in this terminal (Windows), only while it has the focus. Held keys repeat (about 30/s)."""

    def __init__(self):
        import msvcrt
        self.msvcrt = msvcrt

    def read(self):
        keys = []
        while self.msvcrt.kbhit():
            ch = self.msvcrt.getwch()
            if ch in ("\x00", "\xe0"):  # special key (arrows...): skip its second code
                self.msvcrt.getwch()
                continue
            keys.append(ch.upper())
        return keys


class ScriptKeys:
    """Scripted keys for testing: "W*30,D*60,R,Q" = W for 30 loops, D for 60 loops, then R, then Q."""

    def __init__(self, script):
        self.queue = []
        for item in script.split(","):
            key, _, count = item.strip().partition("*")
            self.queue += [[key.upper()]] * (int(count) if count else 1)

    def read(self):
        return self.queue.pop(0) if self.queue else ["Q"]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="serial port of the robot, e.g. COM12 (default: first found)")
    parser.add_argument("--speed", type=float, default=SPEED, help="bending speed while a key is held (rad/s)")
    parser.add_argument("--max-theta", type=float, default=MAX_THETA, help="angle limit of each motor (rad)")
    parser.add_argument("--max-current", type=float, default=MAX_CURRENT, help="mA, a move above is undone")
    parser.add_argument("--simulate", action="store_true", help="made-up robot instead of the hardware")
    parser.add_argument("--script", default=None, help='scripted keys instead of the keyboard, e.g. "W*30,R,Q"')
    args = parser.parse_args()

    keys = ScriptKeys(args.script) if args.script else TerminalKeys()
    robot = FakeRobot() if args.simulate else EmioMotorsOnly(args.port)
    maxTheta, step = args.max_theta, args.speed / KEY_RATE
    print(__doc__.split("Usage")[0].strip())
    print(f"\nConnected to the {'made-up' if args.simulate else 'real'} robot (limit +-{maxTheta:g} rad, "
          f"{args.speed:g} rad/s, current limit {args.max_current:g} mA). Straightening first...")

    commanded = robot.angles
    try:
        commanded = rampTo(robot, np.zeros(4), start=commanded, maxCurrent=np.inf)
        bend = {"x": 0., "z": 0.}
        blocked, lastKeyTime, lastPrint = set(), 0., 0.
        while True:
            pressed = keys.read()
            now = time.time()
            if pressed:
                lastKeyTime = now
            elif now - lastKeyTime > RELEASED_AFTER:
                blocked.clear()  # the keys have been released

            if "Q" in pressed:
                break
            if "R" in pressed:
                try:
                    commanded = rampTo(robot, np.zeros(4), start=commanded, maxCurrent=args.max_current)
                    bend = {"x": 0., "z": 0.}
                except OverCurrentError as e:
                    print(f"\n{e}: straightening stopped")
                    commanded = robot.angles
                    bend = bendFromAngles(commanded)
                continue

            newBend = bend
            for key in pressed:
                if key not in blocked:
                    newBend = applyKey(newBend, key, step, maxTheta)
            if newBend != bend:
                target = anglesFromBend(newBend)
                robot.angles = target
                currents = robot.currents
                if np.any(currents > args.max_current):
                    robot.angles = commanded  # undo the step
                    blocked.update(k for k in pressed if k in KEYS)
                    print(f"\nMotor current {np.round(currents).tolist()} mA above {args.max_current:.0f} mA: "
                          f"step undone, release the key")
                else:
                    commanded, bend = target, newBend

            if now - lastPrint > 0.1 or args.script:
                lastPrint = now
                print(f"\r bend x {bend['x']:+.2f}  z {bend['z']:+.2f} rad | angles "
                      f"{' '.join(f'{a:+.2f}' for a in commanded)} | currents "
                      f"{' '.join(f'{c:4.0f}' for c in robot.currents)} mA   ", end="", flush=True)
            if not args.simulate:
                time.sleep(LOOP_PERIOD)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStraightening and releasing the motors...")
        try:
            rampTo(robot, np.zeros(4), start=commanded, maxCurrent=np.inf)
        finally:
            robot.close()
        print("Done.")


if __name__ == "__main__":
    main()
