"""
Steer the real continuum robot by eye with the keyboard, from this terminal: no camera, no model, no SOFA.

Hold (while this terminal has the focus):
    W / S   bend toward +x / -x   (motor 1 winds and motor 3 releases, or the reverse)
    D / A   bend toward +z / -z   (motor 0 winds and motor 2 releases, or the reverse)
    1 / 2   motor 1 winds / releases (alone)      3 / 4   motor 2      5 / 6   motor 3      7 / 8   motor 4
            (motors numbered 1-4 here, 0-3 in the code and the angles display, which lists them in order)
    R       back to straight (all motors at 0)
    Q       quit (back to straight, motors released); Ctrl+C does the same
With W/A/S/D, opposite cables work in pairs (one winds, the other releases), so they never fight; with 1-8 a single
motor moves, so watch the opposite cable (it can go slack, or fight if both are wound). Each motor stays within
+-4 rad (--max-theta; beyond +-pi the motors are switched to multi-turn mode). No motor current limit by default (--max-current to set one); currents are displayed.

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
MOTOR_KEYS = {"1": (0, 1.), "2": (0, -1.), "3": (1, 1.), "4": (1, -1.),  # (motor index, wind + / release -)
              "5": (2, 1.), "6": (2, -1.), "7": (3, 1.), "8": (3, -1.)}
KEY_RATE = 30.  # key-repeat events per second while a key is held (Windows default, about 30)
LOOP_PERIOD = 0.02  # s
RELEASED_AFTER = 0.6  # s without key events: the keys are released (longer than the first key-repeat delay)
SPEED = 1.0  # rad/s, bending speed while a key is held
MAX_THETA = 4.0  # rad, angle limit of each motor for steering by eye (data collection keeps config.MAX_THETA)
SINGLE_TURN_LIMIT = 3.1  # rad: beyond about +-pi, the motors need their extended (multi-turn) position mode
MAX_CURRENT = float("inf")  # mA, no limit by default (user's choice); --max-current sets one. The motors still protect
# themselves (overload error, torque off at 53 degC)


def bendFromAngles(theta):
    """Bend toward +x and +z (rad) from the 4 motor angles."""
    return {axis: (theta[plus] - theta[minus]) / 2. for axis, (plus, minus) in PAIRS.items()}


def anglesFromBend(bend):
    """The 4 motor angles for a bend: each pair winds one cable and releases the other by the same angle."""
    theta = np.zeros(4)
    for axis, (plus, minus) in PAIRS.items():
        theta[plus], theta[minus] = bend[axis], -bend[axis]
    return theta


def applyKey(theta, key, step, maxTheta):
    """
    The 4 motor angles after one key event, clamped to +-maxTheta (unchanged for other keys): W/S/A/D move an opposite
    pair (one winds and the other releases by `step`), 1-8 a single motor.
    """
    theta = np.array(theta, dtype=float)
    if key in KEYS:
        axis, direction = KEYS[key]
        plus, minus = PAIRS[axis]
        theta[plus] += direction * step
        theta[minus] -= direction * step
    elif key in MOTOR_KEYS:
        motor, direction = MOTOR_KEYS[key]
        theta[motor] += direction * step
    return np.clip(theta, -maxTheta, maxTheta)


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


def steer(robot, keys, commanded, maxTheta, step, maxCurrent, printAll=False, sleep=True):
    """
    The W/A/S/D steering loop, starting from the `commanded` angles (the robot already there), until Q or Ctrl+C.
    Returns the last commanded angles (the caller straightens and releases). Also used by trajectory.py.
    """
    commanded = np.array(commanded, dtype=float)
    blocked, lastKeyTime, lastPrint = set(), 0., 0.
    try:
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
                    commanded = rampTo(robot, np.zeros(4), start=commanded, maxCurrent=maxCurrent)
                except OverCurrentError as e:
                    print(f"\n{e}: straightening stopped")
                    commanded = robot.angles
                continue

            target = commanded
            for key in pressed:
                if key not in blocked:
                    target = applyKey(target, key, step, maxTheta)
            if not np.array_equal(target, commanded):
                robot.angles = target
                currents = robot.currents
                if np.any(currents > maxCurrent):
                    robot.angles = commanded  # undo the step
                    blocked.update(k for k in pressed if k in KEYS or k in MOTOR_KEYS)
                    print(f"\nMotor current {np.round(currents).tolist()} mA above {maxCurrent:.0f} mA: "
                          f"step undone, release the key")
                else:
                    commanded = target

            if now - lastPrint > 0.1 or printAll:
                lastPrint = now
                bend = bendFromAngles(commanded)
                print(f"\r bend x {bend['x']:+.2f}  z {bend['z']:+.2f} rad | angles "
                      f"{' '.join(f'{a:+.2f}' for a in commanded)} | currents "
                      f"{' '.join(f'{c:4.0f}' for c in robot.currents)} mA   ", end="", flush=True)
            if sleep:
                time.sleep(LOOP_PERIOD)
    except KeyboardInterrupt:
        pass
    return commanded


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="serial port of the robot, e.g. COM12 (default: first found)")
    parser.add_argument("--speed", type=float, default=SPEED, help="bending speed while a key is held (rad/s)")
    parser.add_argument("--max-theta", type=float, default=MAX_THETA, help="angle limit of each motor (rad)")
    parser.add_argument("--max-current", type=float, default=MAX_CURRENT,
                        help="mA, a move above is undone (default: no limit)")
    parser.add_argument("--simulate", action="store_true", help="made-up robot instead of the hardware")
    parser.add_argument("--script", default=None, help='scripted keys instead of the keyboard, e.g. "W*30,R,Q"')
    args = parser.parse_args()

    keys = ScriptKeys(args.script) if args.script else TerminalKeys()
    multiTurn = args.max_theta > SINGLE_TURN_LIMIT
    robot = FakeRobot() if args.simulate else EmioMotorsOnly(args.port, multiTurn=multiTurn)
    maxTheta, step = args.max_theta, args.speed / KEY_RATE
    print(__doc__.split("Usage")[0].strip())
    print(f"\nConnected to the {'made-up' if args.simulate else 'real'} robot (limit +-{maxTheta:g} rad, "
          f"{args.speed:g} rad/s, " + ("multi-turn mode, " if multiTurn else "") + ("no current limit" if args.max_current == float("inf") else
                                       f"current limit {args.max_current:g} mA") + "). Straightening first...")

    commanded = robot.angles
    try:
        commanded = rampTo(robot, np.zeros(4), start=commanded, maxCurrent=np.inf)
        commanded = steer(robot, keys, commanded, maxTheta, step, args.max_current, printAll=bool(args.script),
                          sleep=not args.simulate)
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
