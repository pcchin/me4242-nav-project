"""
Steer the real continuum robot by eye with the keyboard, from this terminal: no camera, no model, no SOFA.

Hold (while this terminal has the focus):
    W / S   bend toward +x / -x   (motor 1 winds and motor 3 releases, or the reverse)
    D / A   bend toward +z / -z   (motor 0 winds and motor 2 releases, or the reverse)
    R       back to straight (all motors at 0)
    Q       quit (back to straight, motors released); Ctrl+C does the same
Opposite cables always work in pairs (one winds, the other releases), so they never fight. Each motor stays within
+-4 rad (--max-theta; beyond +-pi the motors are switched to multi-turn mode). No motor current limit by default (--max-current to set one); currents are displayed.

This variant is meant to be used as a library to main.py and should not be ran independently.
"""
import argparse
import sys
import time

import numpy as np

import config
from robot_io import EmioMotorsOnly, FakeRobot, OverCurrentError, rampTo

PAIRS = {"x": (1, 3), "z": (0, 2)}  # (motor winding toward +, motor winding toward -)
KEYS = {"w": ("x", 1.), "s": ("x", -1.), "d": ("z", 1.), "a": ("z", -1.)} #W,S,D,A
KEY_RATE = 30.  # key-repeat events per second while a key is held (Windows default, about 30)
LOOP_PERIOD = 0.02  # s
RELEASED_AFTER = 0.6  # s without key events: the keys are released (longer than the first key-repeat delay)
SPEED = 1.0  # rad/s, bending speed while a key is held
MAX_THETA = 4.0  # rad, angle limit of each motor for steering by eye (data collection keeps config.MAX_THETA)
SINGLE_TURN_LIMIT = 3.1  # rad: beyond about +-pi, the motors need their extended (multi-turn) position mode
MAX_CURRENT = float("inf")  # mA, no limit by default (user's choice); --max-current sets one. The motors still protect
# themselves (overload error, torque off at 53 degC)
COMMON_ZERO = np.zeros(4)

robot = None
commanded = None
bend = None

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

def get_motor_coords():
    global commanded
    return commanded

def set_joint_angles(pos, emio_motors):
    commanded = rampTo(robot, pos, emio_motors, start=commanded, maxCurrent=MAX_CURRENT)
    reset_bend()

def reset_bend():
    global bend
    bend = {"x": 0., "z": 0.}

def on_key_pressed(keys_pressed, emio_motors):
    global commanded
    global bend

    if "r" in keys_pressed:
        try:
            commanded = rampTo(robot, COMMON_ZERO, emio_motors, start=commanded, maxCurrent=MAX_CURRENT)
            reset_bend()
        except OverCurrentError as e:
            print(f"\n{e}: straightening stopped")
            commanded = robot.angles(emio_motors)
            bend = bendFromAngles(commanded)
        return
    if "q" in keys_pressed:
        # Does nothing, holdover
        return
    newBend = bend
    for key in keys_pressed:
        newBend = applyKey(newBend, key, step, maxTheta)
    if newBend != bend:
        target = anglesFromBend(newBend)
        robot.set_angles(target, emio_motors)
        currents = robot.currents(emio_motors)
        if np.any(currents > MAX_CURRENT):
            robot.set_angles(commanded, emio_motors)  # undo the step
            print(f"\nMotor current {np.round(currents).tolist()} mA above {MAX_CURRENT:.0f} mA: "
                    f"step undone")
        else:
            commanded, bend = target, newBend
    print(f"\r bend x {bend['x']:+.2f}  z {bend['z']:+.2f} rad | angles "
                    f"{' '.join(f'{a:+.2f}' for a in commanded)} | currents "
                    f"{' '.join(f'{c:4.0f}' for c in robot.currents(emio_motors))} mA   ", end="", flush=True)

def shutdown(emio_motors):
    global robot
    global commanded
    print("\nStraightening and releasing the motors...")
    try:
        rampTo(robot, COMMON_ZERO, emio_motors, start=commanded, maxCurrent=np.inf)
    finally:
        # Handled in scripts
        # robot.close()
        pass
    print("Done.")


def init_motors(emio_motors):
    global robot
    global maxTheta
    global step
    global commanded
    global bend
    multiTurn = MAX_THETA > SINGLE_TURN_LIMIT
    robot = EmioMotorsOnly()
    maxTheta, step = MAX_THETA, SPEED / KEY_RATE
    print(__doc__.split("Usage")[0].strip())
    print(f"\nConnected to the real robot (limit +-{maxTheta:g} rad, "
          f"{SPEED:g} rad/s, " + ("multi-turn mode, " if multiTurn else "") + ("no current limit" if MAX_CURRENT == float("inf") else
                                   f"current limit {MAX_CURRENT:g} mA") + "). Straightening first...")

    commanded = robot.angles(emio_motors)
    commanded = rampTo(robot, COMMON_ZERO, emio_motors, start=commanded, maxCurrent=np.inf)
    reset_bend()

