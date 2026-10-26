"""
Settings shared by the data collection, the training, the IK and the SOFA scene.
Check them on your robot before collecting data.

Conventions:
    - Motor angles `theta` (rad), 4 values, sent to the motors as is (0 rad = cables taut, the mounting position).
    - Wound length of cable i (mm): `MOTOR_DIRECTIONS[i] * PULLEY_RADIUS * theta[i]`, positive when the cable is wound.
    - Positions (mm) in the scene frame of Emio: origin at the center between the motors, Y up.
      Cable 0 faces +Z, cable 1 faces +X, cable 2 faces -Z, cable 3 faces -X.
"""
import numpy as np

PULLEY_RADIUS = 20.  # mm, radius of the spools (40 mm diameter)
MOTOR_DIRECTIONS = np.array([1., 1., 1., 1.])  # sign of the motor rotation that winds each cable (all wind when positive)

MAX_PULL = 20.  # mm, maximum wound/released length of each cable (1 rad on the 20 mm radius spool)
MAX_THETA = MAX_PULL / PULLEY_RADIUS  # rad, angle limit of each motor
MAX_COCONTRACTION = 2.  # mm, maximum length wound on both cables of an opposite pair
OPPOSITE_PAIRS = [(0, 2), (1, 3)]

MAX_CURRENT = 600.  # mA, a motor above this current aborts the move (XM430-W210 stalls around 1300 mA)
MAX_STEP = 0.02  # rad, angle change per motion step
STEP_PERIOD = 0.02  # s, time between motion steps (MAX_STEP / STEP_PERIOD = 1 rad/s)

SETTLE_TIMEOUT = 5.  # s, maximum waiting time for the tip to settle (large bends take longer)
SETTLE_TOLERANCE = 0.4  # mm, the tip is settled when it moves less than this between two readings
N_FRAMES = 10  # camera readings averaged per sample
MAX_TIP_STD = 1.5  # mm, samples with a noisier camera reading are discarded at training

CONFIGURATION = "extended"  # Emio configuration, used by the camera

# Tip marker tracking (the camera may see other blobs of the same colour)
WORKSPACE_RADIUS = 170.  # mm, detections further from the robot's axis are ignored
WORKSPACE_Y = (-240., -20.)  # mm, height range of the tip
TIP_SEARCH_RADIUS = 50.  # mm, at start-up (robot at rest) the tip marker must be this close to the axis
BACKGROUND_RADIUS = 15.  # mm, detections this close to a static background blob are ignored
MAX_JUMP = 80.  # mm, a detection further than this from the last tip position is not the tip
PREDICTION_GATE = 40.  # mm, when the tip position is predicted (calibrated physics), detections further are not the tip
# Stuck tracker: the measured tip barely moves while the predicted tip moves a lot, over several samples
STUCK_SAMPLES = 5
STUCK_MEASURED_SPREAD = 2.  # mm
STUCK_PREDICTED_SPREAD = 15.  # mm


def windLengths(theta, directions=MOTOR_DIRECTIONS, pulleyRadius=PULLEY_RADIUS):
    """
    Wound length of each cable (mm), positive when wound and negative when released. Works on (4,) or (n, 4) arrays.
    """
    return np.asarray(theta) * directions * pulleyRadius


def cocontraction(theta, directions=MOTOR_DIRECTIONS, pulleyRadius=PULLEY_RADIUS, pairs=OPPOSITE_PAIRS):
    """
    Length wound on both cables of each opposite pair (mm), shape (..., nb pairs). Zero when at most one is wound.
    """
    w = windLengths(theta, directions, pulleyRadius)
    return np.stack([np.clip(np.minimum(w[..., i], w[..., j]), 0., None) for i, j in pairs], axis=-1)


def isSafe(theta, maxTheta=MAX_THETA, maxCocontraction=MAX_COCONTRACTION):
    """
    True when all angles are within the limits and no opposite pair is co-contracted beyond the limit.
    Works on (4,) or (n, 4) arrays.
    """
    theta = np.asarray(theta)
    inRange = np.all(np.abs(theta) <= maxTheta + 1e-9, axis=-1)
    return inRange & np.all(cocontraction(theta) <= maxCocontraction + 1e-9, axis=-1)
