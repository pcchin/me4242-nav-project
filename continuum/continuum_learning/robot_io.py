"""
Access to the robot for the data collection and `goto.py`:
    - `EmioRobot`: the real Emio, motors with `emioapi.EmioMotors` and tip position with the depth camera
      (`emioapi.emiocamera.EmioCamera`, positions in the scene frame of Emio, in mm, once the camera is calibrated).
    - `FakeRobot`: a made-up 4-cable constant-curvature robot, to test the scripts without hardware.
    - `rampTo()`: safe motion, small angle steps with a motor current check.
Both robots have the same interface: `angles` (get/set, rad), `currents` (mA), `readTip()`, `close()`.
"""
import time
import numpy as np

import config


class OverCurrentError(RuntimeError):
    pass


def pickTip(candidates, background, lastTip, prediction=None, reach=None):
    """
    Which detection (n, 3) is the tip marker, or None:
        - detections farther from the robot's base than it can reach (`reach` = (base, radius)) are ignored,
        - detections close to a static background blob are ignored,
        - if the tip is predicted, only detections within PREDICTION_GATE of the prediction are kept,
        - the tip is the detection closest to its last position, if within MAX_JUMP.
    """
    candidates = np.asarray(candidates, dtype=float).reshape(-1, 3)
    if reach is not None and len(candidates):
        candidates = candidates[np.linalg.norm(candidates - reach[0], axis=1) <= reach[1]]
    if len(background) and len(candidates):
        distances = np.linalg.norm(candidates[:, None, :] - np.asarray(background)[None, :, :], axis=2)
        candidates = candidates[distances.min(axis=1) > config.BACKGROUND_RADIUS]
    if prediction is not None and len(candidates):
        candidates = candidates[np.linalg.norm(candidates - prediction, axis=1) <= config.PREDICTION_GATE]
    if not len(candidates):
        return None
    distances = np.linalg.norm(candidates - lastTip, axis=1)
    nearest = int(np.argmin(distances))
    return candidates[nearest].copy() if distances[nearest] <= config.MAX_JUMP else None


class FakeRobot:
    """
    Made-up continuum robot, hanging along -Y, 195 mm long, cables 6 mm off the axis.
    Cables only pull: the bending comes from the difference of wound lengths in each opposite pair, and the length
    wound on both cables of a pair (co-contraction) shortens the backbone a little. Camera noise of 0.3 mm.
    """

    LENGTH = 195.
    OFFSET = 6.
    SHORTENING = 0.3  # mm of backbone shortening per mm of co-contraction
    NOISE = 0.3  # mm

    def __init__(self, seed=0, hideBeyond=None, strayBlob=None):
        """
        `hideBeyond` (rad): the camera loses the tip when the bend is larger (marker turned away).
        `strayBlob` ((3,) mm): a stray detection always in view. Together they reproduce the tracker locking onto a
        stray blob, to test the tracking.
        """
        self._angles = np.zeros(4)
        self._rng = np.random.default_rng(seed)
        self.hideBeyond = hideBeyond
        self.strayBlob = None if strayBlob is None else np.asarray(strayBlob, dtype=float)
        self._lastTip = self.tipFromAngles(np.zeros(4))[0]
        self._background = np.zeros((0, 3))
        self._prediction = None
        self._reach = None

    def setReach(self, base, radius):
        """Ignore detections farther than `radius` mm from the robot's base `base` (it can't reach there)."""
        self._reach = (np.asarray(base, dtype=float), float(radius))

    def setPrediction(self, position):
        self._prediction = None if position is None else np.asarray(position, dtype=float)
        if self._prediction is not None:
            self._lastTip = self._prediction.copy()

    def refreshTracking(self, duration=0.):
        self._lastTip = self.tipFromAngles(np.zeros(4))[0]

    def _bend(self):
        w = config.windLengths(self._angles)
        return np.hypot((w[0] - w[2]) / 2., (w[1] - w[3]) / 2.) / self.OFFSET

    @staticmethod
    def tipFromAngles(theta):
        theta = np.atleast_2d(theta)
        w = config.windLengths(theta)
        bz = (w[:, 0] - w[:, 2]) / 2.  # bending toward cable 0 (+Z)
        bx = (w[:, 1] - w[:, 3]) / 2.  # bending toward cable 1 (+X)
        c = config.cocontraction(theta).sum(axis=1)
        length = FakeRobot.LENGTH - FakeRobot.SHORTENING * c
        phi = np.hypot(bx, bz) / FakeRobot.OFFSET  # bending angle (rad)
        psi = np.arctan2(bx, bz)  # bending direction around Y
        safe = np.where(phi < 1e-9, 1., phi)
        rho = np.where(phi < 1e-9, 0., length / safe * (1. - np.cos(phi)))
        y = np.where(phi < 1e-9, -length, -length / safe * np.sin(phi))
        return np.stack([rho * np.sin(psi), y, rho * np.cos(psi)], axis=1)

    @property
    def angles(self):
        return self._angles.copy()

    @angles.setter
    def angles(self, value):
        self._angles = np.asarray(value, dtype=float).copy()

    @property
    def currents(self):
        w = np.abs(config.windLengths(self._angles))
        c = np.repeat(config.cocontraction(self._angles), 2)[[0, 2, 1, 3]]
        return 50. + 10. * w + 100. * c

    def readTip(self, nFrames=config.N_FRAMES):
        tip = self.tipFromAngles(self._angles)[0]
        frames = []
        for _ in range(nFrames):
            detections = []
            if self.hideBeyond is None or self._bend() <= self.hideBeyond:
                detections.append(tip + self._rng.normal(0., self.NOISE, 3))
            if self.strayBlob is not None:
                detections.append(self.strayBlob + self._rng.normal(0., 3., 3))  # wobbly stray blob
            picked = pickTip(detections, self._background, self._lastTip, self._prediction, self._reach)
            if picked is not None:
                self._lastTip = picked
                frames.append(picked)
        if not frames:
            return None, None, 0
        frames = np.array(frames)
        return np.median(frames, axis=0), frames.std(axis=0), len(frames)

    def waitSettled(self, **kwargs):
        return True

    def close(self):
        pass


class EmioRobot:
    """
    The real Emio: 4 motors (rad) and the tip marker seen by the depth camera (scene frame, mm).
    Put a coloured marker on the tip. Calibrate the camera beforehand (Emio Labs, camera calibration module).

    The camera may also detect other blobs of the marker's colour (background). At start-up, with the robot at rest,
    the blobs that don't move are recorded: the one closest to the robot's axis is the tip marker, the others are
    background and ignored. Then the tip is tracked: in each frame, the detection closest to its last position.
    """

    def __init__(self, configuration=config.CONFIGURATION, trackingTime=2.):
        from emioapi.emiomotors import EmioMotors
        from emioapi.emiocamera import EmioCamera

        self.motors = EmioMotors()
        if self.motors.findAndOpen() < 0:
            raise RuntimeError("Could not connect to the Emio motors. Check the USB and the power supply.")

        self.camera = EmioCamera(track_markers=True, configuration=configuration)
        cameras = EmioCamera.listCameras()
        index = self.motors.device_index
        serial = cameras[index] if cameras and index is not None and index < len(cameras) else None
        if not self.camera.open(serial):
            self.motors.close()
            raise RuntimeError("Could not open the depth camera.")

        self._lastTip = None
        self._background = np.zeros((0, 3))
        try:
            self._initTracking(trackingTime)
        except RuntimeError:
            self.close()
            raise

    def _detections(self):
        """All the blobs of the marker's colour in the next frame, within the robot's reach (n, 3)."""
        self.camera.update()
        markers = np.array([m[0:3] for m in self.camera.trackers_pos], dtype=float).reshape(-1, 3)
        radius = np.hypot(markers[:, 0], markers[:, 2])
        inReach = (radius <= config.WORKSPACE_RADIUS) & (markers[:, 1] >= config.WORKSPACE_Y[0]) \
            & (markers[:, 1] <= config.WORKSPACE_Y[1])
        return markers[inReach]

    def _initTracking(self, duration):
        """Find the tip marker and the static background blobs, with the robot at rest."""
        detections, nbFrames = [], 0
        end = time.time() + duration
        while time.time() < end:
            frame = self._detections()
            nbFrames += 1
            detections += list(frame)
        # Group the detections into blobs (10 mm), and keep the ones seen in at least 30% of the frames
        clusters = []
        for p in detections:
            for c in clusters:
                if np.linalg.norm(c["center"] - p) < 10.:
                    c["points"].append(p)
                    c["center"] = np.mean(c["points"], axis=0)
                    break
            else:
                clusters.append({"center": p, "points": [p]})
        static = [c["center"] for c in clusters if len(c["points"]) >= 0.3 * max(nbFrames, 1)]
        if not static:
            raise RuntimeError("The camera doesn't see the tip marker. Check the marker, the lighting and the "
                               "camera calibration.")
        static = np.array(static)
        tipIndex = int(np.argmin(np.hypot(static[:, 0], static[:, 2])))
        if np.hypot(static[tipIndex, 0], static[tipIndex, 2]) > config.TIP_SEARCH_RADIUS:
            raise RuntimeError(f"No marker near the robot's axis (closest blob at {np.round(static[tipIndex], 1)} mm)."
                               " Start with the robot at rest, hanging straight.")
        self._lastTip = static[tipIndex]
        self._background = np.delete(static, tipIndex, axis=0)
        print(f"Tip marker found at {np.round(self._lastTip, 1)} mm; {len(self._background)} background blob(s) "
              f"ignored{': ' + str(np.round(self._background, 0).tolist()) if len(self._background) else ''}")

    @property
    def angles(self):
        return np.array(self.motors.angles, dtype=float)

    @angles.setter
    def angles(self, value):
        self.motors.angles = [float(a) for a in value]

    CURRENT_UNIT = 2.69  # mA per unit of the present current register (XM430)

    @property
    def currents(self):
        """
        Motor currents (mA, absolute values). `EmioMotors.currents` fails in emioapi 0.3.0 (its motor configs have no
        `current_unit`), so the raw register is read and converted. Zeros if that's not possible either (the
        over-current check is then off).
        """
        try:
            raw = np.array(self.motors._mg.getPresentCurrent(), dtype=float)
            raw[raw > 32767] -= 65536  # signed 16-bit register
            return np.abs(raw * self.CURRENT_UNIT)
        except Exception as e:
            if not getattr(self, "_warnedCurrents", False):
                print(f"WARNING: can't read the motor currents ({e}): the over-current check is off.")
                self._warnedCurrents = True
            return np.zeros(4)

    def setReach(self, base, radius):
        """Ignore detections farther than `radius` mm from the robot's base `base` (it can't reach there)."""
        self._reach = (np.asarray(base, dtype=float), float(radius))

    def setPrediction(self, position):
        """
        Where the tip is expected (e.g. from the calibrated physics model), or None. While set, only detections within
        `PREDICTION_GATE` mm of it can be the tip, so a stray blob elsewhere is never taken for the tip.
        """
        self._prediction = None if position is None else np.asarray(position, dtype=float)
        if self._prediction is not None:
            self._lastTip = self._prediction.copy()

    def refreshTracking(self, duration=2.):
        """With the robot at rest: find the tip again and add the static blobs seen now to the background."""
        previous = self._background
        self._initTracking(duration)
        if len(previous):
            merged = list(self._background)
            for p in previous:
                if not merged or np.min(np.linalg.norm(np.array(merged) - p, axis=1)) > config.BACKGROUND_RADIUS:
                    merged.append(p)
            self._background = np.array(merged).reshape(-1, 3)

    def _readMarker(self, timeout=1.):
        """One camera reading of the tip marker (the detection closest to its last position), or None."""
        end = time.time() + timeout
        while time.time() < end:
            picked = pickTip(self._detections(), self._background, self._lastTip, getattr(self, "_prediction", None),
                             getattr(self, "_reach", None))
            if picked is not None:
                self._lastTip = picked
                return picked
        return None

    def readTip(self, nFrames=config.N_FRAMES):
        """Median of `nFrames` readings of the tip marker: (position, std, number of frames used)."""
        frames = [p for p in (self._readMarker() for _ in range(nFrames)) if p is not None]
        if not frames:
            return None, None, 0
        frames = np.array(frames)
        return np.median(frames, axis=0), frames.std(axis=0), len(frames)

    def waitSettled(self, timeout=config.SETTLE_TIMEOUT, tolerance=config.SETTLE_TOLERANCE):
        """Wait until the tip moves less than `tolerance` mm between two readings. Returns False on timeout."""
        end = time.time() + timeout
        previous = self._readMarker()
        while time.time() < end:
            time.sleep(0.1)
            current = self._readMarker()
            if previous is not None and current is not None and np.linalg.norm(current - previous) < tolerance:
                return True
            previous = current
        return False

    def close(self):
        """Close the camera and the motors (closing the motors turns their torque off)."""
        try:
            self.camera.close()
        finally:
            self.motors.close()


class EmioMotorsOnly:
    """
    The real Emio's 4 motors only, without the camera (for steering by eye, `teleop.py`).
    Same interface as `EmioRobot` for the motors: `angles` (get/set, rad), `currents` (mA), `close()`.
    """

    CURRENT_UNIT = EmioRobot.CURRENT_UNIT
    currents = EmioRobot.currents

    def __init__(self, port=None):
        from emioapi.emiomotors import EmioMotors
        self.motors = EmioMotors()
        if self.motors.findAndOpen(device_name=port) < 0:
            raise RuntimeError(f"Could not connect to the Emio motors{' on ' + port if port else ''}. Check the USB, "
                               f"the power supply, and that Emio Labs / other scripts aren't using the port.")

    @property
    def angles(self):
        return np.array(self.motors.angles, dtype=float)

    @angles.setter
    def angles(self, value):
        self.motors.angles = [float(a) for a in value]

    def close(self):
        """Close the connection (turns the motors' torque off)."""
        self.motors.close()


def rampTo(robot, target, maxStep=config.MAX_STEP, period=config.STEP_PERIOD, maxCurrent=config.MAX_CURRENT,
           start=None):
    """
    Move to `target` (rad) in steps of at most `maxStep` rad, checking the motor currents at each step.
    On over-current, go back to the starting angles and raise `OverCurrentError`.
    `start`: last commanded angles (defaults to the measured ones).
    """
    target = np.asarray(target, dtype=float)
    start = robot.angles if start is None else np.asarray(start, dtype=float)
    nbSteps = max(1, int(np.ceil(np.max(np.abs(target - start)) / maxStep)))
    for k in range(1, nbSteps + 1):
        robot.angles = start + (target - start) * k / nbSteps
        if period > 0 and not isinstance(robot, FakeRobot):
            time.sleep(period)
        currents = robot.currents
        if np.any(currents > maxCurrent):
            reached = start + (target - start) * k / nbSteps
            rampTo(robot, start, maxStep, period, maxCurrent=np.inf, start=reached)
            raise OverCurrentError(f"Motor current {np.round(currents).tolist()} mA above {maxCurrent} mA")
    return target
