import ctypes
import os
import sys

import Sofa


class Keyboard:
    """
    Reads whether keys are held down, directly from Windows, and only while the SOFA window (this process) is the
    active window. Used because the GUI doesn't pass key presses on to the scene. Does nothing on other systems.
    """

    def __init__(self):
        self.enabled = sys.platform.startswith("win")
        if self.enabled:
            from ctypes import wintypes
            self._user32 = ctypes.windll.user32
            self._pid = wintypes.DWORD()

    def focused(self):
        """True when the active window belongs to this process (the SOFA window)."""
        hwnd = self._user32.GetForegroundWindow()
        self._user32.GetWindowThreadProcessId(hwnd, ctypes.byref(self._pid))
        return self._pid.value == os.getpid()

    def isDown(self, key):
        """`key`: a letter, e.g. "W"."""
        return bool(self._user32.GetAsyncKeyState(ord(key.upper())) & 0x8000)

    def pressed(self, keys):
        """The keys among `keys` currently held down (empty if the SOFA window isn't active)."""
        if not self.enabled or not self.focused():
            return []
        return [k for k in keys if self.isDown(k)]


class LearnedIKController(Sofa.Core.Controller):
    """
    Drives the 4 motors of the continuum robot to a target tip position with a learned inverse kinematics
    (see `labs/sandbox/continuum_learning`).

    The target is the position of `targetNode`'s `MechanicalObject` (mm, scene frame of Emio). Move it with:
        - the keyboard, while the SOFA window is active: hold W / S to move it along +x / -x, A / D along -z / +z
          (`keySpeed` mm/s); R puts it back at its initial position,
        - the sliders `targetX/Y/Z` (e.g. in the GUI's My Robot window), kept in sync with the target node both ways,
        - the TCP controls of the GUI's Move window once it is given to the GUI with `MyGui.setIPController`,
        - or its position in the scene graph.
    By default only (x, z) are targeted and y follows: bending raises the tip, so most (x, y, z) targets are out of
    reach. Set `controlY` to True to also target y (only if the model says y is controllable).

    When the target has moved by more than `solveDistance` mm, the controller solves the inverse kinematics (starting
    from the previous solution, for a smooth motion) and writes the 4 motor angles to the inputs of a
    `CableMotorController` in "angles" mode, which rate-limits them, moves the simulated cables and feeds the
    `MotorController` of the real robot.

    Outputs: `predictedX/Y/Z` (tip position predicted by the learned model, mm), `error` (mm) and `reachable`.

    Arguments:
        - `targetNode`: node with a `MechanicalObject` (Vec3 or Rigid3) holding the target.
        - `cableController`: `CableMotorController` with `inputMode="angles"`.
        - `model`: learned model loaded with `ik.loadModel`.
        - `solveIK`: the `ik.solveIK` function.
        - `predictedNodes` (optional): nodes with a `MechanicalObject` (Vec3 or Rigid3) moved to where the model
          predicts the real tip goes (e.g. a marker, and the TCP shown by the GUI).
        - `keySpeed` (`float`): speed of the target while a key is held (mm/s).
        - `solveDistance` (`float`): the IK is solved again when the target has moved by more than this (mm).
    """

    # Key -> (axis index, direction): W/S along x, A/D along z
    KEYS = {"W": (0, 1.), "S": (0, -1.), "D": (2, 1.), "A": (2, -1.)}

    def __init__(self, targetNode, cableController, model, solveIK, predictedNodes=(), keySpeed=20.,
                 solveDistance=1., *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "LearnedIKController"

        self.targetNode = targetNode
        self.predictedNodes = list(predictedNodes)
        self.cableController = cableController
        self.model = model
        self.solveIK = solveIK
        self.theta = None
        self.solvedTarget = None
        self.keySpeed = keySpeed
        self.solveDistance = solveDistance
        self.keyboard = Keyboard()

        # Target sliders (e.g. in the GUI's My Robot window), kept in sync with the target node both ways
        initial = [float(v) for v in targetNode.getMechanicalState().position.value[0][0:3]]
        for axis, value in zip("XYZ", initial):
            self.addData(name="target" + axis, type="float", value=value, help="Target tip position (mm)",
                         group="Target")
        self.synced = initial
        self.initialTarget = list(initial)
        for axis in "XYZ":
            self.addData(name="predicted" + axis, type="float", value=0., help="Predicted tip position (mm)",
                         group="Outputs")
        self.addData(name="controlY", type="bool", value=False,
                     help="Also target y (otherwise y follows x and z)", group="Target")
        self.addData(name="error", type="float", value=0., help="Predicted distance to the target (mm)",
                     group="Outputs")
        self.addData(name="reachable", type="bool", value=True, help="Target reachable", group="Outputs")

    def getTarget(self):
        target = [float(v) for v in self.targetNode.getMechanicalState().position.value[0][0:3]]
        if not (self.controlY.value and self.model["y_controllable"]):
            target[1] = float("nan")  # y follows (x, z): leave it free
        return target

    @staticmethod
    def _moveTo(node, position):
        state = node.getMechanicalState()
        current = list(state.position.value[0])
        state.position.value = [list(position) + current[3:]]  # keeps the orientation of a Rigid3

    def syncSliders(self):
        """A moved slider moves the target node; a moved target node (keyboard, TCP controls) moves the sliders."""
        sliders = [self.targetX.value, self.targetY.value, self.targetZ.value]
        node = [float(v) for v in self.targetNode.getMechanicalState().position.value[0][0:3]]
        if any(abs(s - p) > 1e-6 for s, p in zip(sliders, self.synced)):
            self._moveTo(self.targetNode, sliders)
            self.synced = sliders
        elif any(abs(n - p) > 1e-6 for n, p in zip(node, self.synced)):
            for axis, value in zip("XYZ", node):
                self.findData("target" + axis).value = value
            self.synced = node

    def moveWithKeys(self, keys, dt):
        """Move the target for the keys held down during a time step `dt` (s)."""
        if not keys:
            return
        position = [float(v) for v in self.targetNode.getMechanicalState().position.value[0][0:3]]
        if "R" in keys:
            position = list(self.initialTarget)
        else:
            for key in keys:
                if key in self.KEYS:
                    axis, direction = self.KEYS[key]
                    position[axis] += direction * self.keySpeed * dt
        self._moveTo(self.targetNode, position)

    def onAnimateBeginEvent(self, event):
        keys = self.keyboard.pressed(list(self.KEYS) + ["R"])
        self.moveWithKeys(keys, self.getContext().dt.value)
        self.syncSliders()
        target = self.getTarget()
        # While a key is held, solve every `solveDistance` mm (solving takes time); otherwise solve any change
        tolerance = self.solveDistance if keys else 0.05
        if self.solvedTarget is not None and all(abs(a - b) < tolerance or (a != a and b != b)
                                                 for a, b in zip(target, self.solvedTarget)):
            return

        result = self.solveIK(target, self.model, initialGuess=self.theta)
        self.theta = result["theta"]
        self.solvedTarget = target
        for i, angle in enumerate(self.theta):
            self.cableController.inputs[i].value = [float(angle)]
        predicted = [float(v) for v in result["predicted"]]
        for axis, value in zip("XYZ", predicted):
            self.findData("predicted" + axis).value = value
        for node in self.predictedNodes:
            self._moveTo(node, predicted)
        self.error.value = result["error"]
        self.reachable.value = bool(result["reachable"])
