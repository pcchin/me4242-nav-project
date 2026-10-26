import Sofa


def setCableDisplacement(cable, displacement):
    """
    Drive a simulated cable with the given displacement (mm, positive when the cable is wound).

    Only a pulled cable imposes its length. A released cable (displacement <= 0) is paid out by its motor, so in the
    simulation it is left free (zero force) instead of imposing its length: with four cables imposing their lengths,
    the problem is over-constrained (the four lengths cannot all be met exactly) and the solver makes the robot explode.
    """
    if displacement > 0.:
        cable.CableConstraint.findData('valueType').read("displacement")
        cable.CableConstraint.value = [displacement]
    else:
        cable.CableConstraint.findData('valueType').read("force")
        cable.CableConstraint.value = [0.]


class CableMotorController(Sofa.Core.Controller):
    """
    Links the input of each cable (pulled length, in mm) to:
        - the `CableConstraint` of the simulated cable,
        - the angle of the motor winding this cable on its spool: angle = direction * displacement / pulleyRadius.

    The cables work in antagonistic pairs (cable i and cable i + nbCables/2) and are never slack:
    pulling a cable releases the opposite one by the same length, which lets the robot bend toward the pulled cable.
    The displacement of cable i is thus `input[i] - input[opposite(i)]`, positive when the cable is wound and
    negative when it is released.

    The displacements follow the inputs at a limited speed (`maxStep` mm per time step), so that moving a slider
    does not yank the robot (nor the real motors).

    The motor angles are Data fields (`motorAngle0`, `motorAngle1`, ...) that can be given to the `MotorController`
    to drive the real robot.

    With `inputMode="angles"`, the inputs are instead the 4 motor angles (`angle0`, `angle1`, ..., in rad), driven
    independently (no pairing): the displacement of cable i is `direction * pulleyRadius * angle[i]`. This is the mode
    used by the learned inverse kinematics.

    Class Members:
        - `inputs`: list of Data, pulled length of each cable (mm), or motor angles (rad) in "angles" mode.
          Set them from the GUI or from the scene graph.
          Each one is a vector of one value (like `JointActuator.value`), as expected by the GUI's Move window:
          `MyGui.MoveWindow.setActuators(controller.inputs, [0] * len(controller.inputs), "displacement")`.
        - `angles`: list of Data, corresponding motor angles (rad).

    Arguments:
        - `cables`: list of cable nodes, each holding a `CableConstraint` named "CableConstraint".
        - `pulleyRadius` (`float`): radius of the spools fixed on the motors (mm).
        - `directions` (`list[float]`): 1 or -1 for each motor, the sign of the rotation that winds the cable.
          The motors of Emio face each other two by two, so by default [1, -1, 1, -1]. Check it on your robot.
        - `maxStep` (`float`): maximum change of a cable displacement per time step (mm). In "angles" mode, the
          angles change by at most `maxStep / pulleyRadius` rad per time step.
        - `inputMode` (`str`): "pulls" (pulled lengths, antagonistic pairs) or "angles" (4 independent motor angles).
        - `slack` (`list[float]`, "angles" mode): angle (rad) each motor turns before its cable starts pulling
          (negative: the cable already pulls at 0 rad). From a calibration, default 0.
        - `gain` (`list[float]`, "angles" mode): how strongly each cable shortens the robot once taut
          (1 = ideal cable). From a calibration, default 1.
          The simulated cable displacement is `gain * pulleyRadius * (direction * angle - slack)`.
    """

    def __init__(self, cables, pulleyRadius=10., directions=None, maxStep=0.1, inputMode="pulls", slack=None,
                 gain=None, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "CableMotorController"

        self.cables = cables
        self.pulleyRadius = pulleyRadius
        self.directions = directions if directions is not None else [1., -1., 1., -1.]
        self.maxStep = maxStep
        self.inputMode = inputMode
        self.slack = slack if slack is not None else [0.] * len(cables)
        self.gain = gain if gain is not None else [1.] * len(cables)
        self.displacements = [0.] * len(cables)
        self.currentAngles = [0.] * len(cables)

        self.inputs = []
        self.angles = []
        for i in range(len(cables)):
            if inputMode == "angles":
                self.addData(name="angle" + str(i), type="vector<double>", value=[0.],
                             help="Angle of motor " + str(i) + " (rad)", group="Inputs")
                self.inputs.append(self.findData("angle" + str(i)))
            else:
                self.addData(name="cable" + str(i), type="vector<double>", value=[0.],
                             help="Pulled length of cable " + str(i) + " (mm)", group="Inputs")
                self.inputs.append(self.findData("cable" + str(i)))
            self.addData(name="motorAngle" + str(i), type="float", value=0.,
                         help="Angle of motor " + str(i) + " (rad)", group="Outputs")
            self.angles.append(self.findData("motorAngle" + str(i)))

    def getDisplacement(self, i):
        """
        Target displacement of cable i (mm), positive when wound:
            - "pulls" mode: its pulled length minus the pulled length of the opposite cable,
            - "angles" mode: the length its motor angle shortens the robot by (with the slack and gain).
        """
        if self.inputMode == "angles":
            return self.gain[i] * self.pulleyRadius * (self.directions[i] * self.inputs[i].value[0] - self.slack[i])
        opposite = (i + len(self.cables) // 2) % len(self.cables)
        return self.inputs[i].value[0] - self.inputs[opposite].value[0]

    def onAnimateBeginEvent(self, _):
        if self.inputMode == "angles":
            # The angles follow the inputs at a limited speed; they go to the motors as they are
            maxAngleStep = self.maxStep / self.pulleyRadius
            for i in range(len(self.cables)):
                target = self.inputs[i].value[0]
                self.currentAngles[i] += max(-maxAngleStep, min(maxAngleStep, target - self.currentAngles[i]))
                self.angles[i].value = self.currentAngles[i]
                self.displacements[i] = self.gain[i] * self.pulleyRadius * (
                        self.directions[i] * self.currentAngles[i] - self.slack[i])
            # Two opposite cables both pulling would only squash the backbone (and make the simulation diverge with
            # a stiff wire): only the difference bends the robot, so the common part is removed (one active per pair)
            half = len(self.cables) // 2
            for i in range(half):
                common = min(max(self.displacements[i], 0.), max(self.displacements[i + half], 0.))
                self.displacements[i] -= common
                self.displacements[i + half] -= common
            for i, cable in enumerate(self.cables):
                setCableDisplacement(cable, self.displacements[i])
            return

        for i, cable in enumerate(self.cables):
            target = self.getDisplacement(i)
            step = max(-self.maxStep, min(self.maxStep, target - self.displacements[i]))
            self.displacements[i] += step
            setCableDisplacement(cable, self.displacements[i])
            self.angles[i].value = self.directions[i] * self.displacements[i] / self.pulleyRadius
