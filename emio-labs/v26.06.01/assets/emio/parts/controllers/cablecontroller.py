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

    Class Members:
        - `inputs`: list of Data, pulled length of each cable (mm). Set them from the GUI or from the scene graph.
          Each one is a vector of one value (like `JointActuator.value`), as expected by the GUI's Move window:
          `MyGui.MoveWindow.setActuators(controller.inputs, [0] * len(controller.inputs), "displacement")`.
        - `angles`: list of Data, corresponding motor angles (rad).

    Arguments:
        - `cables`: list of cable nodes, each holding a `CableConstraint` named "CableConstraint".
        - `pulleyRadius` (`float`): radius of the spools fixed on the motors (mm).
        - `directions` (`list[float]`): 1 or -1 for each motor, the sign of the rotation that winds the cable.
          The motors of Emio face each other two by two, so by default [1, -1, 1, -1]. Check it on your robot.
        - `maxStep` (`float`): maximum change of a cable displacement per time step (mm).
    """

    def __init__(self, cables, pulleyRadius=10., directions=None, maxStep=0.1, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "CableMotorController"

        self.cables = cables
        self.pulleyRadius = pulleyRadius
        self.directions = directions if directions is not None else [1., -1., 1., -1.]
        self.maxStep = maxStep
        self.displacements = [0.] * len(cables)

        self.inputs = []
        self.angles = []
        for i in range(len(cables)):
            self.addData(name="cable" + str(i), type="vector<double>", value=[0.],
                         help="Pulled length of cable " + str(i) + " (mm)", group="Inputs")
            self.addData(name="motorAngle" + str(i), type="float", value=0.,
                         help="Angle of motor " + str(i) + " (rad)", group="Outputs")
            self.inputs.append(self.findData("cable" + str(i)))
            self.angles.append(self.findData("motorAngle" + str(i)))

    def getDisplacement(self, i):
        """
        Target displacement of cable i (mm): its pulled length minus the pulled length of the opposite cable.
        """
        opposite = (i + len(self.cables) // 2) % len(self.cables)
        return self.inputs[i].value[0] - self.inputs[opposite].value[0]

    def onAnimateBeginEvent(self, _):
        for i, cable in enumerate(self.cables):
            target = self.getDisplacement(i)
            step = max(-self.maxStep, min(self.maxStep, target - self.displacements[i]))
            self.displacements[i] += step
            setCableDisplacement(cable, self.displacements[i])
            self.angles[i].value = self.directions[i] * self.displacements[i] / self.pulleyRadius
