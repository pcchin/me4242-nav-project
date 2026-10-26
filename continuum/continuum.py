"""
Emio set up as a tendon-driven continuum robot: instead of the legs, each motor winds a cable on a spool.
The four cables go from the motors to the center of the box and down through the disks of a flexible backbone.

Three ways to drive it:
    - default: the 4 motor angles (rad), from the Move window,
    - `--inputs pulls`: the pulled length of each cable (mm, antagonistic pairs), from the Move window,
    - `--model <model.npz>`: a target tip position (TCP controls of the Move window), reached with the learned
      inverse kinematics
      (see continuum_learning/README.md).

If `continuum_learning/models/sim_calibration.json` exists (made by `calibrate_sim.py` from the real robot's data),
the simulation uses it so that the simulated tip goes where the real tip goes for the same motor angles: each
motor's slack and gain, the cables' angles, the robot's length and its base position. `--calibration none` turns it off.
"""
import json
import os
import sys
from math import pi, sin, cos

import Sofa
from splib3.loaders import getLoadingLocation

# Real device settings, check them on your robot
PULLEY_RADIUS = 20.  # mm, radius of the spools fixed on the motors (40 mm diameter)
ROBOT_LENGTH = 195.  # mm, length of the continuum robot (bending part)
NB_DISKS = 10  # spacer disks, base included
CABLE_OFFSET = 6.  # mm, distance from the backbone axis to the cable holes
DISK_MASS = 1.0e-3  # kg, mass of one spacer disk (weigh yours)
# Backbone: flexible wire (units: mm, kg, s -> Young modulus in kPa, density in kg/mm^3)
WIRE_RADIUS = 0.5  # mm
WIRE_YOUNG_MODULUS = 2.0e8  # spring steel ~200 GPa (nitinol ~5.0e7)
WIRE_MASS_DENSITY = 7.85e-6  # spring steel (nitinol ~6.45e-6)
MOTOR_DIRECTIONS = [1., 1., 1., 1.]  # sign of the motor rotation that winds each cable (all wind when positive)
MOTORS_DISTANCE = 100.  # mm, distance from the motors to the center (same as Emio)
MAX_PULL = 9.  # mm, limit of the cable inputs (~1.5 x CABLE_OFFSET, about 90 degrees of bending)


def getParserArgs():
    import sys
    import argparse

    parser = argparse.ArgumentParser(prog=sys.argv[0],
                                     description='Simulate Emio as a cable-driven continuum robot.')
    parser.add_argument('--no-connection', dest="connection", action='store_false',
                        help="use when you want to run the simulation without the robot")
    parser.add_argument('--inputs', choices=["pulls", "angles"], default="angles",
                        help="inputs of the Move window: motor angles (rad) or cable pulled lengths (mm)")
    parser.add_argument('--model', default=None,
                        help="learned model (model.npz): drive the robot to a target position with the learned IK")
    parser.add_argument('--calibration', default="continuum_learning/models/sim_calibration.json",
                        help="simulation calibration from calibrate_sim.py (relative to this folder), or 'none'")
    try:
        args = parser.parse_args()
    except SystemExit:
        Sofa.msg_error(sys.argv[0], "Invalid arguments, get defaults instead.")
        args = parser.parse_args([])

    return args


def createScene(rootnode):
    from emio.utils.header import addHeader, addSolvers
    from continuumrobot import ContinuumRobot
    from cablecontroller import CableMotorController

    args = getParserArgs()

    settings, modelling, simulation = addHeader(rootnode)
    settings.addObject('RequiredPlugin', name="continuum",
                       pluginName=['Sofa.GL.Component.Rendering3D'])  # Needed to use components [OglModel]
    # More damping than the default: the wire is very stiff and light, this keeps the simulation stable
    addSolvers(simulation, rayleighStiffness=0.1)

    rootnode.dt = 0.01
    rootnode.gravity = [0., -9810., 0.]
    rootnode.VisualStyle.displayFlags.value = ["showBehavior", "hideWireframe"]

    # Structure of Emio (visual only)
    box = modelling.addChild("Box")
    box.addObject("MeshSTLLoader", filename=getLoadingLocation("../../data/meshes/base-extended.stl", __file__))
    box.addObject("OglModel", src=box.MeshSTLLoader.linkpath, color=[1, 1, 1, 0.05])

    # Motors positions, same as in the Emio class. The cables are pulled from there.
    motorPositions = []
    for i in range(4):
        angle = 2. * pi / 4 * i
        motorPositions.append([MOTORS_DISTANCE * sin(angle), 0., MOTORS_DISTANCE * cos(angle)])
    motors = modelling.addChild("Motors")
    motors.addObject("MechanicalObject", template="Vec3", position=motorPositions,
                     showObject=True, showObjectScale=8, drawMode=1, showColor=[0.2, 0.2, 0.2, 1])

    # Calibration from the real robot's data (calibrate_sim.py): geometry, and each motor's slack and gain
    calibration = None
    here = os.path.dirname(os.path.abspath(__file__))
    calibrationPath = args.calibration if os.path.isabs(args.calibration) else os.path.join(here, args.calibration)
    if args.calibration != "none" and os.path.exists(calibrationPath):
        with open(calibrationPath) as f:
            calibration = json.load(f)
        Sofa.msg_info(__file__, f"Using the simulation calibration {calibrationPath} "
                                f"(mean tip error {calibration['error_mean_mm']:.1f} mm on the data)")

    # Continuum robot hanging from the center of the box, cable i faces motor i
    robot = ContinuumRobot(name="ContinuumRobot", nbCables=4, pullPoints=motorPositions,
                           length=calibration["length"] if calibration else ROBOT_LENGTH,
                           translation=calibration["base"] if calibration else [0., 0., 0.],
                           cableAngles=calibration["cable_angles_rad"] if calibration else None,
                           nbSections=(NB_DISKS - 1) * 3,  # one disk every 3 elements
                           nbDisks=NB_DISKS,
                           cableOffset=CABLE_OFFSET,
                           diskMass=DISK_MASS,
                           radius=WIRE_RADIUS,
                           youngModulus=WIRE_YOUNG_MODULUS,
                           massDensity=WIRE_MASS_DENSITY)
    simulation.addChild(robot)

    # Inputs (cable pulls in mm, or motor angles in rad) -> cables in simulation and motor angles (rad)
    inputMode = "angles" if args.model else args.inputs
    controller = rootnode.addObject(CableMotorController(robot.cables,
                                                         pulleyRadius=PULLEY_RADIUS,
                                                         directions=MOTOR_DIRECTIONS,
                                                         inputMode=inputMode,
                                                         slack=calibration["slack_rad"] if calibration else None,
                                                         gain=calibration["gain"] if calibration else None))

    # Learned inverse kinematics: target position -> 4 motor angles
    learnedIK = None
    if args.model:
        from emio.parts.controllers.learnedik import LearnedIKController
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "continuum_learning"))
        import ik
        modelPath = args.model if os.path.isabs(args.model) else os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                              args.model)
        model = ik.loadModel(modelPath)
        restTip = [float(v) for v in model["forward"].predict([0., 0., 0., 0.])[0]]
        # Target, shown with a red dot: moved with the TCP sliders of the Move window (same setup as the labs' targets)
        target = modelling.addChild("Target")
        target.addObject('EulerImplicitSolver', firstOrder=True)
        target.addObject('CGLinearSolver', iterations=50, tolerance=1e-10, threshold=1e-10)
        target.addObject("MechanicalObject", template="Rigid3", position=[restTip + [0., 0., 0., 1.]],
                         showObject=True, showObjectScale=15)
        targetDot = target.addChild("Dot")
        targetDot.addObject("MechanicalObject", template="Vec3", position=[[0., 0., 0.]],
                            showObject=True, showObjectScale=10, drawMode=1, showColor=[1, 0, 0, 1])
        targetDot.addObject("RigidMapping", index=0)
        # Where the model predicts the real tip goes (green dot): on the red target when it's reachable.
        # The "TCP" frame (same position) is what the GUI shows as the robot's current tool position.
        predicted = modelling.addChild("PredictedTip")
        predicted.addObject("MechanicalObject", template="Vec3", position=[restTip],
                            showObject=True, showObjectScale=7, drawMode=1, showColor=[0, 1, 0, 1])
        tcp = modelling.addChild("TCP")
        tcp.addObject("MechanicalObject", template="Rigid3", position=[restTip + [0., 0., 0., 1.]])
        # Workspace: every tip position measured during data collection (what the robot can reach and the model knows)
        workspace = modelling.addChild("Workspace")
        workspace.addObject("MechanicalObject", template="Vec3", position=model["train_xyz"].tolist(),
                            showObject=True, showObjectScale=4, drawMode=0, showColor=[0.2, 0.6, 1, 0.8])
        learnedIK = rootnode.addObject(LearnedIKController(target, controller, model, ik.solveIK,
                                                           predictedNodes=[predicted, tcp]))

    # GUI
    try:
        import Sofa.ImGui as MyGui
        if learnedIK is not None:
            # Target sliders in the My Robot window (Target group); the IK sets the motor angles
            restTarget = [learnedIK.targetX.value, learnedIK.targetY.value, learnedIK.targetZ.value]
            for axis, rest in zip("XYZ", restTarget):
                MyGui.MyRobotWindow.addSettingInGroup(axis.lower() + " (mm)", learnedIK.findData("target" + axis),
                                                      rest - 80., rest + 80., "Target")
            # Also the TCP controls of the Move window, if the GUI shows them with this solver
            MyGui.setIPController(rootnode.Modelling.Target, rootnode.Modelling.TCP, rootnode.ConstraintSolver)
            MyGui.MoveWindow.setTCPLimits(-80., 80., -pi, pi)
            for axis in "XYZ":
                MyGui.SimulationState.addData("Learned IK", "predicted " + axis.lower(),
                                              learnedIK.findData("predicted" + axis))
            MyGui.SimulationState.addData("Learned IK", "error (mm)", learnedIK.error)
        else:
            # Same as the direct control in the labs: each input is a vector, the second list gives the index in each vector
            MyGui.MoveWindow.setActuators(controller.inputs, [0] * len(controller.inputs), "displacement")
            if inputMode == "angles":
                MyGui.MoveWindow.setActuatorsDescription("Motors angle (rad)")
                MyGui.MoveWindow.setActuatorsLimits(-MAX_PULL / PULLEY_RADIUS, MAX_PULL / PULLEY_RADIUS)
            else:
                MyGui.MoveWindow.setActuatorsDescription("Cables pulled length (mm)")
                MyGui.MoveWindow.setActuatorsLimits(0., MAX_PULL)
        for i, cable in enumerate(robot.cables):
            MyGui.SimulationState.addData("Cables tension", "C" + str(i), cable.CableConstraint.force)
            MyGui.SimulationState.addData("Motors angle", "M" + str(i), controller.angles[i])
    except ImportError:
        Sofa.msg_info(__file__, "No SofaImGui, set the inputs from the CableMotorController in the scene graph.")

    # Connection to the real robot: the motors follow the angles computed from the cable inputs
    if args.connection:
        from emio.parts.controllers.motorcontroller import MotorController
        rootnode.addObject(MotorController(controller.angles, name="MotorController"))

    return rootnode
