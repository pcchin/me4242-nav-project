"""
Emio set up as a tendon-driven continuum robot: instead of the legs, each motor winds a cable on a spool.
The four cables go from the motors to the center of the box and down through the disks of a flexible backbone.
Set the pulled length of each cable from the Move window (or from the CableMotorController in the scene graph).
"""
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
MOTOR_DIRECTIONS = [1., -1., 1., -1.]  # sign of the motor rotation that winds each cable
MOTORS_DISTANCE = 100.  # mm, distance from the motors to the center (same as Emio)
MAX_PULL = 9.  # mm, limit of the cable inputs (~1.5 x CABLE_OFFSET, about 90 degrees of bending)


def getParserArgs():
    import sys
    import argparse

    parser = argparse.ArgumentParser(prog=sys.argv[0],
                                     description='Simulate Emio as a cable-driven continuum robot.')
    parser.add_argument('--no-connection', dest="connection", action='store_false',
                        help="use when you want to run the simulation without the robot")
    try:
        args = parser.parse_args()
    except SystemExit:
        Sofa.msg_error(sys.argv[0], "Invalid arguments, get defaults instead.")
        args = parser.parse_args([])

    return args


def createScene(rootnode):
    from emio.utils.header import addHeader, addSolvers
    from emio.parts.continuumrobot import ContinuumRobot
    from emio.parts.controllers.cablecontroller import CableMotorController

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

    # Continuum robot hanging from the center of the box, cable i faces motor i
    robot = ContinuumRobot(name="ContinuumRobot", nbCables=4, pullPoints=motorPositions,
                           length=ROBOT_LENGTH,
                           nbSections=(NB_DISKS - 1) * 3,  # one disk every 3 elements
                           nbDisks=NB_DISKS,
                           cableOffset=CABLE_OFFSET,
                           diskMass=DISK_MASS,
                           radius=WIRE_RADIUS,
                           youngModulus=WIRE_YOUNG_MODULUS,
                           massDensity=WIRE_MASS_DENSITY)
    simulation.addChild(robot)

    # Cable inputs (mm) -> cables in simulation and motor angles (rad)
    controller = rootnode.addObject(CableMotorController(robot.cables,
                                                         pulleyRadius=PULLEY_RADIUS,
                                                         directions=MOTOR_DIRECTIONS))

    # GUI: one slider per cable in the Move window, tensions in the Simulation State window
    try:
        import Sofa.ImGui as MyGui
        MyGui.MoveWindow.setActuatorsDescription("Cables pulled length (mm)")
        # Same as the direct control in the labs: each input is a vector, the second list gives the index in each vector
        MyGui.MoveWindow.setActuators(controller.inputs, [0] * len(controller.inputs), "displacement")
        MyGui.MoveWindow.setActuatorsLimits(0., MAX_PULL)
        for i, cable in enumerate(robot.cables):
            MyGui.SimulationState.addData("Cables tension", "C" + str(i), cable.CableConstraint.force)
            MyGui.SimulationState.addData("Motors angle", "M" + str(i), controller.angles[i])
    except ImportError:
        Sofa.msg_info(__file__, "No SofaImGui, set the cable inputs from the CableMotorController in the scene graph.")

    # Connection to the real robot: the motors follow the angles computed from the cable inputs
    if args.connection:
        from emio.parts.controllers.motorcontroller import MotorController
        rootnode.addObject(MotorController(controller.angles, name="MotorController"))

    return rootnode
