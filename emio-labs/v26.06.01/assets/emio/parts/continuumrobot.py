"""
This module defines the `ContinuumRobot` class: a tendon-driven continuum robot made of a flexible backbone
(beam model from BeamAdapter) and cables (CableConstraint from SoftRobots) routed through spacer disks.
The file `continuumrobot.py` also includes an example usage, which can be tested by running the script with the `runSofa` command:
```bash
runSofa -l SofaPython3,SofaImGui -g imgui continuumrobot.py
```
"""

from math import pi, sin, cos

import Sofa
import numpy as np

import emio.parameters as parameters


class ContinuumRobot(Sofa.Prefab):
    """
    The `ContinuumRobot` class represents a tendon-driven continuum robot hanging from its base.

    Class Overview:
        - `backbone`: The flexible rod (Rigid3 frames, beam model).
        - `cables`: List of cable nodes, each holding a `CableConstraint` named "CableConstraint".

    Constructor Arguments:
        - `pullPoints` (`list[Vec3d]`, optional): Where each cable is pulled from (e.g. the motors' spools).
          By default, 10 mm above the cable hole of the base. Cable `i` goes through the holes at angle
          `2*pi*i/nbCables` around Y (same convention as the Emio motors), so cable `i` faces motor `i`.
        - `cableAngles` (`list[float]`, optional): angle of each cable's holes around Y (rad, 0 = +Z, pi/2 = +X),
          e.g. from a calibration. Defaults to `2*pi*i/nbCables`.

    Class Variables:
        - `length` (`float`): Length of the backbone (mm).
        - `nbSections` (`int`): Number of beam elements.
        - `nbDisks` (`int`): Number of spacer disks the cables pass through (the base counts as one).
        - `nbCables` (`int`): Number of cables, evenly distributed around the backbone.
        - `cableOffset` (`float`): Distance from the backbone axis to the cable holes (mm).
        - `radius` (`float`): Radius of the backbone (circular cross-section).
        - `massDensity`, `poissonRatio`, `youngModulus` (`float`): Material of the backbone.
        - `translation` (`Vec3d`): Position of the base.

    Example Usage:
    ```python
    robot = ContinuumRobot(name="ContinuumRobot", nbCables=4)
    simulation.addChild(robot)
    robot.cables[0].CableConstraint.value = [5.]  # pull the first cable by 5 mm
    ```
    """
    prefabData = [
        {'name': 'length', 'type': 'float', 'help': 'Backbone length (mm)', 'default': 150.},
        {'name': 'nbSections', 'type': 'int', 'help': 'Number of beam elements', 'default': 30},
        {'name': 'nbDisks', 'type': 'int', 'help': 'Number of spacer disks (base included)', 'default': 7},
        {'name': 'nbCables', 'type': 'int', 'help': 'Number of cables', 'default': 4},
        {'name': 'cableOffset', 'type': 'float', 'help': 'Cable distance from axis (mm)', 'default': 8.},
        {'name': 'radius', 'type': 'float', 'help': 'Backbone radius (mm)', 'default': 2.},
        {'name': 'diskMass', 'type': 'float', 'help': 'Mass of one spacer disk (kg), 0 to ignore the disks', 'default': 0.},

        {'name': 'massDensity', 'type': 'float', 'help': '', 'default': parameters.massDensity},
        {'name': 'poissonRatio', 'type': 'float', 'help': '', 'default': parameters.poissonRatio},
        {'name': 'youngModulus', 'type': 'float', 'help': '', 'default': parameters.youngModulus},
        {'name': 'translation', 'type': 'Vec3d', 'help': 'Position of the base', 'default': [0.0, 0.0, 0.0]},
    ]

    def __init__(self, pullPoints=None, cableAngles=None, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

        self._pullPoints = pullPoints
        self._cableAngles = cableAngles
        self._addRequiredPlugins()
        positions = self._getBackbonePositions()
        self._addBackbone(positions)
        self._addDisks(positions)
        self._addCables(positions)

    def _getBackbonePositions(self):
        """
        Rigid3 frames of the backbone, hanging along -Y from `translation`.
        The beam axis is the local X axis, so each frame is rotated by -90 degrees around Z (local X -> world -Y).
        """
        q = [0., 0., -sin(pi / 4), cos(pi / 4)]
        ds = self.length.value / self.nbSections.value
        t = self.translation.value
        return [[t[0], t[1] - i * ds, t[2]] + q for i in range(self.nbSections.value + 1)]

    def _addBackbone(self, positions):
        """
        FEM line model from the plugin BeamAdapter, clamped at its base.
        """
        self.backbone = self.addChild("Backbone")
        self.backbone.addObject('MeshTopology',
                                position=[pos[0:3] for pos in positions],
                                edges=[[i, i + 1] for i in range(len(positions) - 1)])
        self.backbone.addObject('MechanicalObject', template='Rigid3', position=positions,
                                showObject=True, showObjectScale=3)
        self.backbone.addObject('BeamInterpolation', straight=True,
                                crossSectionShape="circular",
                                radius=self.radius.value,
                                defaultYoungModulus=self.youngModulus.value,
                                defaultPoissonRatio=self.poissonRatio.value)
        self.backbone.addObject('AdaptiveBeamForceFieldAndMass', computeMass=True,
                                massDensity=self.massDensity.value)
        # The base of the backbone is clamped. Replace this with an attachment if the base should move.
        self.backbone.addObject('FixedProjectiveConstraint', indices=[0])

    def _getDiskIndices(self):
        """
        Indices of the backbone frames carrying a spacer disk (evenly spread from the base to the tip).
        """
        n = self.nbSections.value
        nbDisks = max(2, self.nbDisks.value)
        return sorted(set(round(k * n / (nbDisks - 1)) for k in range(nbDisks)))

    def _addDisks(self, positions):
        """
        The mass of the spacer disks, rigidly attached to the backbone frames (the base disk is fixed, so it is skipped).
        With a thin wire as backbone, the disks carry most of the mass and inertia of the robot.
        """
        if self.diskMass.value <= 0.:
            return
        diskIndices = self._getDiskIndices()[1:]
        disks = self.backbone.addChild("Disks")
        disks.addObject('MechanicalObject', template='Rigid3', position=[positions[i] for i in diskIndices])
        disks.addObject('UniformMass', totalMass=self.diskMass.value * len(diskIndices))
        disks.addObject('RigidMapping', globalToLocalCoords=True, rigidIndexPerPoint=diskIndices)

    def _addCables(self, positions):
        """
        Each cable is a list of points (one per disk, offset from the axis) rigidly attached to the backbone frames.
        The CableConstraint shortens the cable between its pull point (above the base) and its last point (the tip).
        """
        diskIndices = self._getDiskIndices()
        self.cables = []
        for c in range(self.nbCables.value):
            angle = self._cableAngles[c] if self._cableAngles is not None else 2. * pi * c / self.nbCables.value
            # The disks are in the world XZ plane since the backbone hangs along -Y.
            # Same angle convention as the motors of Emio: [sin, 0, cos]
            offset = np.array([self.cableOffset.value * sin(angle), 0., self.cableOffset.value * cos(angle)])
            points = [list(np.array(positions[i][0:3]) + offset) for i in diskIndices]
            if self._pullPoints is not None:
                pullPoint = list(self._pullPoints[c])
            else:
                pullPoint = list(np.array(points[0]) + [0., 10., 0.])

            cable = self.backbone.addChild("Cable" + str(c))
            cable.addObject('MechanicalObject', template='Vec3', position=points,
                            showObject=True, showObjectScale=2, showColor=[1, 1, 0, 1])
            cable.addObject('CableConstraint', name="CableConstraint",
                            indices=list(range(len(points))),
                            pullPoint=pullPoint,
                            valueType="displacement", value=[0.])
            cable.addObject('RigidMapping', globalToLocalCoords=True, rigidIndexPerPoint=diskIndices)
            self.cables.append(cable)

    def _addRequiredPlugins(self):
        """
        Add the RequiredPlugins of the ContinuumRobot class.
        """
        plugins = self.addChild("RequiredPlugins")
        plugins.addObject('RequiredPlugin',
                          pluginName=['Sofa.Component.Topology.Container.Constant'  # Needed to use components [MeshTopology]
                                      , 'Sofa.Component.Constraint.Projective'  # Needed to use components [FixedProjectiveConstraint]
                                      , 'Sofa.Component.Mapping.NonLinear'  # Needed to use components [RigidMapping]
                                      , 'Sofa.Component.Mass'  # Needed to use components [UniformMass]
                                      , 'BeamAdapter'
                                      , 'SoftRobots'  # Needed to use components [CableConstraint]
                                      ])


def createScene(rootnode):
    from emio.utils.header import addHeader, addSolvers
    from splib3.animation import AnimationManager, animate
    from emio.parts.controllers.cablecontroller import setCableDisplacement
    """
    Test the simulation of a continuum robot: the cables are pulled one after the other.
    """

    settings, modelling, simulation = addHeader(rootnode)
    rootnode.addObject(AnimationManager(rootnode))
    rootnode.VisualStyle.displayFlags = ["showBehavior"]
    rootnode.dt = 0.01
    rootnode.gravity = [0., -9810., 0.]
    addSolvers(simulation)

    robot = ContinuumRobot(name="ContinuumRobot", nbCables=4)
    simulation.addChild(robot)

    # Pull each cable in turn, with a sine bump of 10 mm, while the opposite cable is released by the same amount
    nbCables = len(robot.cables)

    def animation(cables, factor):
        phase = factor * nbCables
        pulls = []
        for c in range(nbCables):
            local = phase - c
            pulls.append(10. * sin(pi * local) if 0. <= local <= 1. else 0.)
        for c, cable in enumerate(cables):
            setCableDisplacement(cable, pulls[c] - pulls[(c + nbCables // 2) % nbCables])

    animate(animation, {'cables': robot.cables}, duration=12., mode='loop')

    return robot  # used for tests
