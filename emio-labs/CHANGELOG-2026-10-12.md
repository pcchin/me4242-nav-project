# Changelog — 2026-10-12

Continuum robot for Emio: a cable-driven (tendon-driven) continuum robot driven by the four Emio servo motors, in place of the legs.

> **Status: not yet tested.** The scenes run only inside the Emio Labs app, which bundles SOFA, so none of the changes below could be run while they were being written.
> Open issues:
> - **Robot "flying all over the place" in the simulation.** The likely main cause (over-constrained cables) and three other fixes have been added, but none has been checked yet. See [Stability](#stability).
> - **Robot connection drops right after connecting, then SOFA crashed while loading a plugin.** The Emio Labs app's unpacked files in Temp were found damaged. See [Robot connection](#robot-connection).

## Files

| File | Status |
|---|---|
| `v26.06.01/assets/emio/parts/continuumrobot.py` | New |
| `v26.06.01/assets/emio/parts/controllers/cablecontroller.py` | New |
| `v26.06.01/assets/labs/sandbox/continuum.py` | New |
| `v26.06.01/assets/labs/sandbox/sandbox.md` | Modified |

No existing Emio file (`leg.py`, `robot.py`, `motor.py`, `motorcontroller.py`, …) was changed.

---

## `emio/parts/continuumrobot.py` (new)

`ContinuumRobot` prefab: a flexible backbone bent by cables routed through spacer disks.

- **Backbone:** Rigid3 frames hanging along −Y from `translation`, modelled with BeamAdapter (`BeamInterpolation`, circular cross-section, and `AdaptiveBeamForceFieldAndMass`). Each frame is rotated −90° about Z because BeamAdapter beams run along the local X axis. The base frame is clamped with `FixedProjectiveConstraint`.
- **Disks:** frames spread evenly along the backbone, base included (`nbDisks`).
- **Disk mass (`diskMass`, added later today):** a `UniformMass` on each disk frame except the fixed base, attached with `RigidMapping`. Set it to 0 to leave the disks out.
- **Cables:** one SoftRobots `CableConstraint` per cable (`valueType="displacement"`). Each cable has one point per disk, `cableOffset` mm from the axis, attached to its disk frame with `RigidMapping` (`rigidIndexPerPoint`).
  - Cable *i* sits at angle `2πi/nbCables` around Y, using the same `[sin, 0, cos]` convention as the Emio motors, so cable *i* faces motor *i*.
  - The optional constructor argument `pullPoints` sets where each cable is pulled from (the motors). By default it is 10 mm above the base.
  - The cables are created in `"displacement"` mode with no `minForce`. During the simulation, `setCableDisplacement()` in `cablecontroller.py` switches a released cable to zero force (see [Stability](#stability)).
- **Test scene (`createScene`):** pulls each cable in turn with a 10 mm sine bump, while the opposite cable is released by the same amount.
- **Parameters:** `length`, `nbSections`, `nbDisks`, `nbCables`, `cableOffset`, `radius`, `diskMass`, `massDensity`, `poissonRatio`, `youngModulus`, `translation`.

## `emio/parts/controllers/cablecontroller.py` (new)

`CableMotorController` links the cable inputs to the simulation and to the motors.

- **Inputs:** `cable0`–`cable3`, the length each cable is pulled, in mm.
- **Outputs:** `motorAngle0`–`motorAngle3`, in rad.
- **Antagonistic pairs (0↔2, 1↔3):** pulling a cable releases the opposite one by the same amount, so the robot bends toward the pulled cable and no cable goes slack.
- **Each step:**
  - The target is `input[i] − input[opposite]`: positive when the cable is wound, negative when it is released.
  - The displacement moves toward the target by at most `maxStep` (0.1 mm per step).
  - `setCableDisplacement()` applies it: a pulled cable holds its length; a released cable is set to zero force.
  - `angle = direction × displacement / pulleyRadius`.
- **Motor directions:** set with the `directions` argument. The default is `[1, -1, 1, -1]` because the Emio motors face each other in pairs; this needs checking on the robot.

## `labs/sandbox/continuum.py` (new)

The Emio scene for the continuum robot.

- Shows the Emio extended box (visual only), with the 4 motor positions at 100 mm from the center as in the `Emio` class.
- The `ContinuumRobot` hangs from the center, and each cable is pulled from its motor.
- **GUI:**
  - Move window: one slider per cable, from 0 to `MAX_PULL`.
  - Simulation State window: each cable's tension and each motor's angle.
  - Without SofaImGui, the inputs can be edited in the scene graph instead.
- **Real robot:** reuses Emio's `MotorController`, which drives the motors from `motorAngle0`–`motorAngle3` and uses the GUI's connection toggle. `--no-connection` runs the simulation alone.
- **Damping:** `addSolvers(simulation, rayleighStiffness=0.1)`, up from the default 0.01.

Device settings, at the top of the file:

| Setting | Value | Source |
|---|---|---|
| `PULLEY_RADIUS` | 20 mm (40 mm diameter spool) | User: 40 mm diameter, then 20 mm, then back to 40 mm (2026-10-13) |
| `ROBOT_LENGTH` | 195 mm | User |
| `NB_DISKS` | 10, base included | User |
| `CABLE_OFFSET` | 6 mm | User |
| `DISK_MASS` | 1 g | **Placeholder**: weigh a disk |
| `WIRE_RADIUS` | 0.5 mm (1 mm wire) | **Placeholder** |
| `WIRE_YOUNG_MODULUS` | 2.0e8 kPa (spring steel) | **Placeholder**: nitinol ≈ 5.0e7 |
| `WIRE_MASS_DENSITY` | 7.85e-6 kg/mm³ (spring steel) | **Placeholder**: nitinol ≈ 6.45e-6 |
| `MOTOR_DIRECTIONS` | `[1, -1, 1, -1]` | **To check** on the robot |
| `MOTORS_DISTANCE` | 100 mm | Same as `Emio` |
| `MAX_PULL` | 9 mm (≈ 1.5 × cable offset) | Derived |

The backbone uses `(NB_DISKS − 1) × 3 = 27` elements of 7.2 mm, so every third frame carries a disk.

Units: mm, kg, s, so Young's modulus is in kPa.

## `labs/sandbox/sandbox.md` (modified)

Added a **Continuum Robot** section at the end of the Sandbox page with three buttons:
- a SOFA button for `assets/labs/sandbox/continuum.py` (the Emio scene),
- an Open button for the same file,
- a SOFA button for `assets/emio/parts/continuumrobot.py` (the robot alone, cables animated).

---

## Design decisions and their history

### Cable slack and opposite cables

1. **First version:** `CableConstraint` with `minForce=0`, so cables could only pull and would go slack otherwise.
2. **Antagonistic pairs:** the user asked that cables not go slack. `minForce` was removed (cables became bilateral) and opposite cables were paired (0↔2, 1↔3): pulling one cable released the opposite one by the same amount.
3. **Held at fixed length:** the user first decided against antagonistic actuation. The pairing was removed and every cable followed its own input, with the others holding their length. On a backbone that doesn't stretch, this blocks the bending: the opposite side has to lengthen, but its cable is held, so the robot can only bend by compressing the backbone.
4. **Back to antagonistic pairs** (step 2): when one cable pulls, the opposite one releases by the same amount, allowing the robot to bend.
5. **Final, simulation only:** to fix the instability, a released cable no longer holds its length in the simulation and is set to zero force. On the real robot nothing changes: its motor still pays it out by the same amount and it stays taut.

Notes on the final version:
- **Pre-tension:** the released cable's motor turns backwards from angle 0, so mount the cables taut at angle 0.
- **Accuracy:** releasing exactly the pulled length is exact for a constant-curvature bend of a thin robot. With a 6 mm offset over 195 mm it's very close, but at large bends the released cable may tighten slightly in the simulation. Releasing a bit more would fix that.

### Stability

The user reported the robot "flying all over the place" after the steel wire was introduced. The likely cause is that the wire is very stiff and light (≈ 1.2 g), while the cables pull 6 mm off its axis. Fixes:
- The disks now have mass (`diskMass`, 1 g each, placeholder).
- Rayleigh stiffness damping went from 0.01 to 0.1.

**Main cause found later: the four cables over-constrained the robot.** With all four cables holding an exact length, there was no exact solution:
- Lengthening the backbone changes all four cables equally, so cables 0 + 2 always change by the same total as 1 + 3. One constraint is redundant.
- When the robot bends, the cables run in straight lines between disks, so "pull *d* / release *d*" doesn't match exactly. Only stretching or squashing the steel wire could absorb the difference.

The constraint solver's forces kept growing, which threw the robot around. Fixes in `cablecontroller.py`:
- `setCableDisplacement()`: a **pulled** cable (displacement > 0) holds its length. A **released** cable is set to zero force (`valueType` switched to `"force"`). On the real robot, the motor pays it out and it stays taut. At most two constraints are active, one per bending direction.
- `maxStep` (0.1 mm per time step, i.e. 10 mm/s): the cable values follow the sliders gradually instead of jumping. The motor angles sent to the robot are limited the same way.
- The standalone test in `continuumrobot.py` uses `setCableDisplacement()` too.

**2026-10-13: headless tests, and a GUI fix.** The scene was run without a window using the SOFA bundled with the app (`runSofa -g batch`). Stable in every case:
- at rest;
- cable 0 pulled 6 mm: about 57° of bend;
- cables 0 and 1 pulled 9 mm, then a jump to cable 2;
- the standalone test.

The robot still flew in the GUI, so the likely cause is the Move window sliders:
- The scene gave the sliders four single-number inputs with positions `[0, 1, 2, 3]`, copied from the inverse-control GUI (`robot.py`).
- The direct-control labs (`lab_models.py`, `lab_inversekinematics.py`) instead give **one-element lists** with position `0` for every slider. The position is the entry inside each list.
- With the old setup, sliders 1–3 pointed past the end of their one-number fields.

Fix:
- `cable0`–`cable3` are now `vector<double>` with one value.
- `continuum.py` calls `setActuators(inputs, [0, 0, 0, 0], "displacement")` and `setActuatorsLimits(0, MAX_PULL)`.
- From code, set an input with `controller.cable0.value = [5.]`.

Not yet checked in the GUI.

**SOFA plugin warning:** the scene now declares the plugins for `UniformMass` (`Sofa.Component.Mass`, in `continuumrobot.py`) and `OglModel` (`Sofa.GL.Component.Rendering3D`, in `continuum.py`). This silences the `SceneCheckMissingRequiredPlugin` warning. Behaviour is unchanged.

Next steps if it still flies:
- **At rest:** lower `dt` to 0.005, or try the nitinol values.
- **Only when pulling:** look at the cable constraints.

## Robot connection

**Symptom:** in the GUI, the connection switches on, then fails with "Could not find or connect Emio robots. Please check the connection." and switches off again.

**What was checked** (nothing was changed):
- **"Robot connected." is misleading.** It is printed by the GUI (`SofaImGui.dll`) when the toggle is switched on, not when the motors answer. The error comes from `findAndOpen()` failing in `motorcontroller.py` (lines 28–32).
- **The hardware is fine.** The Emio USB adapter (FTDI) is on **COM9**. COM4 and COM5 are Bluetooth ports. All four motors (IDs 0–3, XM430-W210) answered a read-only ping at 1 Mbaud.
- **The connection call works outside SOFA.** `EmioMotors().findAndOpen()` connected on COM9, with all motors reading about 0 rad, and the connection was closed again.
- **Closing a connection disables torque.** `close()` turns off the motor torque, so the motors release any cable tension when the connection drops.

**Possible causes inside SOFA:**
- Another connection holding COM9: a SOFA window still open, or the previous scene's connection thread surviving a scene reload.
- The damaged app install below.

**Damaged app install.** Emio Labs runs from a copy it unpacks into `AppData\Local\Temp\<random>\` each time it starts. During the session, `resources\sofa\bin\python\python.exe` and `pythonw.exe` disappeared from that folder, and SOFA then crashed in `PluginManager::loadPlugin`. The cause is unknown: Windows Defender shows no detections, and Windows' automatic temp-file cleanup (Storage Sense) is a possible cause. `python.exe` from that folder was run twice during the connection checks above.

**Fix:**
1. Quit Emio Labs completely.
2. Restart it from `Downloads\emio-labs-v26.06.01_windows_amd64.exe`, which unpacks a fresh copy.
3. Don't reload the scene while connected.
4. If files keep disappearing, turn off Storage Sense's temp cleanup.

## To do

- [ ] Restart Emio Labs fresh, then check SOFA loads without the plugin crash.
- [ ] Run `continuum.py` in Emio Labs and check it is stable, at rest and when pulling.
- [ ] Check that switching `valueType` between `"displacement"` and `"force"` during the simulation works in this SoftRobots version.
- [ ] Connect the real robot on COM9 and check the connection stays on.
- [ ] Check the GUI sliders appear (`MoveWindow.setActuators` with plain float Data).
- [ ] Weigh a disk and set `DISK_MASS`.
- [ ] Set the wire's material and diameter.
- [ ] Check `MOTOR_DIRECTIONS` on the robot with small pulls.
- [ ] Mount the cables taut with the servos at angle 0.
- [ ] Optional: inverse control (`CableActuator` with `addHeader(inverse=True)`) to place the tip at a target.

## Other notes from today

While reviewing `emio/parts/leg.py` (unchanged), one possible bug was spotted:
- **Crash on missing files:** lines 161–167 rotate the extremity frame even when `_checkData()` fails. In that case `extremity` has no mechanical state, so the constructor raises an exception instead of leaving `isValid()` to return `False`.
