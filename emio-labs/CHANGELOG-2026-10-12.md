# Changelog — 2026-10-12 to 2026-10-19

Continuum robot for Emio: a cable-driven (tendon-driven) continuum robot driven by the four Emio servo motors, in place of the legs.

> **Status (2026-10-19):**
> - **Robot connection:** solved. The robot is on COM10.
> - **SOFA simulation:** stable in headless tests at rest, with pulls, with fast side switches, and in the angle and learned-IK modes. The "flying" seen in the GUI was most likely the slider setup, which has been fixed; to confirm in the GUI. See [Stability](#stability).
> - **Learned inverse kinematics** (from 2026-10-19, see the [section further down](#2026-10-19-learned-inverse-kinematics)): works on the real robot, with the model about 1–3 mm from the measured tip. What limits it is the hardware: slack cables (small reach, about 4 mm repeatability) and a marker the camera loses when the tip bends toward +x/+z.
> - **Next steps:** see the end of this file.

## Files

| File | Status |
|---|---|
| `v26.06.01/assets/emio/parts/continuumrobot.py` | New |
| `v26.06.01/assets/emio/parts/controllers/cablecontroller.py` | New |
| `v26.06.01/assets/labs/sandbox/continuum.py` | New |
| `v26.06.01/assets/labs/sandbox/sandbox.md` | Modified |
| `v26.06.01/assets/emio/parts/controllers/learnedik.py` | New (2026-10-19) |
| `v26.06.01/assets/labs/sandbox/continuum_learning/` (`config.py`, `robot_io.py`, `collect_data.py`, `train_model.py`, `ik.py`, `goto.py`, `README.md`) | New (2026-10-19) |

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

---

## 2026-10-19: learned inverse kinematics

**Goal:** set a tip position and have the real robot go there directly. The material parameters are unknown, so the mapping is learned from the real robot with the depth camera, instead of the SOFA model.

**Approach.** The 4 motor angles are redundant: many combinations reach the same tip position. A model trained directly on position → angles would average incompatible answers. So the scripts:
- learn the **forward model** f: 4 angles → tip (x, y, z);
- solve the inverse by **optimisation**: `argmin ‖W(f(θ) − p*)‖² + λ‖θ‖²`, plus a co-contraction penalty, within the angle limits. Among all solutions, this picks the one closest to rest, with no opposite cables wound against each other.

**New folder `labs/sandbox/continuum_learning/`** (usage in its `README.md`):
- `config.py`: shared settings (40 mm spool, motor directions, ±0.45 rad, 2 mm co-contraction limit, 600 mA current limit).
- `robot_io.py`: the real robot (`EmioMotors` plus the `EmioCamera` tip marker, in scene coordinates), a made-up 4-cable robot for tests, and ramped motion with a current check.
- `collect_data.py`: Sobol samples in the safe 4D region, visited in a short path order, with repeats and returns to rest. Writes a CSV row per sample and can resume after a stop.
- `train_model.py`:
  - compares a Gaussian process and an MLP on held-out data;
  - checks whether y is controllable on its own;
  - saves a portable `model.npz` (numpy only) and a report plot.
- `ik.py`: `loadModel()` and `solveIK()`. Needs only numpy and scipy, so it also runs in SOFA's Python, which has no scikit-learn.
- `goto.py`: moves the real robot to x, y, z and measures the error with the camera.

**Python environment:** a new conda environment `emio-ml` (Python 3.10, `emioapi`, scikit-learn 1.7.2, scipy 1.15.3). The `me4242` environment had none of these packages.

**SOFA integration:**
- `CableMotorController`: new `inputMode="angles"`, where 4 independent motor angles drive the cables, with no pairing.
- `emio/parts/controllers/learnedik.py`: new `LearnedIKController`.
  - The target is set in the My Robot window. When it moves, the angles are solved (starting from the previous solution) and fed to `CableMotorController`.
  - y is free by default (`controlY` to target it), because bending raises the tip.
- `continuum.py`: new `--inputs angles` and `--model <npz>` options. A relative model path is resolved from the scene's folder.
- `sandbox.md`: new buttons for angle inputs, the learned IK, and the README.

**Tested without hardware (made-up robot, 839 samples):**

| Check | Result |
|---|---|
| Forward model, Gaussian process (selected) | 0.18 mm mean test error |
| Forward model, MLP | 1.97 mm mean test error |
| IK, model error | 0.10 mm mean |
| `goto.py`, reachable targets | 0.05 mm measured error |
| `goto.py`, unreachable targets | Reported as out of reach; the robot goes to the closest point |
| Safety limits | All samples and IK solutions within ±0.45 rad and the 2 mm co-contraction limit |
| IK solve time | About 35 ms |

SOFA headless runs (`runSofa -g batch`) were stable in both `--inputs angles` and `--model` modes. Not yet run on the real robot.

**Next on the real robot:**
- [ ] Calibrate the camera and put a marker on the tip.
- [ ] Check `MOTOR_DIRECTIONS` with small angles, and mount the cables taut at 0 rad.
- [ ] `collect_data.py --n 200`, train, and check a few targets with `goto.py`.
- [ ] Collect the full set (about 1200 samples), retrain, and measure the error.

**Fix (2026-10-19): `AttributeError: 'EmioMotors' object has no attribute 'currents'`.** `pip install emioapi` installs 0.2.0, the latest on PyPI, which has no motor current reading. Emio Labs bundles 0.3.0.
- **Environment:** `emio-ml` now has 0.3.0 and `dynamixelmotorsapi`, copied from the app. 0.3.0 also matches the app's camera calibration.
- **`robot_io.py`:** if the current reading is missing, it now warns and skips the over-current check instead of crashing.
- **`collect_data.py`:** a data file holding only a header no longer gets a second header on resume.
- **README:** setup instructions updated for 0.3.0.
- **Follow-up:** reading the currents still failed in 0.3.0, because of a bug in Emio's `dynamixelmotorsapi`. Its `currents` property reads `MotorConfig.current_unit`, which doesn't exist. The app's copy has the same bug. `robot_io.py` now reads the raw `present_current` register (signed 16-bit, 2.69 mA per unit on the XM430). Checked on the robot: about 0 to 3 mA at rest.

**First real data collection (2026-10-19): unusable, camera picked the wrong blobs.** 233 rows with the motors fine (≤ 134 mA, angles within 0.012 rad of the commands), but:
- 197 rows were `not_settled`;
- the tip readings were impossible (z = 120–180 mm, with 72 mm of noise);
- the "rest" position drifted by 40 mm.

The camera sees up to 18 blobs of the marker's colour (2.9 per frame on average), and the script took the first one. The real tip marker was steady at about (−2, −139, −5) mm. The camera calibration was fine: `~/.config/emioapi/camera_2d_points.csv` is loaded at start-up.

Fix in `robot_io.py`:
- At start-up, with the robot at rest, the static blob closest to the axis is the tip, and the other static blobs are background.
- Detections outside the reachable region (`WORKSPACE_RADIUS`, `WORKSPACE_Y`) or near a background blob are ignored.
- The tip is tracked as the detection closest to its last position (`MAX_JUMP`), and samples use the median of 10 frames.
- New settings in `config.py`.

Result: 0.2–0.4 mm noise at rest, and the tip is tracked through moves of each motor. The bad file was kept as `data/samples_BAD_wrong_marker_2026-10-19.csv`.

Observations from that test (each motor at +0.15 rad):
- **Motor 1's direction:** motors 0 and 2 winded (loaded) and motor 3 released (0 mA), all as configured. Motor 1 was loaded (43 mA) although `MOTOR_DIRECTIONS` says −1 (release). Likely `[1, 1, 1, -1]`; to confirm on the spool.
- **Small motion:** only 5–10 mm of tip motion for 3 mm of cable, so the cables probably have slack at 0 rad.
- **Hysteresis:** the robot came back about 4 mm from its starting point.

**Second real collection (200 samples, 2026-10-19):** 194 `ok`, 33 `no_marker`, 6 `not_settled`.

| Result | Value |
|---|---|
| Repeatability | median 3.95 mm, max 8.3 mm |
| Forward model test error (GP) | 2.75 mm, at the data's noise floor |
| Height y | Follows (x, z), so targets are (x, z) |
| Reach | Only x −15…+36 mm, z −44…+26 mm |
| Rest position drift | Up to 8 mm |

- **Likely cause of the small reach, drift and scatter:** slack cables.
- **Lost marker:** all 33 `no_marker` rows bent toward +x/+z, away from the camera at the −x/−z corner. The marker turns away from the camera or is hidden by the robot. `goto.py 0 nan 20` (edge of reach, +z) lost the marker the same way.
- **To do:** move the marker so it's visible from all sides (the tip's bottom face, or a ball or ring), tension the cables at 0 rad, then collect again from scratch.
- **`goto.py`:** after the move, it now searches for the marker at the model's predicted tip position instead of the starting one.

**`goto.py` tests on the real robot (2026-10-19), with the 200-sample model:**

| Target (x, y, z) | Result |
|---|---|
| A camera-side target | Measured, **0.91 mm** error |
| `0 nan 20` and `0 -135 20` (toward +z) | `no marker seen`. y was ignored, because the model says y follows (x, z) |
| `-10 nan -30` | Out of reach; the closest predicted point is (−2.5, −26.4), 8.3 mm away |

- **Camera check at the +z pose:** the robot was moved to the `0 nan 20` angles and every detection was listed, unfiltered. The camera worked (122 frames in 4 s), but it only saw stray blobs around (−30, −50, 180) mm and nothing near the expected tip (0, −135, 20). So the marker is **physically hidden** when the tip bends away from the camera (−x/−z corner). The tracking isn't discarding it, and the camera isn't turning off.
- **`-10 nan -30`:** measured at (−3.7, −26.1), only **about 1.2 mm from the model's prediction**. The 7.4 mm error to the target comes from the limited reach, not from the model.
- **Why it's out of reach:** in the dry run, motor 3 sat at its **0.45 rad limit** (9 mm released). The reach is limited by `MAX_PULL` as well as by the slack cables.

**Change to `goto.py`:** it now prints two errors, to the **target** and to the **prediction**. The first includes "can't reach"; the second shows how well the model knows the robot.

**Status at the end of 2026-10-19:**
- `data/samples.csv` (200 samples, slack cables, side-facing marker) and `models/model.npz` (GP, 2.75 mm test error) are the current data and model. `data/samples_BAD_wrong_marker_2026-10-19.csv` is the unusable first collection.
- `config.py` is unchanged:
  - `MOTOR_DIRECTIONS = [1, -1, 1, -1]`. The motion test suggested motor 1 is `+1` (loaded when its angle increases); **still to confirm on the spool.**
  - `MAX_PULL = 9` mm, 40 mm spool, `MAX_COCONTRACTION = 2` mm, `MAX_CURRENT = 600` mA.
- What's limiting accuracy is the hardware, not the model.

**Next steps:**
- [ ] Confirm motor 1's direction on its spool; set `MOTOR_DIRECTIONS = [1, 1, 1, -1]` if it winds on a positive angle.
- [ ] Move the marker so the camera sees it in every bend (the tip's bottom face, or a ball or ring).
- [ ] Tension the cables with all motors at 0 rad.
- [ ] Optionally raise `MAX_PULL` (e.g. 14 mm) if the robot still bends too little, watching the currents.
- [ ] Rename `data/samples.csv`, collect again from scratch, and retrain. Aim for `no_marker` ≈ 0, repeatability well below 4 mm, and a larger reach in `model_report.png`.
- [ ] Choose `goto.py` targets inside the reach shown in `model_report.png`; check them with `--dry-run`.

## 2026-10-21: plots of the data and the model

**New script `continuum_learning/plot_model.py`.** Run `python plot_model.py`; it writes three PNGs to `models/`:
- **`plot_motor_effects.png`:** tip x, y, z against each motor's angle. Data points, plus the model's average prediction as that motor sweeps its range with the others as in the data (partial dependence), plus the model with the others at 0.
- **`plot_predicted_vs_measured.png`:** the same 80/20 split as `train_model.py`, with the model refitted on 80%. On 39 held-out samples: RMSE x 2.0, y 1.0, z 2.0 mm; mean 3D error 2.75 mm.
- **`plot_workspace.png`:** measured tip positions seen from above (coloured by y), with the model's predicted grid of bends.

**What the plots show** (200-sample data):
- **Motor 1 winds on a positive angle:** its curve is flat for negative angles and rises for positive ones. This confirms `MOTOR_DIRECTIONS` should be `[1, 1, 1, -1]`.
- **Motor 3 has almost no effect** over its whole range: cable 3 is very slack or loosely attached. That explains why the reach stops at about x = −15 mm.
- **Every motor acts on one side only** (flat on the release side): the cables are slack.
- **The workspace is a lopsided diagonal band** (x −15…+37, z −45…+30) instead of an area around the rest point. Causes: cable 3 not pulling, and the marker hidden on the +x/+z side.

**Next steps (updated):** tension cable 3 in particular (and the others), set `MOTOR_DIRECTIONS = [1, 1, 1, -1]`, move the marker, collect again, then `train_model.py` and `plot_model.py`. Expect a roughly round workspace around the rest point, and motor curves that slope on both sides.

## 2026-10-21: motor directions corrected, workspace shown in the simulation

**Motor directions:** confirmed by the user that motors 1 and 3 wind on positive angles. Motors 0 and 2 were already seen winding on positive angles.
- `MOTOR_DIRECTIONS = [1, 1, 1, 1]` in `continuum_learning/config.py` **and** `labs/sandbox/continuum.py`.
- **Simulation:** the simulated cables 1 and 3 now wind when the real ones do. Before, they released, so the simulated robot bent the wrong way.
- **Model retrained** on the same data (unchanged: GP, 2.75 mm), because the model file stores the directions for the co-contraction limit. The old model is kept as `models/model_old_directions.npz`.
- **Finding:** with the old directions, **41 of the 233 samples wound both cables 1 and 3 at once** (up to 7.4 mm), with the motors pulling against each other. Harmless here: the cables were slack and the motors drew ≤ 108 mA. Future collections will block those poses properly. A few of those measured poses are now outside the co-contraction limit, which is why the IK check gives a 0.23 mm mean but a 0.08 mm 95th percentile.

**Learned-IK scene (`continuum.py --model`):** two new displays.
- **Workspace cloud** (light blue points): every tip position measured during data collection, i.e. what the robot can reach and the model knows.
- **Green dot:** where the model predicts the **real** tip goes, updated by `LearnedIKController` (new optional `predictedNode` argument). It sits on the red target when the target is reachable, and at the closest reachable point otherwise.
- **Headless test with the real model:** 193 workspace points; target (15, −20) reached by the prediction (green = red, 0.02 mm).

**The simulated robot doesn't match the real one.** Same angles, target (15, −20):

| | x | y | z |
|---|---|---|---|
| Predicted real tip | 15 | −137 | −20 |
| SOFA tip | 47 | −177 | −53 |

The SOFA robot bends much more, and it hangs at y = −195 at rest against −139 for the real marker: a **56 mm difference in height/length**. Its cables are taut, while the real ones are slack. Trust the green dot (the real robot), not the simulated robot.

**Not yet applied:** the user says the robot can reach 2.5 rad. `MAX_PULL` is still 9 mm (0.45 rad).

## 2026-10-21: simulation calibrated to the real robot, motor angles as inputs

**Request:** make the simulation match the trained model; drive it from the motor angles instead of the cable lengths. Both were done.

**Motor angles are now the default input:**
- `continuum.py`: `--inputs angles` is the default; `--inputs pulls` keeps the cable-length mode.
- `sandbox.md`: the main button drives the 4 motor angles; a second button keeps the cable-length mode.
- `CableMotorController` in "angles" mode now rate-limits the **angles** (`maxStep / pulleyRadius` rad per step). The angles sent to the motors are exactly the rate-limited inputs; before, they were recomputed from the cable displacement.

**New `continuum_learning/calibrate_sim.py`** fits, from the collected data (193 samples), a constant-curvature model of how each motor angle bends the robot:
- per motor, a **slack** (rad turned before the cable pulls; negative = pre-tensioned at 0 rad) and a **gain** (how strongly the cable bends the robot);
- each **cable's angle** around the axis;
- the robot's **length** and **base position**.

It saves `models/sim_calibration.json` and a plot. Fitted values:

| Motor | Slack | Gain | Cable angle |
|---|---|---|---|
| 0 | −2.0 mm (−0.101 rad) | 0.18 | −19° |
| 1 | +1.0 mm | 0.21 | 81° |
| 2 | −2.9 mm | 0.25 | 160° |
| 3 | +3.1 mm | 0.11 | 257° |

- **Geometry:** length 192.6 mm (the user's 195 mm), base 54 mm above motor height, rest tip (0.3, −139, −1.1).
- **Fit error:** 3.0 mm mean, 5.1 mm 95th percentile. Without fitting the cable angles it was 5.75 mm: the cable holes are rotated about 15–20° from the camera's axes.

**SOFA uses the calibration** (`continuum.py --calibration`, default `continuum_learning/models/sim_calibration.json`; `none` turns it off):
- `ContinuumRobot`: new `cableAngles` argument; the length and base come from the calibration;
- `CableMotorController`: new `slack` and `gain`. The simulated cable displacement is `gain * r * (direction * angle - slack)`.

**Simulation fix:** two opposite cables both pulling (pre-tension, or both motors winding) made SOFA diverge right away (tip at 10⁷⁰ mm). The controller now removes their common part, so at most one cable per opposite pair is active. Only the difference bends the robot, as in the calibration model.

**Checked in headless SOFA** by replaying all 193 measured angle sets in the calibrated scene: stable, and the tip is **4.2 mm mean / 3.9 mm median / 9.0 mm 95th percentile** from the measured tip, about the real robot's own repeatability (about 4 mm). Learned-IK scene, target (15, −20): the simulated tip is 5 mm from the model's prediction, against about 55 mm before calibration.

**Drag the target (2026-10-21).** In the learned-IK scene (`continuum.py --model`), the target is now dragged in the 3D view instead of set with sliders.
- **Target:** the `Target` node is a Rigid3 frame (EulerImplicit + CG + `MechanicalObject`, the same setup as the labs' targets), with a red dot. It's given to the GUI with `MyGui.setIPController(Target, TCP, ConstraintSolver)`; `setIPController` accepts any constraint solver, checked with the bundled SofaImGui. That gives a draggable target in the 3D view, plus the TCP controls in the Move window (`setTCPLimits(-80, 80, -π, π)`).
- **TCP frame:** a new `TCP` node follows the model's predicted real tip, as does the green dot.
- **Sliders removed:** the My Robot target sliders are gone.
- **`LearnedIKController`:** reads the target from `targetNode`'s position (Vec3 or Rigid3), with no more `targetX/Y/Z` data or `initialTarget`. `predictedNode` became `predictedNodes` (a list).
- **Headless test:** moving the target to (15, −20) gave predicted, green dot and TCP at (15, −136.9, −20), with the calibrated simulated tip at (10.3, −137.1, −21.9). The drag handle itself can only be checked in the GUI.
- **Correction:** Emio's docs only describe moving the TCP target with the **Move window's TCP controls**. Dragging in the 3D view is unconfirmed (the GUI couldn't be run here). The Sandbox page and the `continuum.py` docstring now say "TCP controls of the Move window". Possible next step: give the target a small collision sphere so SOFA's standard Shift + left-click drag can move it (not done yet).

**Grab the target with the mouse (2026-10-21).** In the learned-IK scene, **Shift + left-click and drag** on the red target sphere moves the target, and the robot follows.
- **The target is now a simulated body:** 1 g, Euler implicit with heavy damping (`rayleighMass=20`), `SparseLDLSolver` and `GenericConstraintCorrection`, and gravity cancelled by an equal `ConstantForceField`. It has a `SphereCollisionModel` (radius 6 mm) on its red dot, so SOFA's mouse (`AttachBodyButtonSetting`, stiffness 10) can pick it. There's no collision pipeline, so the sphere collides with nothing; the scene check warns about that, which is expected.
- **Approaches that failed, found with a headless test** (a spring of stiffness 10 pulling the sphere toward a "mouse" point, then released):
  - the labs' massless target with no constraint correction: the force reached the target, but `FreeMotionAnimationLoop` never moved it;
  - a controller moving the target by force ÷ stiffness: it oscillated, because the loop restarts from the free position;
  - setting `gravity` on the target's node: it had no effect, and the target sank after the release.
- **Final result:**
  - the target reaches the mouse point (15, −20) in about 5–20 steps;
  - the IK follows (predicted 15.1, −136.9, −20.1) and the simulated tip moves to (10.4, −22.0);
  - after release, the target stays exactly in place;
  - setting the target directly, as the Move window's TCP controls do, still works.
- The Sandbox page now mentions Shift + drag. Dragging with the real mouse in the GUI is still to be confirmed by the user.
- **Mouse grab, follow-up:** in the GUI, Shift + click created SOFA's mouse objects, but they failed with `Invalid vOp operation 1 (null(V_DERIV) ...)` on `MechanicalObject(MousePosition)` and `(MappedMousePosition)`, so nothing moved. The spring-based `AttachBodyButtonSetting` (added by Emio's `addHeader`) doesn't work with the `FreeMotionAnimationLoop`. `continuum.py` now removes it and adds the constraint-based **`ConstraintAttachButtonSetting`** (`button="Left"`), which this SOFA build has (checked headless). The target and the robot both have `GenericConstraintCorrection`. The scene loads and runs; the grab is still to be confirmed in the GUI.
- **Mouse grab, follow-up 2:** with `ConstraintAttachButtonSetting` the user could grab, but grabbed "some other pink dot" instead of the red dot. The grab sphere was drawn in see-through red (pinkish) and the red dot was drawn separately on top of it. The red dot now **is** the grab sphere (`SphereCollisionModel`, radius 5 mm, solid red), and the separate point drawing is removed, so what you see is what you grab. The sphere is the scene's only collision model, so nothing else can be picked. Headless drag test: still fine (target at the "mouse" point (15, −20) by step 20, stays after release). The GUI check is pending.

**Mouse grab cancelled (2026-10-21, user's choice).** The user will move the target with the Move window's TCP sliders. Reverted in `continuum.py`:
- the target is back to the labs' setup: Rigid3 frame, `EulerImplicitSolver(firstOrder)` + `CGLinearSolver`, no mass, no `ConstantForceField`, no constraint correction;
- the red dot is a plain point again: no `SphereCollisionModel`;
- `AttachBodyButtonSetting` from `addHeader` is no longer replaced by `ConstraintAttachButtonSetting`;
- the extra plugins (Collision.Geometry, Mass, MechanicalLoad) are removed.

Kept: `setIPController(Target, TCP, ConstraintSolver)` and `setTCPLimits(-80, 80, -π, π)` for the TCP sliders, the green predicted-tip dot, and the workspace cloud. The Sandbox page and the `LearnedIKController` docstring no longer mention the mouse. This target setup was tested headless before the mouse work: setting the target directly, as the sliders do, drives the IK and the robot. It couldn't be rerun now because Emio Labs (and its bundled SOFA) wasn't running. Syntax checked.

**Target sliders restored (2026-10-21).** The user couldn't find a slider in the learned-IK scene: the My Robot target sliders had been removed for the mouse grab, and the Move window's TCP controls (`setIPController`) may not show up with this scene's non-inverse solver (every Emio lab pairs them with the QP inverse solver).
- **Sliders back:** `LearnedIKController` has `targetX/Y/Z` again, shown in **My Robot → Target** (`addSettingInGroup`, rest position ± 80 mm). They're synced both ways with the `Target` node (`syncSliders()`): a moved slider moves the target, and a target moved some other way (TCP controls, scene graph) moves the sliders.
- **TCP controls kept:** the `setIPController` hookup stays, in case the GUI shows them.
- **Headless test:** sliders to (15, −20) → target, prediction (15, −20), simulated tip (10.3, −21.9). Target node moved to (−5, 10) → sliders followed, prediction (−5, 10), simulated tip (−3.3, 13.4).

**WASD keyboard control (2026-10-21).** `LearnedIKController.onKeypressedEvent`:
- **W / S** move the target ±z, **A / D** move it ∓x / ±x, by `keyStep` = 2 mm per press; **R** resets it to the initial (rest) position;
- both lowercase and uppercase keys work;
- the sliders follow through `syncSliders()`.

SofaGLFW (Emio's GUI) forwards keys to the scene as `KeypressedEvent` (seen in the binary). As in other SOFA GUIs, this usually needs **Ctrl** held, with the 3D view focused; to be confirmed by the user. Headless test (key events sent to the controller): W×5 → z +10 mm, D×5 → x +10 mm, R → back to rest; the sliders, the prediction and the simulated tip followed. The Sandbox page mentions Ctrl + W/A/S/D.
- **Keyboard follow-up:** `onKeypressedEvent` got no keys in the GUI ("no response"), with or without Ctrl. Replaced by a `Keyboard` class in `learnedik.py` that reads the key state from Windows (`GetAsyncKeyState`) each step, only while the **SOFA window is the active window** (the foreground window belongs to the SOFA process), so typing elsewhere doesn't move the robot. Requested remap: **W / S → +x / −x, A / D → −z / +z**; **R** resets. **Holding** a key moves the target smoothly at `keySpeed` = 20 mm/s (sim time). While a key is held, the IK re-solves every `solveDistance` = 1 mm (each solve takes ≈35 ms), and once more on release. Headless test with a fake keyboard: W held 0.5 s → x +10 mm; D held 0.5 s → z +10 mm; R → rest; the sliders, the prediction and the simulated tip followed, and the prediction ends exactly on the target. The real Windows reader runs inside SOFA's Python (it reported "SOFA window not active" during the test, as expected). Typing W/A/S/D/R in a GUI text field also moves the target.

## 2026-10-23: full-range collection (2.5 rad) went wrong, tracking fixed

**What the user did:** set `MAX_PULL = 50` mm (2.5 rad) in `config.py` and `continuum.py`, then ran `collect_data.py` (1200 samples).
- The resume logic treated the 233 rows of the earlier 200-sample run in `data/samples.csv` as steps of the *new* plan, so the file now holds the old ±0.45 rad data (rows 0–232) followed by the new ±2.5 rad data.
- From row ~588, almost every reading sat at **(−34, −55, 163) mm** whatever the angles, marked `not_settled`: the tracker had locked onto a stray green blob once the tip marker went out of view at large bends. A second stray point at (−25, −171, −22) was found too (below the robot's lowest reach).
- **152 of the "ok" large-angle readings are physically impossible:** farther from the robot's base than its 193 mm length. The 56 that remain are about 96 mm from the physics prediction. **No reliable large-angle data;** the original 193 samples are still good.
- Motor 0 hit the 600 mA limit (605–619 mA) at large angles, so those samples were skipped.

**Fixes:**
- `robot_io.py`:
  - shared `pickTip()` filter: reach sphere, background blobs, prediction gate (`PREDICTION_GATE` 40 mm), then nearest to the last tip (`MAX_JUMP`);
  - `setPrediction()`, `setReach()`, and `refreshTracking()` (re-find the tip at rest and add new static blobs to the background);
  - `FakeRobot(hideBeyond=…, strayBlob=…)` reproduces the lost-tip/stray-blob problem.
- `collect_data.py`:
  - tracks the tip near the **calibrated physics prediction** (`sim_calibration.json`), but only for angles within the calibration's data range (`max_theta` + 0.1); beyond that the gate is off;
  - **reach check** (base, length + 10 mm);
  - `StuckDetector`: the measured tip moves < 2 mm over 5 samples while the predicted tip moves > 15 mm → `tracking_lost` until the next rest pose; at each rest pose, wait to settle and `refreshTracking()`;
  - `--max-theta` for staged collection;
  - new columns `max_theta` and `plan`; it **refuses to resume in a file of another plan**;
  - `--calibration`, `--simulate-trap`.
- `config.py`: `SETTLE_TIMEOUT` 3 → 5 s, `SETTLE_TOLERANCE` 0.2 → 0.4 mm, plus the new tracking constants.
- `train_model.py`: the model's `max_theta` = the largest |angle| in the data (min with `MAX_THETA`), so the IK never extrapolates.
- `calibrate_sim.py`: saves `max_theta` (the data's angle range).
- New `clean_data.py`: writes a cleaned copy with only good, non-stuck, reachable samples. Stuck readings are found with the physics model, so slack-cable samples (really still, and predicted still) are kept. On a snapshot of `samples.csv`: 2 stuck points (375 rows), 332 rows out of reach; **249 kept** (all 193 old + 56 doubtful large-angle).

**Tests (made-up robot):**
- With the camera trap (tip hidden beyond a 1.5 rad bend, stray blob at (−34, −55, 163)): no reading ever at the stray blob; `ok` readings within 0.28 mm of the true tip.
- Same with the prediction gate off.
- `StuckDetector` flags a still reading with a moving prediction after 5 samples, and doesn't flag a really still robot or a moving tip.
- Resume guard refuses a file of another plan.
- All edited files compile.

**Next:** stop the running collection; fix the marker so the camera sees it at large bends (and check the robot isn't physically caught); collect **in stages** (1.0 → 1.5 → 2.5 rad) with `clean_data.py` and `calibrate_sim.py` after each stage. The README has the commands.
- **Limit lowered to 2 rad (user's choice, 2026-10-23):** `MAX_PULL = 40` mm (2 rad on the 20 mm radius spool) in `continuum_learning/config.py` and `continuum.py`, so `MAX_THETA = 2.0` rad. The angle sliders, `collect_data.py` (default `--max-theta`), the IK cap and `clean_data.py` all follow. Staged collection: 1.0 → 1.5 → 2.0 rad.
- **Limit lowered to 1 rad (user's choice, 2026-10-23: "1 rad is enough"):** `MAX_PULL = 20` mm (1 rad on the 20 mm radius spool) in `config.py` and `continuum.py`; no staged collection needed. README section 2 rewritten: one collection (`collect_data.py --n 200 --out data/run1.csv`, then `clean_data.py`); more data = another `--seed` and a new file; train on all clean files together. The old 2b commands (default `data/samples.csv`, now refused by the plan guard) were removed.

## 2026-10-23: first 1 rad collection (`data/stage1.csv`) and what it showed

**Data:** 233 rows (plan seed0_n200_r30_max1): 107 `ok`, 122 `no_marker`, 4 `not_settled`; no stuck readings; motor currents ≤ 406 mA. `clean_data.py` → `stage1_clean.csv`: 107 samples. The tip now reaches x −82…75, z −43…127 mm and rises to y −25 mm, much bigger bends than before.

**Two bugs found in the new tracking (fixed):**
- **Reach check too tight:** the old calibration (base y = 53.6, length 193) allowed at most 202.6 mm from the base. The marker now sits lower (rest around y −149), so readings near rest were cut off as out of reach. The kept readings piled up just under the limit (max 202.4 mm). This removed the near-rest samples, including all 26 samples within the gated range and every rest pose. `REACH_MARGIN` 10 → **30 mm** in `collect_data.py` and `clean_data.py` (the stray blob at 227 mm is still rejected).
- **Outdated calibration:** `collect_data.py` now checks at start-up that the tip found at rest is within `PREDICTION_GATE/2` of the calibration's predicted rest tip; if not, it warns and turns off the prediction gate.

**Marker still hidden by bend direction** (gate off): no_marker 24 % toward −x/−z (the camera side), 40 % toward +x/−z, 53 % toward +x/+z, 72 % toward −x/+z. This is physical: the marker needs to be visible from all sides.

**Results:**
- `train_model.py` on `stage1_clean.csv`: **repeatability median 23.6 mm** (9 repeats; the motors return to the same angles within 0.003–0.03 rad, but the tip lands up to 32 mm apart, mostly with less bend on the 2nd visit at the end of the run). Test error 13.3 mm (GP); y follows (x, z); IK limited to 1.0 rad. Saved as `models/model.npz` (the previous file is `model_before_stage1.npz`).
- `calibrate_sim.py` (after a fix: the initial guess is clipped into the bounds, as there's no rest reading): **poor fit, 27.6 mm** (gain 1.29, length 143 mm). It was moved to `sim_calibration_stage1_poorfit.json`, and the previous calibration (3.0 mm on the old data) was restored as `sim_calibration.json`.
- Per-third linear fits of the tip against the angles vary (e.g. motor 2: −56 / −31 / −39 mm per rad), so drift vs. hysteresis isn't settled.

**Conclusion:** at ±1 rad the robot isn't repeatable (about 24 mm), from cable friction or hysteresis, and/or cable slip or stretch over the run. No model can do better than that. Next: check the cables physically (slipping on the spools, stretch, friction in the disk holes), test repeatability directly (the same `goto.py` target several times), and make the marker visible from all sides.

## 2026-10-26: direct WASD steering without the model

**Request:** control the end effector with WASD by eye, without the collected data or the model.

- **New `KeyboardBendController`** (`emio/parts/controllers/learnedik.py`, reusing the Windows `Keyboard` reader; the SOFA window must be active):
  - **W / S:** motor 1 winds and motor 3 releases by the same angle (or the reverse), so the tip bends toward +x / −x;
  - **D / A:** the same with motors 0 / 2, toward +z / −z;
  - **R:** all motors back to 0.
  - Opposite cables work in pairs (user's choice), so they never fight. The bend speed is **0.3 rad/s** (user's choice, `KEY_BEND_SPEED` in `continuum.py`). Each motor is clamped to ±`MAX_PULL/PULLEY_RADIUS` = ±1 rad. It writes the `CableMotorController` angle inputs only while a key is held, and reads the current angles first, so the Move window's angle sliders can be mixed with the keys.
  - Outputs `bendX` / `bendZ`, shown in Simulation State.
- **`continuum.py`:** `--inputs keys` (the angle mode underneath, plus the keyboard controller); the real robot follows through `MotorController` when connected.
- **`sandbox.md`:** a new button for it.
- **Tested:** logic with a Sofa stand-in (W 1 s → x 0.3 rad on motors 1/3; D 2 s → z 0.6 on motors 0/2; holds on release; continues from slider changes; clamps at ±1 rad; R resets). The SOFA headless run wasn't possible: the app's unpacked SOFA Python lost its standard library (`resources/sofa/bin/python/Lib` holds only `site-packages`), likely Windows temp cleanup again. Restart Emio Labs before using the scenes.
- **Note:** `collect_data.py`'s tracking-streak fix (see the saved plan) is **not** done yet.

## 2026-10-26: `teleop.py`, steering by eye from the terminal (no SOFA)

**Why:** the user wants to steer the real robot blindly with W/A/S/D, without data, camera, model or SOFA. In the SOFA scene, connecting failed repeatedly ("Could not find or connect"), while a direct connection to the robot (now on **COM12**, adapter FT9BTLU1; four different FTDI adapters seen on COM9–12) works: all 4 motors answer.

**New:**
- `robot_io.EmioMotorsOnly(port)`: the motors without the camera (`angles`, `currents` via the raw-register fallback, `close()`).
- `continuum_learning/teleop.py`:
  - **W / S** = motor 1 winds and motor 3 releases (or the reverse), toward +x / −x; **D / A** = motors 0 / 2, toward +z / −z; **R** straightens; **Q** / Ctrl+C straightens and releases the motors.
  - Opposite cables in pairs; 0.3 rad/s while held (`--speed`; Windows key repeat ≈ 30 events/s, `speed/30` rad per event); each motor clamped to ±`MAX_THETA` (1 rad).
  - Motor currents checked every step: above `MAX_CURRENT` (600 mA, `--max-current`), the step is undone and the key ignored until released.
  - Ramp to straight at start and on exit; status line with the bend, angles and currents.
  - Keys are read with `msvcrt`, so only while that terminal has focus.
  - `--simulate` and `--script "W*30,D*60,R,Q"` for testing.
- README: a "Steer by eye" section.

**Tests (made-up robot, scripted keys):** W ×40 → bend x 0.40; D ×80 → z 0.80; S ×400 → x clamps at −1.00; R → straight; Q → released. Over-current at 150 mA: the bend stops at 0.49 rad, the warning prints once and the step is undone. The pure functions (`applyKey`, `anglesFromBend`, `bendFromAngles`) are right. The real terminal key reader loads. Not yet run on the real robot.
- **`teleop.py` limit and speed (2026-10-26, user's request):** angle limit **±2.5 rad** (`MAX_THETA` in `teleop.py`, `--max-theta`) and speed **0.5 rad/s** (`SPEED`, `--speed`), for steering by eye only. Data collection and the model tools keep `config.MAX_THETA` = 1 rad. The start-up message shows the limit and speed. Simulated test: W held → 2.50 (clamped), D ×60 → 1.00, S → −2.50. The 600 mA current check still applies; at large angles motor 0 reached ~605–620 mA during the earlier 2.5 rad collection, so steps may be undone near full bend.
- **`teleop.py` current limit (2026-10-26, user's request):** raised from 600 to **900 mA** (`MAX_CURRENT` in `teleop.py`, `--max-current`); the XM430-W210 stalls at about 1.3 A. Data collection keeps `config.MAX_CURRENT` = 600 mA. The start-up message shows it. Simulated check: the limit is still enforced (`--max-current 400` → step undone with a warning).
- **`teleop.py` current limit raised again (user's request):** **1200 mA**, just under the XM430-W210 stall current (about 1300 mA); a higher value would never trigger. Data collection stays at 600 mA.
