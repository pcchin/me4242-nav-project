# Learned inverse kinematics for the continuum robot

Move the tip of the real robot to a chosen position, without knowing the material parameters. The robot's behaviour is learned from data instead:

1. Drive the 4 motors through many angle combinations.
2. Measure where the tip goes with Emio's depth camera.
3. Learn the **forward model**: 4 motor angles → tip position (x, y, z).
4. Find the angles for a target by **optimisation** on that model.

The forward model is learned rather than the inverse because the 4 angles are redundant: many angle combinations reach the same position, and a model trained directly on position → angles would average incompatible answers. Among all the solutions, the optimisation picks the one closest to rest, with no opposite cables wound against each other.

## Files

| File | What it does |
|---|---|
| `config.py` | Settings: spool radius, motor directions, limits, safety. **Check before collecting.** |
| `robot_io.py` | Access to the real robot (motors and camera), a made-up robot for testing, and safe ramped motion |
| `collect_data.py` | Collects samples (angles → tip position) into `data/samples.csv` |
| `train_model.py` | Trains the forward model and saves `models/model.npz` and a report plot |
| `ik.py` | Loads the model and solves the inverse kinematics. Needs only numpy and scipy, so it also runs in SOFA |
| `goto.py` | Moves the real robot to a position and measures the error |
| `plot_model.py` | Plots the data with the trained model: effect of each motor, predicted vs measured, workspace map |
| `teleop.py` | Steer the real robot by eye with W/A/S/D from the terminal: no camera, model or SOFA needed |
| `clean_data.py` | Writes a cleaned copy of a data file: only good samples, without the readings of a stuck tracker or out of the robot's reach |
| `calibrate_sim.py` | Fits the SOFA simulation to the data (each motor's slack and gain, cable angles, robot length and base), so the simulated tip goes where the real one went for the same angles. Saves `models/sim_calibration.json`, which `continuum.py` loads automatically |

## Steer by eye (no data needed)

Close Emio Labs / SOFA, then (in the emio-ml environment, from this folder):

```powershell
python teleop.py --port COM12
```

With this terminal focused, hold **W / S** to bend toward +x / -x, **D / A** toward +z / -z (0.5 rad/s, `--speed` to change), **R** to straighten, **Q** to quit (the robot straightens and the motors are released). Opposite cables work in pairs, each motor stays within ±2.5 rad (`--max-theta`), and a move that makes a motor draw more than 600 mA is undone.

## Setup (once)

```powershell
conda create -n emio-ml python=3.10
conda activate emio-ml
pip install emioapi scikit-learn pandas joblib matplotlib
```

`pip install emioapi` gives version 0.2.0, which can't read the motor currents. The over-current check is then off, with a warning. Use **0.3.0**, the version bundled with Emio Labs, which also matches its camera calibration. While Emio Labs is running, copy it from the app's temporary folder:

```powershell
pip uninstall -y emioapi
$src = (Get-ChildItem $env:TEMP -Directory | Where-Object { Test-Path "$($_.FullName)\resources\sofa\bin\python\Lib\site-packages\emioapi" } | Select-Object -First 1).FullName + "\resources\sofa\bin\python\Lib\site-packages"
$dst = python -c "import site; print(site.getsitepackages()[1])"
foreach ($d in "emioapi","emioapi-0.3.0.dist-info","dynamixelmotorsapi","dynamixelmotorsapi-0.1.0.dist-info","dynamixel_easy_sdk") { Copy-Item -Recurse -Force "$src\$d" "$dst\$d" }
```

## Workflow

Run everything from this folder, in the `emio-ml` environment, **with Emio Labs/SOFA closed**. They would hold the motors' port.

### 1. Prepare the robot

- **Calibrate the camera:** in Emio Labs, run the camera calibration module (Sandbox page). Marker positions are then given in the scene frame, in mm.
- **Put one coloured marker on the tip.** The marker is the "end effector" you will control.
- **Mount the cables taut with all the motors at 0 rad.** `python goto.py 0 nan 0 --dry-run` should print angles close to 0.
- **Check `MOTOR_DIRECTIONS`** in `config.py`. With the robot connected, a positive angle on motor *i* times its direction must **wind** cable *i*.

### 2. Collect data

The motors are limited to **1 rad** (`MAX_PULL = 20` mm in `config.py`). Start the first collection small, with a new file:

```powershell
python collect_data.py --n 200 --out data/run1.csv
python clean_data.py data/run1.csv
```

`clean_data.py` writes `data/run1_clean.csv`: only good samples, without stuck-tracker or out-of-reach readings. For more data, collect again with **another seed and a new file** (e.g. `--seed 1 --out data/run2.csv`) and clean it; train on all the clean files together.

Each `--out` file holds one plan (seed, n, max angle); the script refuses to mix plans in one file. Watch for `no_marker` (marker hidden at large bends), `tracking_lost` (the tracker stopped on a stray blob and re-finds the tip at the next rest pose) and `overcurrent`.

Notes:

- **Safety:** angles are limited to ±`MAX_THETA`, and the length wound on both cables of an opposite pair is limited to `MAX_COCONTRACTION`. A move stops and backs off if a motor draws more than `MAX_CURRENT`.
- **Stopping and resuming:** Ctrl+C stops cleanly and releases the motors. Run the **same command** to resume where it stopped. The sample plan depends on `--seed`, `--n` and `--max-theta`.

### 3. Train

```powershell
python train_model.py
```

It prints:
- **Repeatability:** the same angles measured twice. This is the floor for the accuracy you can hope for.
- **Forward model test error:** a Gaussian process and an MLP are compared, and the better one is kept.
- **Whether y can be controlled** on its own, or only follows x and z.
- **IK error** on test positions.

It saves `models/model.npz` and a report plot (`models/model_report.png`).

### 4. Go to a position

```powershell
python goto.py 40 nan 30            # x y z in mm; nan = y free
python goto.py 40 -180 30           # also target y, if the model says it's controllable
python goto.py 40 nan 30 --dry-run  # only print the angles
```

`goto.py` prints the angles, the predicted position, and the **measured** error.

### 5. In SOFA

On the Sandbox page, use the **learned IK** button, or run `continuum.py --model continuum_learning/models/model.npz`.
- Set the target in the **My Robot** window (Target group).
- The 4 motor angles are solved and sent to the robot when it's connected.
- y follows x and z unless `controlY` is ticked on the `LearnedIKController`.

## Testing without hardware

`--simulate` replaces the robot with a made-up one (constant curvature, cables only pull, 0.3 mm camera noise):

```powershell
python collect_data.py --simulate --n 800 --out data/sim.csv
python train_model.py --data data/sim.csv --out models/sim.npz
python goto.py 40 nan 30 --simulate --model models/sim.npz
```

Expected: forward test error ≈ 0.2 mm, and `goto.py` error < 0.1 mm.

## Accuracy

The main error sources on the real robot are:
- **Hysteresis and friction** in the cables, so the result depends on where the robot comes from. The repeats in the data measure this.
- **Camera noise and calibration.**
- **Slow drift:** cable creep and motor temperature.

If the error is too large, the next step is closed-loop correction with the camera, as in `lab_closedloop`.
