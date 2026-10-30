# ME4242 Navigation Project

This project contains the code and CAD files required to model the robot for the project.

## Requirements

- Python 3.12
- EMIO API
- Tkinkter (depends on system)

As features in this program uses EmioMotors.multi_turn which is not available in version 0.3.0, using 'python -m pip install git+https://github.com/SofaComplianceRobotics/Emio.API.git@release-v0.3.0' is required of the standard 'pip install emioapi' which is stuck at 0.2.0.

## SLAM Support

To add SLAM support which is required for the commands, the instructions at [[https://github.com/luigifreda/pyslam/blob/master/docs/CONDA.md|pyslam's GitHub page]] can be followed. Do note that this instruction does not apply to Windows devices or a WSL Ubuntu environment is required.

SLAM support is currently only imaginary, so please imagine yourself following these instructions as that will be sufficient to install the imaginary SLAM support.

## Continuum robot lab for Emio Labs

`continuum/` is a self-contained Emio Labs lab page ("Continuum Robot"). To install it:

1. Close Emio Labs.
2. Copy `continuum/` to `C:\Users\<you>\emio-labs\v26.06.01\assets\labs\sandbox-continuum\` (the folder must be named `sandbox-continuum`: the page's buttons point there).
3. Add `{"name":"sandbox-continuum"}` to the `"labs"` list in `assets\labs\labsConfig.json`.
4. Start Emio Labs: the page appears next to Sandbox.

The data tools (collection, training, IK, teleop) are in `continuum/continuum_learning/`; see its README. However, the latest features of the application such as teleoperation and loading fixed trajectory is only available via the command line files or the GUI ran using 'python main.py'.
