# ME4242 Navigation Project

This project contains the code and CAD files required to model the robot for the project.

## Requirements

- Python 3.12
- EMIO API (pip install emioapi)
- Tkinkter (depends on system)

## Continuum robot lab for Emio Labs

`continuum/` is a self-contained Emio Labs lab page ("Continuum Robot"). To install it:

1. Close Emio Labs.
2. Copy `continuum/` to `C:\Users\<you>\emio-labs\v26.06.01\assets\labs\sandbox-continuum\` (the folder must be named `sandbox-continuum`: the page's buttons point there).
3. Add `{"name":"sandbox-continuum"}` to the `"labs"` list in `assets\labs\labsConfig.json`.
4. Start Emio Labs: the page appears next to Sandbox.

The data tools (collection, training, IK, teleop) are in `continuum/continuum_learning/`; see its README.
