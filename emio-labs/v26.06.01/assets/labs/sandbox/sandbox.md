# Sandbox

::: highlight
##### Overview

Set up Emio exactly how you want with this sandbox page. 
Here, you can configure Emio with the legs and connector of your choice.

Also by running this simulation, you can quickly iterate and optimize your own designs without needing physical hardware, 
making this an ideal space for rapid prototyping.
:::

#include(assets/labs/sandbox/sections/1_originalset.md)
#include(assets/labs/sandbox/sections/2_yourownset.md)
#include(assets/labs/modules/camera_calibration.md)
#include(assets/labs/modules/beam_resampler.md)

## Continuum Robot

Emio as a tendon-driven continuum robot: each motor winds a cable on a spool, and the four cables bend a flexible backbone hanging from the center of the box.
Set the four motor angles (rad) from the Move window. Turn on the robot connection to drive the real motors.
If a simulation calibration exists (`continuum_learning/calibrate_sim.py`), the simulated robot is matched to the real one.

#runsofa-button(file="assets/labs/sandbox/continuum.py")

#open-button(file="assets/labs/sandbox/continuum.py")

Steer the robot directly with the keyboard, by eye (no camera or trained model needed). With the SOFA window active, hold **W / S** to bend toward +x / -x, **D / A** toward +z / -z, **R** to straighten:

#runsofa-button(file="assets/labs/sandbox/continuum.py", pyargs=["--inputs", "keys"])

Drive the robot with the pulled length of each cable (mm) instead, opposite cables working in pairs:

#runsofa-button(file="assets/labs/sandbox/continuum.py", pyargs=["--inputs", "pulls"])

Go to a target tip position with the learned inverse kinematics. Train the model first (see `continuum_learning/README.md`); move the target (red) with the keyboard while the SOFA window is active (hold **W/S** to move along x, **A/D** along z, **R** to reset) or with the sliders in the My Robot window, Target group (x and z; y follows). The green dot shows where the real tip goes, the blue points the reachable workspace:

#runsofa-button(file="assets/labs/sandbox/continuum.py", pyargs=["--model", "continuum_learning/models/model.npz"])

#open-button(file="assets/labs/sandbox/continuum_learning/README.md")

Test the continuum robot alone, without Emio (the cables are pulled one after the other):

#runsofa-button(file="assets/emio/parts/continuumrobot.py")