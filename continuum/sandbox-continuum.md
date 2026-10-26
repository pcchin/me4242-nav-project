# Continuum Robot

::: highlight
##### Overview

Set up Emio exactly how you want with this sandbox page. 
Here, you can configure Emio with the legs and connector of your choice.

Also by running this simulation, you can quickly iterate and optimize your own designs without needing physical hardware, 
making this an ideal space for rapid prototyping.
:::

## Continuum Robot

Emio as a tendon-driven continuum robot: each motor winds a cable on a spool, and the four cables bend a flexible backbone hanging from the center of the box.
Set the four motor angles (rad) from the Move window. Turn on the robot connection to drive the real motors.
If a simulation calibration exists (`continuum_learning/calibrate_sim.py`), the simulated robot is matched to the real one.

#runsofa-button(file="assets/labs/sandbox-continuum/continuum.py")

#open-button(file="assets/labs/sandbox-continuum/continuum.py")

Drive the robot with the pulled length of each cable (mm) instead, opposite cables working in pairs:

#runsofa-button(file="assets/labs/sandbox-continuum/continuum.py", pyargs=["--inputs", "pulls"])

Go to a target tip position with the learned inverse kinematics. Train the model first (see `continuum_learning/README.md`); move the target (red) with the keyboard while the SOFA window is active (hold **W/S** to move along x, **A/D** along z, **R** to reset) or with the sliders in the My Robot window, Target group (x and z; y follows). The green dot shows where the real tip goes, the blue points the reachable workspace:

#runsofa-button(file="assets/labs/sandbox-continuum/continuum.py", pyargs=["--model", "continuum_learning/models/model.npz"])

#open-button(file="assets/labs/sandbox-continuum/continuum_learning/README.md")

Test the continuum robot alone, without Emio (the cables are pulled one after the other):

#runsofa-button(file="assets/emio/parts/continuumrobot.py")
