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
Set the pulled length of each cable from the Move window. Turn on the robot connection to drive the real motors.

#runsofa-button(file="assets/labs/sandbox/continuum.py")

#open-button(file="assets/labs/sandbox/continuum.py")

Test the continuum robot alone, without Emio (the cables are pulled one after the other):

#runsofa-button(file="assets/emio/parts/continuumrobot.py")