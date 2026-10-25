import emioapi
from emioapi._logging_config import logger
import tkinter as tk
from tkinter import ttk
import threading
import time
import signal
import traceback
import numpy as np
from enum import Enum
import teleop

MOTOR_CONTROL_AMOUNT = 0.1
MOTOR_MAX_AMOUNT = 4
MOTOR_MIN_AMOUNT = -1
GLOBAL_KEYS = ['w', 'a', 's', 'd']
keys_pressed = [False, False, False, False]
system_exit = False

TASK_SPACE_BOUNDARY = [[-135,110],[-330,0],[-100,100]]
COORDS_ORIGIN = [0,-200,0]
COORDS_ORIGIN_JOINT_POS = [0,0,0,0]

emio_motors = None
emio_camera = None
emio_motors_coords = [0,0,0,0]
current_end_effector_coords = None
prior_end_effector_coords = None
prior_end_effector_velocity = None
root = None

# SIMULTANEOUS CONTROL VERSION

# Move Motors 0 & 1 to +ve value - Moves away from camera (+ve 1st value)
# Move Motors 2 & 3 to +ve value - Moves towards camera (-ve 1st value)
# Motors 1 & 3 / 2 & 4 tgt - Stiffen
# Motors 2 & 3 +ve value - Moves to left of POV camera (-ve 3rd value)
# Motors 1 & 4 +ve value - Moves to right of POV camera (+ve 3rd value)

# Hardcoded navigation mode
# For future use only
# Mode 1 - Calibration
# Mode 5 - Moving to origin
# Mode 10 - Navigating to target (Camera, fallback SOFA)
# Mode 20 - Picking up target (initial)
# Mode 30 - Check if target picked up (Camera)
# Mode 40 - Navigate to dropoff (Camera, fallback SOFA)
# Mode 50 - Dropoff
# Mode 60 - Check dropoff
# Mode 70 - All targets completed, terminate
class RobotMode(Enum):
    CALIBRATING = 1
    AUTO_MOVING_TO_ORIGIN = 5
    AUTO_MOVING_TO_TARGET = 10
    AUTO_PICKING_UP_TARGET = 20
    AUTO_CHECK_TARGET_PICKED_UP = 30
    AUTO_MOVING_TO_DROPOFF = 40
    AUTO_DROPPING_OFF_TARGET = 50
    AUTO_CHECK_TARGET_DROPPED_OFF = 60
    ALL_DONE = 70

navigation_mode = RobotMode.CALIBRATING

text_calibration_status = None
text_home_pos = None
text_home_joints = None
text_error = None

def raise_stop_flag():
    global system_exit
    print("Stop flag raised via button")
    system_exit = True

def set_home_pos():
    global COORDS_ORIGIN
    global current_end_effector_coords
    global text_home_pos
    global COORDS_ORIGIN_JOINT_POS
    global emio_motors_coords
    COORDS_ORIGIN = current_end_effector_coords
    text_home_pos.set("Home position: " + str(COORDS_ORIGIN))
    # FIXME: Set home pos not working
    COORDS_ORIGIN_JOINT_POS = emio_motors.angles
    text_home_joints.set("Home joints: " + str(emio_motors.angles))

def return_home_direct():
    global emio_motors
    global COORDS_ORIGIN_JOINT_POS
    emio_motors.angles = COORDS_ORIGIN_JOINT_POS

def key_handler(event):
    global keys_pressed
    #print(event.char, event.keysym, event.keycode)
    print(str(event.char) + " pressed")
    for i in range(0, len(GLOBAL_KEYS)):
        if event.char == GLOBAL_KEYS[i]:
            keys_pressed[i] = True

def signal_handler(sig, frame):
    global system_exit
    print("Stop flag raised via Ctrl-C")
    system_exit = True

def check_exit():
    if system_exit:
        root.quit()
    else:
        root.after(50, check_exit)

# Use intermeans to get foreground / background thresholding
# Threshold lvl = min value diff tolerable by the program
def get_intermeans_intensity(depth_frame, threshold=1):
    prev_means = np.mean(depth_frame)
    logger.info(str(prev_means))

def update():
    global emio_motors
    global emio_camera
    global emio_motors_coords
    global current_end_effector_coords
    global prior_end_effector_coords
    global prior_end_effector_velocity
    global keys_pressed

    try:
        emio_camera.update()
        print("-"*20)
        #logger.info(f"Camera parameters: {emio_camera.parameters}")
        #logger.info(f"Camera show: {emio_camera.show_frames}")
        #logger.info(f"Camera tracking: {emio_camera.track_markers}")
        #logger.info(f"Camera compute point cloud: {emio_camera.compute_point_cloud}")
        #logger.info(f"Camera is running: {emio_camera.is_running}")
        #logger.info(f"Count tracker: {len(emio_camera.trackers_pos)}")
        #logger.info(f"Trackers positions: {emio_camera.trackers_pos}")
        #logger.info(f"Point cloud shape: {emio_camera.point_cloud.shape}")
        #logger.info(f"HSV Frame shape: {emio_camera.hsv_frame.shape}")
        #logger.info(f"Mask Frame shape: {emio_camera.mask_frame.shape}")

        # Filter valid trackers
        valid_trackers = [p for p in emio_camera.trackers_pos if all([
                            p[k] >= TASK_SPACE_BOUNDARY[k][0] and
                            p[k] <= TASK_SPACE_BOUNDARY[k][1] for k in range(0, len(TASK_SPACE_BOUNDARY))])]
        logger.info(f"Current mode: {navigation_mode}")
        logger.info(f"Valid trackers: {valid_trackers}")

        # Segmentation: Depth frame edge detection
        depth_frame = emio_camera.depth_frame
        split_intensity = get_intermeans_intensity(depth_frame)

        # Find new postion of end effector
        if (not valid_trackers) or len(valid_trackers) == 0:
            logger.error("No trackers found, end effector may be obstructed")
        else:
            if not prior_end_effector_coords:
                prior_end_effector_coords = valid_trackers[0].copy()
                current_end_effector_coords = prior_end_effector_coords.copy()
                logger.info("Setting end effector coords to first available valid tracker")
                logger.info(f"{valid_trackers[0]}")
            else:
                tgt = 0
                if prior_end_effector_coords is not None:
                    prior_end_effector_coords = current_end_effector_coords
                current_min_square_error = None
                prior_end_effector_coords_square = sum([i ** 2 for i in prior_end_effector_coords])
                for i in range(0, len(valid_trackers)):
                    current_square = sum([j ** 2 for j in valid_trackers[i]])
                    if current_min_square_error is None or current_min_square_error > current_square:
                        tgt = i
                        current_end_effector_coords = valid_trackers[i]
                        current_min_square_error = current_square
        # Find new velocity of end effector
        prior_end_effector_velocity = [current_end_effector_coords[i] - prior_end_effector_coords[i] for i in range(0, len(prior_end_effector_coords))]

        # Check keyboard input
        if keys_pressed[0]:
            # W pressed
            emio_motors_coords = teleop.on_key_pressed(emio_motors_coords, 0)
        if keys_pressed[1]:
            # A pressed
            emio_motors_coords = teleop.on_key_pressed(emio_motors_coords, 1)
        if keys_pressed[2]:
            # S pressed
            emio_motors_coords = teleop.on_key_pressed(emio_motors_coords, 2)
        if keys_pressed[3]:
            emio_motors_coords = teleop.on_key_pressed(emio_motors_coords, 3)
        keys_pressed = [False, False, False, False]

        # Limit check
        for i in range(0, len(emio_motors_coords)):
            if emio_motors_coords[i] < MOTOR_MIN_AMOUNT:
                emio_motors_coords[i] = MOTOR_MIN_AMOUNT
            if emio_motors_coords[i] > MOTOR_MAX_AMOUNT:
                emio_motors_coords[i] = MOTOR_MAX_AMOUNT

        # Actually run it
        emio_motors.angles = emio_motors_coords
        logger.info(str(emio_motors_coords))
        root.after(10, update)
    except KeyboardInterrupt:
        logger.warn("Keyboard interrupt received.")
        root.after(10, update)

def main():
    global keys_pressed
    global system_exit
    global navigation_mode
    global emio_motors
    global emio_camera
    global emio_motors_coords
    global current_end_effector_coords
    global prior_end_effector_coords
    global prior_end_effector_velocity
    global root
    global text_calibration_status
    global text_home_pos
    global text_error
    global COORDS_ORIGIN
    global COORDS_ORIGIN_JOINT_POS

    # If pass, revert to mode 2
    navigation_mode = RobotMode.CALIBRATING

    try:
        emio_camera = emioapi.EmioCamera(show=True, track_markers=True, compute_point_cloud=True)
        emio_camera.fps = 30 # sets the fps to 30. Default is 60 and can only be one of 30. 60 or 90fps
        emio_camera.depth_max = 600 # sets the maximum depth to 600mm. Default is 430mm
        emio_camera.depth_min = 0 # sets the minimum depth to 0mm. Default is 2mm
        emio_motors = emioapi.EmioMotors()

        if not emio_camera.open():
            raise RuntimeError("Unable to open EMIO camera")
        if not emio_motors.open():
            raise RuntimeError("Unable to open EMIO motors")

        root = emio_camera._camera.rootWindow
        # Create an additional window
        control_window = tk.Toplevel(root)
        control_window.title("ME4242 Navigation")
        control_window.geometry("400x400")

        ttk.Label(
            control_window,
            text="ME4242 Navigation Project",
            font=("Arial", 16)
        ).pack(pady=20)

        ttk.Button(
            control_window,
            text="Set Home",
            command=set_home_pos
        ).pack(pady=10)

        ttk.Button(
            control_window,
            text="Return Home",
            command=return_home_direct
        ).pack(pady=10)

        text_calibration_status = tk.StringVar(value="Calibration status: " + str(navigation_mode))
        text_home_pos = tk.StringVar(value="Home position: " + str(COORDS_ORIGIN))
        text_home_joints = tk.StringVar(value="Home joints: " + str(COORDS_ORIGIN_JOINT_POS))
        text_error = tk.StringVar(value="Error: -")

        ttk.Label(
            control_window,
            textvariable=text_calibration_status,
            font=("Arial", 16)
        ).pack(pady=10)

        ttk.Label(
            control_window,
            textvariable=text_home_pos,
            font=("Arial", 16)
        ).pack(pady=10)

        ttk.Label(
            control_window,
            textvariable=text_home_joints,
            font=("Arial", 16)
        ).pack(pady=10)

        ttk.Label(
            control_window,
            textvariable=text_error,
            font=("Arial", 16),
            foreground="red"
        ).pack(pady=10)

        ttk.Button(
            control_window,
            text="Stop",
            command=raise_stop_flag
        ).pack(pady=10)

        # Bind all keys
        for i in range(0, len(GLOBAL_KEYS)):
            control_window.bind(GLOBAL_KEYS[i], key_handler)

        root.after(10, update)
        root.after(50, check_exit)
        root.mainloop()

    except Exception as e:
        print(traceback.format_exc())
        logger.exception(f"An error occurred: {e}")
    # Close GUI window and motors
    if emio_motors:
        emio_motors.close()
    if emio_camera:
        emio_camera.close()
    raise SystemExit

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()

