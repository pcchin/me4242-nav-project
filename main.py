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
from PIL import Image, ImageTk

GLOBAL_KEYS = ['w', 'a', 's', 'd','q', 'r']
keys_pressed = [False, False, False, False, False, False]
system_exit = False

TASK_SPACE_BOUNDARY = [[-135,110],[-330,0],[-100,100]]
COORDS_ORIGIN = [0,-200,0]
COORDS_ORIGIN_JOINT_POS = [0,0,0,0]

emio_camera = None
current_end_effector_coords = None
prior_end_effector_coords = None
prior_end_effector_velocity = None
root = None
custom_image = None

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
text_valid_trackers = None
text_last_pressed = None
text_intermeans_intensity = None

def raise_stop_flag():
    global system_exit
    print("Stop flag raised via button")
    system_exit = True

def set_home_pos():
    global COORDS_ORIGIN
    global current_end_effector_coords
    global text_home_pos
    global text_home_joints
    global COORDS_ORIGIN_JOINT_POS
    if current_end_effector_coords is None:
        return
    COORDS_ORIGIN = current_end_effector_coords
    text_home_pos.set("Home position: " + str(COORDS_ORIGIN))
    COORDS_ORIGIN_JOINT_POS = teleop.get_motor_coords()
    text_home_joints.set("Home joints: " + str(COORDS_ORIGIN_JOINT_POS))

def return_home_direct():
    global COORDS_ORIGIN_JOINT_POS
    teleop.set_joint_angles(COORDS_ORIGIN_JOINT_POS)

def key_handler(event):
    global keys_pressed
    global text_last_pressed
    #print(event.char, event.keysym, event.keycode)
    text_last_pressed.set(str(event.char) + " pressed")
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
def get_intermeans_intensity(tgt_frame, threshold=2, count_limit=10):
    global text_intermeans_intensity
    prev_means = np.mean(tgt_frame)
    while count_limit > 0:
        l_mean = np.mean(tgt_frame[tgt_frame <= prev_means])
        r_mean = np.mean(tgt_frame[tgt_frame > prev_means])
        new_means = 0.5 * (l_mean + r_mean)
        if (abs(new_means - prev_means)) < threshold:
            print(np.mean(tgt_frame > prev_means))
            return (new_means, (tgt_frame > new_means).astype(np.uint8))
        prev_means = new_means
        count_limit = count_limit - 1
    print(np.sum(tgt_frame > prev_means))
    # FIXME: Saturation thresholding gives correct mean but black frame
    return (prev_means, (255 * (tgt_frame > prev_means)).astype(np.uint8))

def update():
    global emio_camera
    global current_end_effector_coords
    global prior_end_effector_coords
    global prior_end_effector_velocity
    global keys_pressed
    global text_valid_trackers
    global text_error
    global custom_image

    try:
        emio_camera.update()
        #print("-"*20)
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
        #logger.info(f"Current mode: {navigation_mode}")
        text_valid_trackers.set(f"Valid trackers: {valid_trackers}")
        text_error.set("Error: -")

        # Segmentation: HSV frame (saturation) edge detection
        (sat_threshold, sat_threshold_frame) = get_intermeans_intensity(emio_camera.hsv_frame[:,:,1])
        text_intermeans_intensity.set("Saturation threshold: " + str(sat_threshold))
        # Display thresholded frame
        img = Image.fromarray(sat_threshold_frame, mode="L")
        photo = ImageTk.PhotoImage(img, master=custom_image)
        custom_image.configure(image=photo)
        custom_image.image = photo

        # Find new postion of end effector
        if (not valid_trackers) or len(valid_trackers) == 0:
            text_error.set("Error: No trackers found, end effector may be obstructed")
        else:
            if not prior_end_effector_coords:
                prior_end_effector_coords = valid_trackers[0]
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
        if prior_end_effector_coords is not None:
            # Find new velocity of end effector
            prior_end_effector_velocity = [current_end_effector_coords[i] - prior_end_effector_coords[i] for i in range(0, len(prior_end_effector_coords))]

        # Check keyboard input
        if any(keys_pressed):
            teleop.on_key_pressed([GLOBAL_KEYS[j] for j in range(0, len(GLOBAL_KEYS)) if keys_pressed[j] == True])
        keys_pressed = [False, False, False, False, False, False]
        # Motor updates are handled by the teleop side

        root.after(10, update)
    except KeyboardInterrupt:
        logger.warn("Keyboard interrupt received.")
        root.after(10, update)

def main():
    global keys_pressed
    global system_exit
    global navigation_mode
    global emio_camera
    global current_end_effector_coords
    global prior_end_effector_coords
    global prior_end_effector_velocity
    global root
    global text_calibration_status
    global text_home_pos
    global text_home_joints
    global text_error
    global text_valid_trackers
    global text_intermeans_intensity
    global text_last_pressed
    global COORDS_ORIGIN
    global COORDS_ORIGIN_JOINT_POS
    global custom_image

    # If pass, revert to mode 2
    navigation_mode = RobotMode.CALIBRATING

    try:
        emio_camera = emioapi.EmioCamera(show=True, track_markers=True, compute_point_cloud=True)
        emio_camera.fps = 30 # sets the fps to 30. Default is 60 and can only be one of 30. 60 or 90fps
        emio_camera.depth_max = 600 # sets the maximum depth to 600mm. Default is 430mm
        emio_camera.depth_min = 0 # sets the minimum depth to 0mm. Default is 2mm

        if not emio_camera.open():
            raise RuntimeError("Unable to open EMIO camera")

        root = emio_camera._camera.rootWindow
        # Create an additional window
        control_window = tk.Toplevel(root)
        control_window.title("ME4242 Navigation")
        control_window.geometry("400x500")

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
        text_valid_trackers = tk.StringVar(value="Valid trackers: -")
        text_intermeans_intensity = tk.StringVar(value="Intermeans intensity: ")
        text_last_pressed = tk.StringVar(value="No keys pressed")

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
            textvariable=text_valid_trackers,
            font=("Arial", 16)
        ).pack(pady=10)

        ttk.Label(
            control_window,
            textvariable=text_intermeans_intensity,
            font=("Arial", 16)
        ).pack(pady=10)

        ttk.Label(
            control_window,
            textvariable=text_last_pressed,
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

        # Create grayscale custom output window
        custom_image_window = tk.Toplevel(root)
        custom_image_window.title("ME4242 Custom Thresholding")
        custom_image = tk.Label(custom_image_window)
        custom_image.pack()

        teleop.init_motors()
        root.after(10, update)
        root.after(50, check_exit)
        root.mainloop()

    except Exception as e:
        print(traceback.format_exc())
        logger.exception(f"An error occurred: {e}")
    # Close GUI window and motors
    teleop.shutdown()
    if emio_camera:
        emio_camera.close()
    raise SystemExit

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()

