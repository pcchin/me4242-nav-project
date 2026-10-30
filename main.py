import emioapi
from emioapi._logging_config import logger
from emioapi.emiomotors import EmioMotors
import tkinter as tk
from tkinter import ttk
import signal
import traceback
import numpy as np
from enum import Enum
import teleop
import trajectory
from trajectory import get_trajectory_is_active
from PIL import Image, ImageTk
import robot_io
import csv

GLOBAL_KEYS = ['w', 'a', 's', 'd','q', 'r', '1', '2', '3', '4', '5', '6', '7', '8']  # 1-8: one motor, see teleop.MOTOR_KEYS
keys_pressed = [False] * len(GLOBAL_KEYS)
system_exit = False
fixed_trajectory_in_progress = False
currently_recording = False
current_csv_file = None
current_csv_file_writer = None

TASK_SPACE_BOUNDARY = [[-135,110],[-330,0],[-100,100]]
COORDS_ORIGIN = [0,-200,0]
COORDS_ORIGIN_JOINT_POS = [0,0,0,0]
UNTWIST_ANGLES = 2

GLOBAL_MAX_THETA = teleop.MAX_THETA  # rad, angle limit of each motor for steering by eye (data collection keeps config.MAX_THETA)
SINGLE_TURN_LIMIT = 3.1  # rad: beyond about +-pi, the motors need their extended (multi-turn) position mode

emio_camera = None
emio_motors = None
current_end_effector_coords = None
prior_end_effector_coords = None
prior_end_effector_velocity = None
root = None
custom_image = None
traj_shape_val = None
traj_amplitude_val = None
traj_loops_val = None
traj_fast_return_home_val = None
traj_reverse_val = None
traj_return_when_terminated_val = None

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
text_recording_state = None

def raise_stop_flag():
    global system_exit
    global emio_motors
    print("Stop flag raised via button")
    if fixed_trajectory_in_progress:
        trajectory.terminate_current(emio_motors)
    else:
        system_exit = True

def set_home_pos():
    global COORDS_ORIGIN
    global current_end_effector_coords
    global text_home_pos
    global text_home_joints
    global COORDS_ORIGIN_JOINT_POS
    global fixed_trajectory_in_progress
    if currently_recording:
        print("Currently recording, cannot set home")
        return
    if fixed_trajectory_in_progress:
        print("Currently running fixed trajectory, cannot set home")
        return
    if current_end_effector_coords is None:
        return
    COORDS_ORIGIN = current_end_effector_coords
    text_home_pos.set("Home position: " + str(COORDS_ORIGIN))
    COORDS_ORIGIN_JOINT_POS = emio_motors.angles
    text_home_joints.set("Home joints: " + str(COORDS_ORIGIN_JOINT_POS))
    teleop.set_common_home(COORDS_ORIGIN_JOINT_POS)

def return_home_direct():
    global COORDS_ORIGIN_JOINT_POS
    global fixed_trajectory_in_progress
    global traj_fast_return_home_val
    if fixed_trajectory_in_progress:
        return
    # teleop.set_joint_angles(COORDS_ORIGIN_JOINT_POS, emio_motors)
    trajectory.terminate_current(emio_motors, traj_fast_return_home_val.get())

def key_handler(event):
    global keys_pressed
    global text_last_pressed
    global fixed_trajectory_in_progress
    global emio_motors
    if currently_recording:
        text_last_pressed.set("Recording ongoing")
    if fixed_trajectory_in_progress:
        # Only checked for q key
        if event.char == "q":
            trajectory.terminate_current(emio_motors)
        else:
            text_last_pressed.set("Fixed trajectory ongoing")
        return
    #print(event.char, event.keysym, event.keycode)
    text_last_pressed.set(str(event.char) + " pressed")
    for i in range(0, len(GLOBAL_KEYS)):
        if event.char == GLOBAL_KEYS[i]:
            keys_pressed[i] = True

def goto_trajectory():
    global traj_amplitude_val
    global traj_loops_val
    global traj_shape_val
    global traj_reverse_val
    global traj_return_when_terminated_val
    global emio_motors
    global currently_recording

    if currently_recording:
        print("Recording in progress, cannot start trajectory")
        return

    try:
        amplitude_val = float(traj_amplitude_val.get())
    except Exception as e:
        amplitude_val = 1.0
    try:
        loops_val = int(traj_loops_val.get())
    except Exception as e:
        loops_val = 1
    if traj_shape_val.get() and len(traj_shape_val.get()) > 0:
        trajectory.main(amplitude_val, loops_val, traj_shape_val.get(), traj_reverse_val.get(), traj_return_when_terminated_val.get(), emio_motors)

def recording_toggle():
    global text_recording_state
    global currently_recording
    if currently_recording:
        text_recording_state.set("Start recording")
        currently_recording = stop_recording()
    else:
        text_recording_state.set("Stop recording")
        currently_recording = start_recording()

def signal_handler(sig, frame):
    global system_exit
    print("Stop flag raised via Ctrl-C")
    system_exit = True

def check_exit():
    if system_exit:
        root.quit()
    else:
        root.after(50, check_exit)

# Return True if recording successfully started, False otherwise
def start_recording():
    global current_csv_file
    global current_csv_file_writer
    # Filename check
    filename = traj_shape_val.get()
    if filename in trajectory.SHAPES:
        print("Existing shape used, unable to start")
        return False
    current_csv_file = open(filename + '.csv', 'w', newline='')
    current_csv_file_writer = csv.writer(current_csv_file)
    print("Starting recording to " + filename + ".csv")
    return True

# Return False if recording successfully stopped, True otherwise
def stop_recording():
    global current_csv_file
    global current_csv_file_writer
    if current_csv_file is not None:
        if current_csv_file:
            current_csv_file.close()
        current_csv_file_writer = None
        current_csv_file = None
    print("Recording done")
    return True

# Use intermeans to get foreground / background thresholding
# Threshold lvl = min value diff tolerable by the program
# FIXME: Unused
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

def untwist_at_origin():
    global emio_motors
    # TODO: Complete
    # Pull up by approx 0.9 deg to untangle it
    if fixed_trajectory_in_progress:
        text_last_pressed.set("Fixed trajectory ongoing")
        return
    # This violates bending regulations, do not tell teleop
    trajectory.terminate_current(emio_motors)
    temp_tgt = teleop.get_common_zero() - [UNTWIST_ANGLES, UNTWIST_ANGLES, UNTWIST_ANGLES, UNTWIST_ANGLES]
    robot = robot_io.EmioMotorsOnly()
    robot_io.rampTo(robot, temp_tgt, emio_motors)
    # Uncoil back
    trajectory.terminate_current(emio_motors)

def update():
    global emio_camera
    global emio_motors
    global current_end_effector_coords
    global prior_end_effector_coords
    global prior_end_effector_velocity
    global keys_pressed
    global text_valid_trackers
    global text_error
    global custom_image
    global fixed_trajectory_in_progress
    global current_csv_file_writer
    global currently_recording

    try:
        emio_camera.update()

        fixed_trajectory_in_progress = get_trajectory_is_active()

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
        #(sat_threshold, sat_threshold_frame) = get_intermeans_intensity(emio_camera.hsv_frame[:,:,1])
        #text_intermeans_intensity.set("Saturation threshold: " + str(sat_threshold))
        # Display thresholded frame

        # TODO: Create slam output frame
        if False:
            img = Image.fromarray(slam_output_frame, mode="L")
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
        if (not fixed_trajectory_in_progress) and any(keys_pressed):
            teleop.on_key_pressed([GLOBAL_KEYS[j] for j in range(0, len(GLOBAL_KEYS)) if keys_pressed[j] == True], emio_motors)
        keys_pressed = [False] * len(GLOBAL_KEYS)
        # Motor updates are handled by the teleop side
        trajectory.update(emio_motors)

        if currently_recording and current_csv_file_writer is not None:
            current_csv_file_writer.writerow(emio_motors.angles)

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
    global text_recording_state
    global COORDS_ORIGIN
    global COORDS_ORIGIN_JOINT_POS
    global custom_image
    global traj_shape_val
    global traj_amplitude_val
    global traj_loops_val
    global traj_reverse_val
    global traj_fast_return_home_val
    global traj_return_when_terminated_val
    global emio_motors

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
        control_window.geometry("400x600")

        text_calibration_status = tk.StringVar(value="Calibration status: " + str(navigation_mode))
        text_home_pos = tk.StringVar(value="Home position: " + str(COORDS_ORIGIN))
        text_home_joints = tk.StringVar(value="Home joints: " + str(COORDS_ORIGIN_JOINT_POS))
        text_error = tk.StringVar(value="Error: -")
        text_valid_trackers = tk.StringVar(value="Valid trackers: -")
        text_intermeans_intensity = tk.StringVar(value="Intermeans intensity: ")
        text_last_pressed = tk.StringVar(value="No keys pressed")
        text_recording_state = tk.StringVar(value="Start recording")
        traj_amplitude_val = tk.StringVar(value="1.0")
        traj_loops_val = tk.StringVar(value="1")
        traj_shape_val = tk.StringVar()
        traj_reverse_val = tk.BooleanVar()
        traj_fast_return_home_val = tk.BooleanVar()
        traj_return_when_terminated_val = tk.BooleanVar()

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

        ttk.Checkbutton(
            control_window,
            text="IGNORE SPEED LIMITS",
            variable=traj_fast_return_home_val,
        ).pack(pady=10)

        ttk.Button(
            control_window,
            text="Return Home",
            command=return_home_direct
        ).pack(pady=10)

        #ttk.Label(
        #    control_window,
        #    textvariable=text_calibration_status,
        #    font=("Arial", 16)
        #).pack(pady=10)

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

        #ttk.Label(
        #    control_window,
        #    textvariable=text_intermeans_intensity,
        #    font=("Arial", 16)
        #).pack(pady=10)

        ttk.Label(
            control_window,
            textvariable=text_last_pressed,
            font=("Arial", 16)
        ).pack(pady=10)

        ttk.Combobox(
            control_window,
            textvariable=traj_shape_val,
            values=trajectory.SHAPES
        ).pack()

        ttk.Label(
            control_window,
            text="Amplitude:",
            font=("Arial", 16)
        ).pack(pady=10)

        ttk.Entry(
            control_window,
            textvariable=traj_amplitude_val
        ).pack(pady=10)

        ttk.Label(
            control_window,
            text="Loops:",
            font=("Arial", 16)
        ).pack(pady=10)

        ttk.Entry(
            control_window,
            textvariable=traj_loops_val
        ).pack()

        ttk.Checkbutton(
            control_window,
            text="Reverse playback",
            variable=traj_reverse_val,
        ).pack(pady=10)

        ttk.Checkbutton(
            control_window,
            text="Return home on termination",
            variable=traj_return_when_terminated_val,
        ).pack(pady=10)

        ttk.Button(
            control_window,
            text="Go to trajectory",
            command=goto_trajectory
        ).pack(pady=10)

        ttk.Button(
            control_window,
            textvariable=text_recording_state,
            command=recording_toggle
        ).pack(pady=10)

        #ttk.Button(
        #    control_window,
        #    text="Untwist at origin",
        #    command=untwist_at_origin
        #).pack(pady=10)

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

        # Create SLAM output window
        custom_image_window = tk.Toplevel(root)
        custom_image_window.title("ME4242 SLAM")
        custom_image = tk.Label(custom_image_window)
        custom_image.pack()

        # Init motors
        emio_motors = EmioMotors()
        if emio_motors.findAndOpen(device_name=None, multi_turn=(GLOBAL_MAX_THETA > SINGLE_TURN_LIMIT)) < 0:
            raise RuntimeError(f"Could not connect to the Emio motors. Check the USB, "
                               f"the power supply, and that Emio Labs / other scripts aren't using the port.")

        teleop.init_motors(emio_motors)
        root.after(int(teleop.LOOP_PERIOD * 1000), update)
        root.after(50, check_exit)
        root.mainloop()

    except Exception as e:
        # print(traceback.format_exc())
        logger.exception(f"An error occurred: {e}")
    # Close GUI window and motors
    if emio_motors is not None:
        teleop.shutdown(emio_motors)
    if emio_camera:
        emio_camera.close()
    if emio_motors:
        emio_motors.close()
    raise SystemExit

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
