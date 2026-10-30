import emioapi
from emioapi._logging_config import logger
import tkinter
import threading
import time
import signal
import traceback

MOTOR_CONTROL_AMOUNT = 0.05
MOTOR_MAX_AMOUNT = 3
MOTOR_MIN_AMOUNT = -1
GLOBAL_KEYS = ['q', 'w', 'e','r', 'a', 's', 'd', 'f']
keys_pressed = [False, False, False, False, False, False, False, False]
system_exit = False

def raise_stop_flag():
    global system_exit
    print("Stop flag raised via button")
    system_exit = True

def key_handler(event):
    global keys_pressed
    print(event.char, event.keysym, event.keycode)
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

def main():
    global keys_pressed
    global system_exit

    emio_motors = None
    emio_camera = None
    emio_motors_coords = [0,0,0,0]

    try:
        emio_camera = emioapi.EmioCamera(show=False, track_markers=True, compute_point_cloud=True)
        emio_camera.fps = 30 # sets the fps to 30. Default is 60 and can only be one of 30. 60 or 90fps
        emio_camera.depth_max = 600 # sets the maximum depth to 600mm. Default is 430mm
        emio_camera.depth_min = 0 # sets the minimum depth to 0mm. Default is 2mm
        emio_motors = emioapi.EmioMotors()

        if not emio_camera.open():
            raise RuntimeError("Unable to open EMIO camera")
        if not emio_motors.open():
            raise RuntimeError("Unable to open EMIO motors")

        while emio_camera.is_running:
            try:
                emio_camera.update()
                print("-"*20)
                logger.info(f"Camera parameters: {emio_camera.parameters}")
                logger.info(f"Camera show: {emio_camera.show_frames}")
                logger.info(f"Camera tracking: {emio_camera.track_markers}")
                logger.info(f"Camera compute point cloud: {emio_camera.compute_point_cloud}")
                logger.info(f"Camera is running: {emio_camera.is_running}")
                logger.info(f"Count tracker: {len(emio_camera.trackers_pos)}")
                logger.info(f"Trackers positions: {emio_camera.trackers_pos}")
                logger.info(f"Point cloud shape: {emio_camera.point_cloud.shape}")
                logger.info(f"HSV Frame shape: {emio_camera.hsv_frame.shape}")
                logger.info(f"Mask Frame shape: {emio_camera.mask_frame.shape}")

                # Check keyboard input
                if keys_pressed[0]:
                    emio_motors_coords[0] += MOTOR_CONTROL_AMOUNT
                if keys_pressed[1]:
                    emio_motors_coords[1] += MOTOR_CONTROL_AMOUNT
                if keys_pressed[2]:
                    emio_motors_coords[2] += MOTOR_CONTROL_AMOUNT
                if keys_pressed[3]:
                    emio_motors_coords[3] += MOTOR_CONTROL_AMOUNT
                if keys_pressed[4]:
                    emio_motors_coords[0] -= MOTOR_CONTROL_AMOUNT
                if keys_pressed[5]:
                    emio_motors_coords[1] -= MOTOR_CONTROL_AMOUNT
                if keys_pressed[6]:
                    emio_motors_coords[2] -= MOTOR_CONTROL_AMOUNT
                if keys_pressed[7]:
                    emio_motors_coords[3] -= MOTOR_CONTROL_AMOUNT
                keys_pressed = [False, False, False, False, False, False, False, False]
                # Limit check
                for i in range(0, len(emio_motors_coords)):
                    if emio_motors_coords[i] < MOTOR_MIN_AMOUNT:
                        emio_motors_coords[i] = MOTOR_MIN_AMOUNT
                    if emio_motors_coords[i] > MOTOR_MAX_AMOUNT:
                        emio_motors_coords[i] = MOTOR_MAX_AMOUNT
                # Actually run it
                emio_motors.angles = emio_motors_coords
                logger.info(str(emio_motors_coords))
                if system_exit:
                    break
            except KeyboardInterrupt:
                logger.info("Keyboard interrupt received.")
                break
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

    # Start GUI window
    root = tkinter.Tk()
    root.title("ME4242 Navigation Project")
    tkinter.Button(root, text="Stop", command=raise_stop_flag).pack()

    # Bind all keys
    for i in range(0, len(GLOBAL_KEYS)):
        root.bind(GLOBAL_KEYS[i], key_handler)

    thread = threading.Thread(target=main, daemon=True)
    thread.start()
    root.after(50, check_exit)
    root.mainloop()
    root.destroy()

