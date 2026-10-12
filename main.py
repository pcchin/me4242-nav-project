import emioapi
from emioapi._logging_config import logger
from pynput import keyboard

MOTOR_CONTROL_AMOUNT = 0.05
MOTOR_MAX_AMOUNT = 2
GLOBAL_KEYS = ['q', 'w', 'e','r', 'a', 's', 'd', 'f']
keys_pressed = [False, False, False, False, False, False, False, False]

def on_press(key, injected):
    global keys_pressed
    try:
        for i in range(0, len(GLOBAL_KEYS)):
            if key.char == GLOBAL_KEYS[i]:
                keys_pressed[i] = True
    except AttributeError:
        print('special key {} pressed'.format(key))

def on_release(key, injected):
    global keys_pressed
    for i in range(0, len(GLOBAL_KEYS)):
        if key.char == GLOBAL_KEYS[i]:
            keys_pressed[i] = False

def main():
    global keys_pressed

    emio_motors = None
    emio_camera = None
    emio_motors_coords = [0,0,0,0]

    try:
        emio_motors = emioapi.EmioMotors()
        emio_camera = emioapi.EmioCamera(show=True, track_markers=True, compute_point_cloud=True)
        emio_camera.fps = 30 # sets the fps to 30. Default is 60 and can only be one of 30. 60 or 90fps
        emio_camera.depth_max = 600 # sets the maximum depth to 600mm. Default is 430mm
        emio_camera.depth_min = 0 # sets the minimum depth to 0mm. Default is 2mm

        if not emio_camera.open():
            raise RuntimeException("Unable to open EMIO camera")
        if not emio_motors.open():
            raise RuntimeException("Unable to open EMIO motors")

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
                # Limit check
                for i in range(0, len(emio_motors_coords)):
                    if emio_motors_coords[i] < 0:
                        emio_motors_coords[i] = 0
                    if emio_motors_coords[i] > MOTOR_MAX_AMOUNT:
                        emio_motors_coords[i] = MOTOR_MAX_AMOUNT
                # Actually run it
                emio_motors.angles = emio_motors_coords
            except KeyboardInterrupt:
                logger.info("Keyboard interrupt received.")
                break
    except Exception as e:
        logger.exception(f"An error occurred: {e}")
        if emio_motors:
            emio_motors.close()
        if emio_camera:
            emio_camera.close()

if __name__ == "__main__":
    # Start keyboard listener
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    main()

