import emioapi
from emioapi._logging_config import logger
import tkinter
import threading
import time

MOTOR_CONTROL_AMOUNT = 0.05
MOTOR_MAX_AMOUNT = 2
GLOBAL_KEYS = ['q', 'w', 'e','r', 'a', 's', 'd', 'f']
keys_pressed = [False, False, False, False, False, False, False, False]
system_exit = False
root = None

def raise_stop_flag():
    global system_exit
    system_exit = True
    print("Stop flag called")

def key_handler(event):
    global keys_pressed
    print(event.char, event.keysym, event.keycode)
    for i in range(0, len(GLOBAL_KEYS)):
        if event.char == GLOBAL_KEYS[i]:
            keys_pressed[i] = True

def main():
    global keys_pressed
    global system_exit
    emio_motors_coords = [0,0,0,0]

    try:

        while True:
            try:

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
                    if emio_motors_coords[i] < 0:
                        emio_motors_coords[i] = 0
                    if emio_motors_coords[i] > MOTOR_MAX_AMOUNT:
                        emio_motors_coords[i] = MOTOR_MAX_AMOUNT
                # Actually run it
                print(emio_motors_coords)

                if system_exit:
                    break
            except KeyboardInterrupt:
                logger.info("Keyboard interrupt received.")
                break
    except Exception as e:
        logger.exception(f"An error occurred: {e}")
    # Close GUI window
    if root:
        root.destroy()

if __name__ == "__main__":
    # Start GUI window
    root = tkinter.Tk()
    root.title("ME4242 Navigation Project")
    tkinter.Button(root, text="Stop", command=raise_stop_flag).pack()

    # Bind all keys
    for i in range(0, len(GLOBAL_KEYS)):
        root.bind(GLOBAL_KEYS[i], key_handler)

    thread = threading.Thread(target=main, daemon=True)
    thread.start()
    root.mainloop()

