import threading
import time
import emioapi

# Initialize the EMIO camera
emiocamera = emioapi.EmioCamera()

# Event used to stop the camera thread
stop_event = threading.Event()


def update_camera():
    """Continuously update the camera in a separate thread."""
    while not stop_event.is_set():
        emiocamera.update()


# Create the camera update thread
camera_thread = threading.Thread(
    target=update_camera,
    name="EmioCameraThread",
    daemon=True
)

# Start the thread
camera_thread.start()

try:
    # Main thread remains available for other operations
    while True:
        print("Main thread is running...")
        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopping camera thread...")

finally:
    stop_event.set()
    camera_thread.join(timeout=2)
    print("Program terminated.")
