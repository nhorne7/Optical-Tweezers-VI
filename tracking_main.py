import Hardware_Interfacing.BaslerCameraControl
import Hardware_Interfacing.BaslerCameraControl.BaslerCameraInterface
import Hardware_Interfacing.KSC101Control.KSC101Interface
import Hardware_Interfacing.WebCameraControl.WebcamInterface
import Hardware_Interfacing.LedD1BDriverControl.ArduinoLEDInterface
import VisualInterface_Frontend.WindowInterface
import Control_Flow.EventHandling
import cv2
import time


# Due to the tracking backend, we import multithreading and multiprocessing modules
import threading
from multiprocessing import Process
import Tracking_Backend.TrackingInterface

    # Create Instances of each Controller & Device
KSC101 = Hardware_Interfacing.KSC101Control.KSC101Interface.CreateDevice("68801184")
KSC101Controller = Hardware_Interfacing.KSC101Control.KSC101Interface.KSC101Controller(KSC101)
camera = Hardware_Interfacing.BaslerCameraControl.BaslerCameraInterface.BaslerCameraController()
window = VisualInterface_Frontend.WindowInterface.WindowController("NH's Particle Detector", 0.5)
ledController = Hardware_Interfacing.LedD1BDriverControl.ArduinoLEDInterface.ArduinoLEDController(3)

# Initialize all controllers objects
KSC101Controller.initialize()
ledController.initialize()
camera.initialize()
window.initialize()
    
# Create sliders & init. variable declarations for circle detection overlay
window.createSlider("param2", 100, 20)
window.createSlider("maxRadius", 100, 40)
window.createSlider("minRadius",100, 0)

# Initialize an Event Handler with all Controllers passed.
event_handler = Control_Flow.EventHandling.EventHandler(window, camera, KSC101Controller, ledController)
TrackingHandler = Tracking_Backend.TrackingInterface.TrackingHandler(
                                              invert=False,
                                              minmass=10000,
                                              pix_diameter=51,
                                              traj_memory=5,
                                              traj_search_range=10,
                                              stub_traj_length=5,
                                              microns_per_pix=0.07289795,
                                              fps=60)
    
def visual_interface_loop(): 
    # Main GUI Loop
    try:
        while True:
            ret, frame = camera.read()
            if not ret:
                print("Failed to capture frame.")
                break

            # EVENT HANDLING AND ANNOTAIONS
            frame, particle_in_box = window.annotateCircleDetect(frame) # This line displays circle detection labels on screen
            frame = window.annotateOverlay(frame) # This line annotates all text
            if event_handler.auto_mode_enabled:
                event_handler.process_auto_mode(particle_in_box, frame, verbose=False) # This handles the control of the autoshutter feature (automatic measurement pipeline)
            
            # After all annotations, we display the image to the visual interface with all annotations
            window.imshow(frame)

            # INPUT PROCESSING & INTERACTIVITY LOOP
            key = cv2.waitKey(1) & 0xFF
            action = event_handler.handle_key(key)
            if action == 'quit':
                break
    finally: # Ensure all destructors are appropriately called and all memory is released.
        camera.release()
        ledController.release()
        KSC101Controller.release()
        cv2.destroyAllWindows()

#def test_function():
#    while True:
#        time.sleep(5)
#        print("[DEBUG] Running Visual Interface Loop")
        

if __name__ == "__main__":
    print("//  KSC101 VISUAL SHUTTER CONTROLLER (BASLER CAMERA)  //")
    VI_THREAD = threading.Thread(target=visual_interface_loop)
    VI_THREAD.start()

    while True:
        time.sleep(1)
        files_detected = TrackingHandler.trackLatest(r"Recordings",r"Tracked_Videos")
        if files_detected == False:
            print("Waiting longer for more videos..")
            time.sleep(30)
    
    

