# Hardware interfacing modules
import Hardware_Interfacing.BaslerCameraControl
import Hardware_Interfacing.BaslerCameraControl.BaslerCameraInterface
import Hardware_Interfacing.KSC101Control.KSC101Interface
import Hardware_Interfacing.WebCameraControl.WebcamInterface
import Hardware_Interfacing.LedD1BDriverControl.ArduinoLEDInterface

# Spectroscope interfacing import
from  Hardware_Interfacing.IsoPlaneControl.Adv_features import *

# Front End Interfacing
import VisualInterface_Frontend.WindowInterface
import Control_Flow.EventHandling

# Extra modules
import cv2
import time
import os

# Due to the tracking backend, we import multithreading and multiprocessing modules
import threading
from multiprocessing import Process
import Tracking_Backend.TrackingInterface

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
                event_handler.process_auto_mode(particle_in_box, verbose=False) # This handles the control of the autoshutter feature (automatic measurement pipeline)
            
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

def tracking_loop():
    while True:
        time.sleep(1)
        files_detected = TrackingHandler.trackLatest(r"Recordings",r"Tracked_Videos", window)
        if files_detected == False:
            print("Waiting longer for more videos..")
            time.sleep(30)

def take_measurement(frames, data_dir, name, show_plot=True):    
    measurement = spec.grab_ROI(frames)
    measurement_mean = mean_frames(measurement)
    wl = spec.get_wavelength() # Returns array of wavelength
    with open(data_dir+r"\{}.csv".format(name),mode = 'w',newline = '') as csvfile:
        writer=csv.writer(csvfile)
        for k in range(0,len(measurement_mean[0])):
            writer.writerow([wl[k],measurement_mean[0][k]])
    if show_plot:
        plt.plot(wl,measurement_mean[0])
        plt.show()
    else:
        return False


# working directory declaration
path = r"C:\Users\nohor3086\Desktop\Optical Tweezers Automation Project"
os.chdir(path)
print(f"Current working directory: {os.getcwd()}")


if __name__ == "__main__":


    # INITIALIZATION OF DEVICES AND DEVICE CONTROLLER OBJECTS
    KSC101 = Hardware_Interfacing.KSC101Control.KSC101Interface.CreateDevice("68801184")
    KSC101Controller = Hardware_Interfacing.KSC101Control.KSC101Interface.KSC101Controller(KSC101)
    camera = Hardware_Interfacing.WebCameraControl.WebcamInterface.WebcamController()
    window = VisualInterface_Frontend.WindowInterface.WindowController("NH's Particle Detector", 1)
    ledController = Hardware_Interfacing.LedD1BDriverControl.ArduinoLEDInterface.ArduinoLEDController(3)

    # Initialize all controllers objects
    KSC101Controller.initialize()
    ledController.initialize()
    camera.initialize()
    window.initialize()
    # Initialize Spectroscope
    spec = initialize(True)

    # Configure Spectroscope Appropriately
    spec.spec_grating = 600 # Specify the grooves per millimeter of your grating
    spec.spec_unit = 'nm' # Specify the unit desired of the spectrum (wavenumber 'cm-1' vs wavelength 'nm')
    spec.spec_center = 750 # Specify a spectrum center in the appropriate prespecified unit.
    spec.speed = 0 # Specify between 0-2 on how fast the readout will be. Slow is 0.
    spec.gain = 2  # Speciy the gain of the system betweon 0-2. High gain is 2.
    spec.t_exposure = 200 # Exposure passed in milliseconds
    spec.set_params() # send all parameter changes to the spectroscope

    spec.auto_ROI(minwidth=15) # Automatic Region of Interest calibration (For single fiber applications keep vertical small to avoid hot fibers/cosmic noise)
    
    te = spec.auto_exposure(Dyn=0.5) # Automatic Exposure calibration

    spec.acquire_background(te = te[0], N=20) # specify exposure used and N (number of frames captured of background)



    # Create sliders & init. variable declarations for circle detection overlay
    window.createSlider("param2", 100, 20)
    window.createSlider("maxRadius", 100, 40)
    window.createSlider("minRadius",100, 0)


    # Initialize an Event Handler with all Controllers passed.
    event_handler = Control_Flow.EventHandling.EventHandler(window, camera, KSC101Controller, ledController, take_measurement)
    TrackingHandler = Tracking_Backend.TrackingInterface.TrackingHandler(
                                                invert=False,
                                                minmass=10000,
                                                pix_diameter=51,
                                                traj_memory=5,
                                                traj_search_range=10,
                                                stub_traj_length=5,
                                                microns_per_pix=0.07289795,
                                                fps=30)
        


    # MAIN VISUAL INTERFACE AND TRACKING LOOP BELOW 
    print("//  KSC101 VISUAL SHUTTER CONTROLLER (BASLER CAMERA)  //")
    TRACK_THREAD = threading.Thread(target=tracking_loop)
    TRACK_THREAD.start()

    visual_interface_loop()
    

