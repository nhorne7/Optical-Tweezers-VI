import Hardware_Interfacing.BaslerCameraControl
import Hardware_Interfacing.BaslerCameraControl.BaslerCameraInterface
import Hardware_Interfacing.KSC101Control.KSC101Interface
import Hardware_Interfacing.WebCameraControl.WebcamInterface
import Hardware_Interfacing.LedD1BDriverControl.ArduinoLEDInterface
import VisualInterface_Frontend.WindowInterface
import Control_Flow.EventHandling
import cv2

if __name__ == "__main__":
    print("//  KSC101 VISUAL SHUTTER CONTROLLER (BASLER CAMERA)  //")
    
    # Create Instances of each Controller & Device
    KSC101 = Hardware_Interfacing.KSC101Control.KSC101Interface.CreateDevice("68801184")
    KSC101Controller = Hardware_Interfacing.KSC101Control.KSC101Interface.KSC101Controller(KSC101)
    camera = Hardware_Interfacing.BaslerCameraControl.BaslerCameraInterface.BaslerCameraController()
    window = VisualInterface_Frontend.WindowInterface.WindowController("NH's Particle Detector", 1)
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
    event_handler = Control_Flow.EventHandling.EventHandler(window, camera, KSC101Controller, ledController) # No raman measurement func added here, so defaults to no control over Isoplane
    
    
    # Main GUI Loop
    try:
        while True:
            ret, frame = camera.read() # Grab a frame from the microscope
            if not ret:
                print("Failed to capture frame.")
                break

            # EVENT HANDLING AND ANNOTAIONS
            frame, particle_in_box = window.annotateCircleDetect(frame) # This line displays circle detection labels on screen, and a boolean as to if a particle is in detection box.
            frame = window.annotateOverlay(frame) # This line annotates all text about modes, particles detected, bounding boxes, etc.
            if event_handler.auto_mode_enabled:
                event_handler.process_auto_mode(particle_in_box, verbose=False) # This handles the control of the autoshutter feature (automatic measurement pipeline). All it needs is to know if a particle is in the bounding box location.
            
            # After all annotations, we display the image to the visual interface with all annotations
            window.imshow(frame)

            # INPUT PROCESSING & INTERACTIVITY LOOP
            key = cv2.waitKeyEx(1)
            action = event_handler.handle_key(key) #  This handles actions based on the key seen.
            if action == 'quit':
                break
    
    
    finally: # Ensure all destructors are appropriately called and all memory is released.
        camera.release()
        ledController.release()
        KSC101Controller.release()
        cv2.destroyAllWindows()