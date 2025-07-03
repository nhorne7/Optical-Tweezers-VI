""""
This is a minimalistic video recorder with a visual of the camera.

It is used to facilitate 'bug free' video capture without having to worry about any other dependencies outside of the camera.

Simply press 'r' to start and stop recording when wanted. Videos are saved to the "\Recordings\.." directory in the project.
"""


import Hardware_Interfacing.BaslerCameraControl
import Hardware_Interfacing.BaslerCameraControl.BaslerCameraInterface
import Hardware_Interfacing.WebCameraControl.WebcamInterface
import VisualInterface_Frontend.WindowInterface
import cv2

if __name__ == "__main__":
    print("//  KSC101 VISUAL SHUTTER CONTROLLER (BASLER CAMERA)  //")
    
    # Create Instances of each Controller & Device
    camera = Hardware_Interfacing.BaslerCameraControl.BaslerCameraInterface.BaslerCameraController()
    window = VisualInterface_Frontend.WindowInterface.WindowController("Minimal Video Recorder", 0.5)
    
    # Initialize all controllers objects
    camera.initialize()
    window.initialize()
    recording = False
    # Main GUI Loop



    try:
        while True:
            ret, frame = camera.read()
            if not ret:
                print("Failed to capture frame.")
                break

            window.imshow(frame)

            # INPUT PROCESSING & INTERACTIVITY LOOP
            key = cv2.waitKey(1) & 0xFF
            if key == ord("r") and not recording:
                camera.startRecording()
                recording = True
            elif key == ord("r") and recording:
                camera.stopRecording()
                recording = False
    
    
    finally: # Ensure all destructors are appropriately called and all memory is released.
        camera.release()
        cv2.destroyAllWindows()