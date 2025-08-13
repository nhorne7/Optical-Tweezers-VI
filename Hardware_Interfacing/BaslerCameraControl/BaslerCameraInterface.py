from pypylon import pylon
import os
import time
import datetime
import cv2

class BaslerCameraController:
    def __init__(self):
        print("Basler Camera Controller Object created")
        self.is_recording = False
        self.ret = False
        self.raw_frame = None
        self.vidout = None
        self.camera = None
        self.converter = None
        self.exposure_time = 5674 # Default Exposure time (ms)

    def initialize(self):
        print("Initializing Basler Camera..")
        try:
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            self.camera.Open()

            # Set Timed exposure mode and disable auto exposure
            # self.camera.ExposureMode.SetValue("Timed")
            # if pylon.IsAvailable(self.camera.ExposureAuto):
            #     self.camera.ExposureAuto.SetValue("Off") # this will allow manual control of the exposure for the camera.
            # Check that the above code does not break the camera controller :)

            self.camera.Width.Value = self.camera.Width.Max  # Set the maximum possible resolution for the BASLER camera
            self.camera.Height.Value = self.camera.Height.Max

            self.camera.ExposureTime.SetValue(self.exposure_time) # Initialize default Exposure Time

            self.camera.StartGrabbing()
            self.converter = pylon.ImageFormatConverter()
            print("Converter Object Created")
            self.converter.OutputPixelFormat = pylon.PixelType_RGB8packed
            self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
            print(f"Basler Camera is ON at Exposure Time (ms): {self.camera.ExposureTime.Value}")
        except Exception as e:
            print(f"Failed to initialize camera: {e}")
            raise

    def release(self):
        if self.is_recording:
            self.stopRecording()
        if self.camera is not None and self.camera.IsGrabbing():
            self.camera.StopGrabbing()
        if self.camera is not None:
            self.camera.Close()
            self.camera = None
        print("Basler Camera is OFF")

    def read(self):
        try:
            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grabResult.GrabSucceeded():
                image = self.converter.Convert(grabResult)
                self.raw_frame = cv2.cvtColor(image.GetArray(), cv2.COLOR_RGB2BGR)
                self.raw_frame_copy = self.raw_frame.copy()
                self.ret = True
            else:
                print('Frame Grab Failed..')
                self.ret = False
        finally:
            grabResult.Release() # Prevent buffer overflow

        if self.is_recording: # Write video to recording if recording
            self.vidout.write(self.raw_frame_copy)
        return self.ret, self.raw_frame

    def startRecording(self, fps=30):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        root = os.getcwd()
        os.makedirs(os.path.join(root, "Recordings"), exist_ok=True)
        out_path = os.path.join("Recordings", f"recording_{timestamp}.avi")
        if self.raw_frame is None:
            raise RuntimeError("Call read() once before starting recording!!")
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        self.vidout = cv2.VideoWriter(out_path, fourcc, fps, (self.raw_frame.shape[1], self.raw_frame.shape[0]))
        if not self.vidout.isOpened():
            raise RuntimeError("Failed to open video writer :(")
        self.is_recording = True
        print(f"Recording started at {out_path}")

    def stopRecording(self):
        if self.vidout:
            self.vidout.release()
            self.vidout = None
        self.is_recording = False
        print("Recording stopped.")

    def screengrab(self):
        root = os.getcwd()
        screenshot_dir = os.path.join(root, "Screenshots")
        os.makedirs(screenshot_dir, exist_ok=True)  # Creates folder if it doesn't exist
        filename = f'img_{datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.jpg'
        outPath = os.path.join(screenshot_dir, filename)
        cv2.imwrite(outPath, self.raw_frame_copy)
        print(f"Screenshot Saved to: {outPath}")

    def setExposureTime(self, exposure_time, verbose=False):
        self.camera.ExposureTime.SetValue(exposure_time)
        if self.camera.Exposuretime.Value != exposure_time:
            print("Could not modify exposure time of BASLER camera!")
            return False
        else:
            self.exposure_time = exposure_time
            print(f"Exposure Time (ms) = {exposure_time}") if verbose else ...
            return True
        
    def getExposureTime(self):
        return(self.camera.ExposureTime.Value)
    
    def setAcquisitionRate(self, rate, verbose = False):
        self.camera.AcquisitionFrameRateEnable.Value = True
        self.camera.AcquisitionFrameRate.Value = rate
        if self.camera.AcquisitionFrameRate.Value != rate:
            print("Acquisition Rate could not be set!")
            return False
        else:
            print(f"Acquisition Rate set to ({rate}) FPS") if verbose else ...
            return True
    
    def getAcquisitionRate(self):
        return(self.camera.ResultingFrameRate.Value)

        