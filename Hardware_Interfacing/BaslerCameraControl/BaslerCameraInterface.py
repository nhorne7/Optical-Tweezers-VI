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

            self.camera.StartGrabbing()
            self.converter = pylon.ImageFormatConverter()
            print("Converter Object Created")
            self.converter.OutputPixelFormat = pylon.PixelType_RGB8packed
            self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
            print("Basler Camera is ON")
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

    # def changeExposureTime(self, exposure_time_us=30000):
    #     self.camera.ExposureTimeAbs.SetValue(exposure_time_us)

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

    def setExposureTime(self, exposure_time):
        self.camera.ExposureTime.Value = exposure_time
        if self.camera.ExposureTime.Value != exposure_time:
            print("Could not modify exposure time of BASLER camera!")
        else:
            return True