import cv2
import time
import os
import datetime


class WebcamController:
    def __init__(self, webcam_slot=0):
        print("Webcam Controller Object created")
        self.is_recording = False
        self.ret = False
        self.raw_frame = None
        self.raw_frame_copy = None
        self.vidout = None
        self.cap =None
        self.webcam_slot = webcam_slot
    
    def initialize(self, webcam_slot=0):
        print(f"Initializing Webcam in Slot {webcam_slot}")
        self.cap = cv2.VideoCapture(webcam_slot)
        if not self.cap.isOpened():
            raise RuntimeError("Camera is Busy or Not Available.")
        print("Webcam is ON")

    def release(self):
        if self.cap:
            self.cap.release()
        if self.vidout:
            self.vidout.release()
            self.is_recording = False
        print("Webcamera is OFF")
        
    def read(self):
        self.ret, self.raw_frame = self.cap.read()
        self.raw_frame_copy = self.raw_frame.copy()
        if not self.ret:
            return False, None
        if self.is_recording:
            self.vidout.write(self.raw_frame_copy)
        return self.ret, self.raw_frame
    
    def startRecording(self, fps = 30):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        root = os.getcwd()
        os.makedirs(os.path.join(root, "Recordings"), exist_ok=True)
        out_path = rf"Recordings\recording_{timestamp}.avi"
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
