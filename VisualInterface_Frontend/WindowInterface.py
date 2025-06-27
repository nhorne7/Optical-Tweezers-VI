import cv2
import numpy as np
import time



class WindowController:
    def __init__(self, window_name="Visual Interface", scale_factor=1):
        print("Window Controller Object created")
        self.window_name = window_name
        self.scale_factor = scale_factor

        self.crosshair = False
        self.slider_values = {}

        self.param2 = 25
        self.minRadius = 0
        self.maxRadius = 40

        self.editing = False
        self.dragging = False
        self.bbox_start = (100,100)
        self.bbox_end = (200,200)
        self.is_recording = False
        self.auto_shutter = False
        self.particle_trapped = False


        self.font = cv2.FONT_HERSHEY_SIMPLEX

    def initialize(self):
        print(f"WindowController '{self.window_name}' is ONLINE")
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

    def imshow(self, frame):
        frame = cv2.resize(frame, (0,0), fx=self.scale_factor, fy=self.scale_factor)
        cv2.imshow(self.window_name, frame)

    def dummy_callback(self, arg):
        pass

    def createSlider(self, name, max_val, init_val=20):
        print(f"Slider '{name}' created")
        self.slider_values[name] = init_val
        cv2.createTrackbar(name, self.window_name, init_val, max_val, self.dummy_callback)

    def getSliderValue(self, name):
        val = cv2.getTrackbarPos(name, self.window_name)
        self.slider_values[name] = val
        return val
    
    def annotateCircleDetect(self, frame):
        eps = 0.01
        self.param2 = self.getSliderValue("param2")+eps
        self.minRadius = self.getSliderValue("minRadius")
        self.maxRadius = self.getSliderValue("maxRadius")
        if len(frame.shape) == 2: # if already GRAY, saave BGR copy to draw circles.
            frame = cv2.cvtColor(frame,cv2.COLOR_GRAY2RGB)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # save GRAY copy as 'gray_frame'
        gray_blurred = cv2.GaussianBlur(gray_frame, (5,5), sigmaX=2, sigmaY=2)
        # Apply Hough transform on the blurred image.
        circle_arr = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, 0.8, 30, param1 = 50, param2 = self.param2, minRadius = self.minRadius, maxRadius = self.maxRadius)
        boundsTL = (min(self.bbox_start[0], self.bbox_end[0]), min(self.bbox_start[1], self.bbox_end[1]))
        boundsBR = (max(self.bbox_start[0], self.bbox_end[0]), max(self.bbox_start[1], self.bbox_end[1]))
        # Draw bounding detection box
        cv2.rectangle(frame, boundsTL, boundsBR, (0,255,255), 1)
        particle_in_box_count = 0

        if circle_arr is not None:
            circle_arr = np.uint16(np.around(circle_arr))
            for circle in circle_arr[0, :]:
                a, b, r = circle[0], circle[1], circle[2]
                #  circle boundary
                cv2.circle(frame, (a, b), r, (0, 255, 0), 2)
                # circle (of radius 1) to show center
                cv2.circle(frame, (a, b), 1, (0, 0, 255), 3)
                # Check for circles in bounding box
                particle_in_box = bool(a >= boundsTL[0] and a <= boundsBR[0] and b >= boundsTL[1] and b <= boundsBR[1])
                if particle_in_box:
                    particle_in_box_count += 1
            if particle_in_box_count > 0:
                cv2.rectangle(frame, boundsTL, boundsBR, (255,0,0), 2)
                frame = cv2.putText(frame, f"{particle_in_box_count} Particle(s) in Detection Box", (5,40), self.font, 0.5, (0,255,0))
    
        frame = cv2.putText(frame, f"Detected Particles: {len(circle_arr[0]) if circle_arr is not None else 0}", (5,20), self.font, 0.5, (0 if circle_arr is None else 255,0,255 if circle_arr is None else 0), 1, cv2.LINE_AA)
        return frame, (particle_in_box_count > 0)
    
    def annotateOverlay(self, frame):
        if self.editing:
            frame = cv2.putText(frame, "EDITING", (5,60), self.font, 0.5, (0,0,255))
        if self.crosshair:
            width, height, _ = frame.shape
            center = (width//2, height//2)
            frame = cv2.line(frame, (center[1]-5, center[0]), (center[1]+5, center[0]), (255,255,255), 1)
            frame = cv2.line(frame, (center[1], center[0]-5), (center[1], center[0]+5), (255,255,255), 1)
        if self.is_recording:
            frame = cv2.putText(frame, "RECORDING VIDEO", (frame.shape[1]-150,35), self.font, 0.5, (0,0,255))
        if self.auto_shutter:
            frame = cv2.putText(frame, "AUTO MODE", (5,80), self.font, 0.5, (255,0,255))
        if not self.auto_shutter:
            frame = cv2.putText(frame, "MANUAL MODE", (5,80), self.font, 0.5, (255,0,255))
        if self.particle_trapped:
            cv2.putText(frame, "PARTICLE TRAPPED", (frame.shape[1]-150,15), self.font, 0.5, (0,0,255))
        return frame


    def mouse_callback(self, event, x, y, flags, param):
        if self.editing:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.bbox_start = (x//self.scale_factor,y//self.scale_factor)
                self.dragging = True
            elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
                self.bbox_end = (x//self.scale_factor,y//self.scale_factor)
            elif event == cv2.EVENT_LBUTTONUP:
                self.bbox_end = (x//self.scale_factor,y//self.scale_factor)
                self.dragging = False

    def release(self):
        self.editing = False
        self.dragging = False
        self.bbox_start = (100,100)
        self.bbox_end = (200,200)
        self.is_recording = False
        self.auto_shutter = False
        self.particle_trapped = False
