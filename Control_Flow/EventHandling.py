import cv2
import time



class EventHandler:
    def __init__(self, window, camera, shutter_controller, led_controller, raman_measurement_func=None):
        self.window = window
        self.camera = camera
        self.shutter = shutter_controller
        self.led = led_controller
        self.raman_meas = raman_measurement_func
        self.frames = 10
        self.data_dir = None
        self.name = "RamanCapture"
        self.show_plot = True

        # Auto shutter mode states
        self.auto_mode_enabled = False
        self.waiting_for_particle = True
        self.waiting_for_measurement = False
        self.waiting_for_dispersal = False

        self.in_trap_start_time = None
        self.measurement_start_time = None
        self.dispersal_start_time = None

        self.shutter_open = True  # Assume starts open
        self.waiting_for_led_recovery = False
        self.led_on_time = None


        self.TRAP_CONFIRMATION_TIME = 2.0  
        self.MEASUREMENT_TIME = 15.0      
        self.DISPERSAL_WAIT_TIME = 10.0     
        self.LED_ON_TIME = 2.0


    def handle_key(self, key):
        if key == ord('q'):
            return 'quit'
        elif key == ord('r'):
            if not self.camera.is_recording:
                self.camera.startRecording()
                self.window.is_recording = True
            else:
                self.camera.stopRecording()
                self.window.is_recording = False
        elif key == ord('e'):
            self.window.editing = not self.window.editing
        elif key == ord('c'):
            self.window.crosshair = not self.window.crosshair
        elif key == ord('a'):
            self.window.auto_shutter = not self.window.auto_shutter
            self.auto_mode_enabled = not self.auto_mode_enabled
            self.waiting_for_dispersal = False
            self.waiting_for_particle = True
            self.in_trap_start_time = None
            self.measurement_start_time = None
            self.dispersal_start_time = None
            self.led_on_time = None
            self.shutter.open()
            print(f"AUTO_SHUTTER = {self.auto_mode_enabled}")

        elif key == ord('s'):
            self.camera.screengrab()
        elif key == ord('o') and not self.window.auto_shutter:
            if self.shutter.is_open:
                self.shutter.close()
            else:
                self.shutter.open()
        return None

    def process_auto_mode(self, particle_in_box: bool, verbose=True):
        current_time = time.time()
        self.window.auto_shutter = True

        particle_detected = particle_in_box

        if self.waiting_for_particle:
            self.led.ledON() # <-- LED controller method
            if particle_detected:
                if self.in_trap_start_time is None:
                    self.in_trap_start_time = current_time
                elif current_time - self.in_trap_start_time >= self.TRAP_CONFIRMATION_TIME:
                    print("Particle is TRAPPED") if verbose else ...
                    self.window.particle_trapped = True
                    self.led.ledOFF()
                    print("Turning LED OFF..") if verbose else ...
                    print("Beginning Raman Measurement..") if verbose else ...
                    self.raman_meas(self.frames, self.data_dir, self.name, self.show_plot) if self.raman_meas != None else ... # CALL RAMAN MEASUREMENT FUNCTION
                    # Start measurement timer
                    self.measurement_start_time = current_time
                    self.waiting_for_particle = False
                    self.waiting_for_measurement = True
            else:
                self.in_trap_start_time = None

        elif self.waiting_for_measurement:
            if current_time - self.measurement_start_time >= self.MEASUREMENT_TIME:
                self.led.ledON()  # Begin LED ON before checking particle
                print("Turning LED ON..") if verbose else ...
                self.led_on_time = current_time  # Start 2-second delay
                self.waiting_for_measurement = False
                self.waiting_for_led_recovery = True

        elif self.waiting_for_led_recovery:
            if current_time - self.led_on_time >= self.LED_ON_TIME:
                if particle_detected:
                    print("Measurement of particle is COMPLETE..") if verbose else ...
                    self.shutter.close()
                    self.window.particle_trapped = False
                    self.shutter_open = False
                    self.dispersal_start_time = current_time
                    self.waiting_for_led_recovery = False
                    self.waiting_for_dispersal = True

                    # START VIDEO RECORDING OF DISPERSAL
                    self.camera.startRecording()
                else:
                    print("Particle has escaped. Restarting.") if verbose else ...
                    self.window.particle_trapped = False
                    self.waiting_for_led_recovery = False
                    self.waiting_for_particle = True
                    self.in_trap_start_time = None

        elif self.waiting_for_dispersal:
            if current_time - self.dispersal_start_time >= self.DISPERSAL_WAIT_TIME:
                print("Dispersal complete. Reopening shutter..") if verbose else ...

                # STOP VIDEO RECORDING
                self.camera.stopRecording()

                self.shutter.open()
                self.shutter_open = True
                self.led.ledON()  # LED on during trap search
                
                # Reset state
                self.waiting_for_dispersal = False
                self.waiting_for_particle = True
                self.in_trap_start_time = None
                self.measurement_start_time = None
                self.dispersal_start_time = None
                self.led_on_time = None
                print("Waiting for New Particle..") if verbose else ...

