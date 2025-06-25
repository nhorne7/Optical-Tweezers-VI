# Optical Tweezers Visual Interface

### About the Visual Interface
This repository features a visual GUI programmed in Python which permits the visual detection of microscale particles and nanoscale diffraction patterns. It's primary backends are supported by:
- [OpenCV](https://docs.opencv.org/4.x/index.html) (for all Image processing tasks)
- [pypylon](https://github.com/basler/pypylon) (for communicating with BASLER brand microscope cameras)
- [pythonnet](https://github.com/pythonnet/pythonnet) (the Python .NET SDK)
- Thorlab's [Kinesis SDK](https://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=Motion_Control) (for Thorlabs component interfacing)
- Arduino's [PyFirmata](https://github.com/berndporr/pyFirmata2) (For Serial Arduino USB Communication)
- various other support packages (numpy, matplotlib, pandas, ...) detailed in the Installation section.

The goal of this project is the autonomous control of the entire Raman Scattering data collection process for nano-scale particles (such as [extracellular vesicles](https://en.wikipedia.org/wiki/Extracellular_vesicle)), while simultaneously conducting Brownian trajectory analysis for individual particles. 

Such trajectory analysis is then later applied to service the calculation of individual particle radii and diffusivity coefficients. These calculations allow for the labelless and efficient classification of particles undergoing Raman analysis.

**Developer's Note**: At the moment, this code has been tested on various microscale level samples with great accuracy in their tracking and radii predictions to theoretical values, however has yet to be tested with visual data from nanoscale diffraction patterns. Such developments are a work in progress once an appropriate nanoscopic visual input method is in place.

---

### Usage of Visual Interface

The visual interface supports a multitude of user-convenient features alongside a few interactivity modes.

Upon the initial startup of the visual interface, the user will notice a few baseline annotations:

// Image to go here// 

- Overlayed detection of particles (green circles with red center)
- A yellow bounding detection box in the top left of the screen
- A green text in the top left stating how many particles are present inside of the yellow bounding box.
- An indication that the visual interface is set to 'MANUAL SHUTTER' mode.
- Two sliders responsible for the control of the detection sensitivity (larger will detect less particles), and the max/min radius of detected particles–in pixels.

#### Important Default Keybinds *(modifiable in EventHandling.py under handle_key method)*:
- **'q'** is used to QUIT the visual interface.
- **'c'** is used to toggle a center crosshair, used for optical alignment/calibration.
- **'s'** allows the user to take an **unannotated full resolution screengrab**, that which the BASLER camera or Webcam currently captures. The resultant screengrab is stored conveniently in the "\Screenshots\\.." folder.
- **'r'** is used to start/stop an **unannotated full resolution screen video capture**, stored in a "\Recordings\\.." directory.
- **'e'** is used to enter EDITING MODE, allowing the adjustment of the yellow detection bounding box using click & drag directly on the screen.
- **'a'** toggles between MANUAL SHUTTER and AUTO SHUTTER modes (more in detail below).
- **'o'** opens and closes the SH05R/M shutter toggleably when interface is in MANUAL MODE.

#### AUTO SHUTTER Mode:

AUTO SHUTTER mode is where the main functionality of this visual interface shines. Once AUTO SHUTTER is enabled, the complete autonomous control of the system will begin, as demonstrated in the control flow diagram below.

AUTO SHUTTER mode works to automatically wait for optically trapped particles, turn off onboard LED's to allow the Raman measurements to take place, turn on and off the optical tweezers, and analyze individual particle trajectories which were Raman measured.

![Auto Shutter Control Flow Diagram](image.jpg)


