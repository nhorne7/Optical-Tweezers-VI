# Optical Tweezers Visual Interface

## About the Visual Interface
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
## Installation & Required Dependencies

This project runs natively in **Python 3.12.10**. 

Using this version of Python or a later version is highly recommended to ensure minimal complication with package compatibility.

#### Required PIP Package Dependencies:

- [Numpy](https://pypi.org/project/numpy/) (Used for fast array manipulation––specifically in tracking data management)
- [Matplotlib](https://pypi.org/project/matplotlib/) (Used for generating tracking plots/histograms)
- [Pandas](https://pypi.org/project/pandas/) (Used to manipulate .csv files returned during the video analysis subprocess)
- [Pims](https://pypi.org/project/PIMS/) (PIMS is a Python Image Sequence package allowing for various video/image export formats)
- [TQDM](https://pypi.org/project/tqdm/) (TQDM is a progress bar package, specifically for the tracking progress)
- [Trackpy](https://pypi.org/project/trackpy/) (Trackpy is the backend for all things tracking)
- [PyAV](https://pypi.org/project/av/) (Used for exporting video to smaller formats)
- [Numba](https://pypi.org/project/numba/) (Allows the use of machine-code decorators to accelerate the tracking process)
- [PyPylon](https://pypi.org/project/pypylon/) (PyPylon interfaces with BASLER cameras as a wrapper for the Pylon SDK)
- [OpenCV](https://pypi.org/project/opencv-python/) (OpenCV is the primary backend of the visual interface & image processing)
- [PythonNET](https://pypi.org/project/pythonnet/) (PythonNET allows for interfacing with Thorlabs instruments and the Kinesis SDK)
- [PyFirmata2](https://pypi.org/project/pyFirmata2/) (PyFirmata2 enables serial arduino communication responsible for setup LED control)
- [PySerial](https://pypi.org/project/pyserial/) (PySerial permits USB serial communication necessary for interfacing with an external Arduino)
- [Arduino-Python3](https://pypi.org/project/arduino-python3/) (Further Arduino compatibility package)

#### Kinesis SDK Installation Instructions:

The **Kinesis SDK** is a Software Development Kit created by Thorlabs to allow easy interfacing with Thorlabs components. 

- It can be downloaded from the [Thorlabs](https://www.thorlabs.com/) website under **[Motion Control Software](https://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=Motion_Control)**.

Ensure that the SDK folder (\Thorlabs\Kinesis) is installed in a convenient location. Ensure also that the Kinesis folder contains three folders: 

- Database
- Drivers
- Firmware Update Utility

These folders should be followed by many **.dll** packages.

Ensure that the following **.dll** packages are present in your Kinesis folder:
- _Thorlabs.MotionControl.DeviceManagerCLI.dll_
- _Thorlabs.MotionControl.GenericMotorCLI.dll_
- _Thorlabs.MotionControl.KCube.SolenoidCLI.dll_

To ensure that the program recognizes your Kinesis SDK, navigate to _Hardware_Interfacing\KSC101Control\KSC101Interface.py_ and ensure that the proper path is placed in the **kinesis_path** variable.

```python
import clr
import os
import time
kinesis_path = r"YOUR_PATH_HERE"
...
```

After this the installation of the Kinesis SDK is complete.

#### pyFirmata2 Installation:

**PyFirmata2** is a Python interface for the Firmata protocol. It allows for USB serial communication with the Arduino through Python. 

As this project deals with an Arduino for setup LED control, it is necessary to perform an initial configuration of the Arduino such that it will work properly.

To successfully configure a new Arduino, perform the following steps:

1. Install the [Arduino IDE](https://www.arduino.cc/en/software/) for your OS.
2. If you are on Windows/Linux also install the [CH340 driver](https://sparks.gogo.co.nz/ch340.html) for proper serial interpretability.
3. Restart your computer to complete installation of the drivers.
4. Open the Arduino IDE.
5. Connect your Arduino Board via USB to your computer.
6. You should now be able to select the Arduino under the _Select Board_ panel. If this does not show up, try using a different USB port, restarting your computer, or reinstalling the IDE and CH340 drivers, ensuring they are compatible with your system version.
7. Once the Arduino IDE recognizes your Arduino Board, ensure proper communication.
8. Open File/Examples/Basics/Blink
9. Load this onto the Arduino. If the onboard LED begins to blink, your Arduino is properly connected.
10. Now under File/Examples, look for a dropdown for **Firmata** and open the **StandardFirmata** sketch. If the dropdown does not appear, navigate under Sketch/Include Library/Manage Libraries, and search for Firmata. Install as usual, and now the Firmata dropdown should show under File/Examples.
11. Load the StandardFirmata sketch onto the Arduino, and close the Arduino IDE.
12. In the project directory, open _Hardware_Interfacing\LedD1BDriverControl\blink.py_ and run the script. The Arduino should now blink, and the setup of pyFirmata2 is complete.

Further StandardFirmata installation resource: [https://roboticsbackend.com/arduino-standard-firmata-tutorial/](https://roboticsbackend.com/arduino-standard-firmata-tutorial/)
    
---

## OOB Usage of Visual Interface

The visual interface supports a multitude of user-convenient features alongside a few interactivity modes.

Upon the initial startup of the visual interface, the user will notice a few on-screen annotations:

// Image to go here// 

- Overlayed detection of particles (green circles with red center)
- A yellow bounding detection box in the top left of the screen
- A green text in the top left stating how many particles are present inside of the yellow bounding box.
- An indication that the visual interface is set to 'MANUAL' mode.
- Two sliders responsible for the control of the detection sensitivity (larger will detect less particles), and the max/min radius of detected particles–in pixels.

#### Important Default Keybinds *(modifiable in EventHandling.py under handle_key method)*:
- **'q'** is used to QUIT the visual interface.
- **'c'** is used to toggle a center crosshair, used for optical alignment/calibration.
- **'s'** allows the user to take an **unannotated full resolution screengrab**, that which the BASLER camera or Webcam currently captures. The resultant screengrab is stored conveniently in the "\Screenshots\\.." folder.
- **'r'** is used to start/stop an **unannotated full resolution screen video capture**, stored in a "\Recordings\\.." directory.
- **'e'** is used to enter EDITING MODE, allowing the adjustment of the yellow detection bounding box using click & drag directly on the screen.
- **'a'** toggles between MANUAL MODE and AUTO MODE modes (more in detail below).
- **'o'** opens and closes the SH05R/M shutter toggleably when interface is in MANUAL MODE.

## AUTO MODE:

AUTO MODE is where the main functionality of this visual interface shines. Once AUTO MODE is enabled, the complete autonomous control of the system will begin, as demonstrated in the control flow diagram below. 

**Ensure that once enabled, the detection bounding box is tightly positioned about the optical trap location, such that it turns blue/detected when a particle is present in the trap.** This can be done manually using EDIT MODE.

![Auto Shutter Control Flow Diagram](Readme_Images/AutoControlFlowchart.jpg)

AUTO SHUTTER mode works to automatically wait for optically trapped particles, turn off onboard LED's to allow the Raman measurements to take place, turn on and off the optical tweezers, and analyze individual particle trajectories which were Raman measured. All of the control flow logic and timing is stored within the **EventHandler** class in the **\Control_Flow\EventHandling.py** file. Here, the member variables controlling the timing of each stage can be updated manually, and extra interfacing can be added at each stage of the measurement process respectively.

```python
...
# The following member variables control the timing of the AUTO MODE pipeline in the EventHandler class.
self.TRAP_CONFIRMATION_TIME = ...
self.MEASUREMENT_TIME = ...
self.DISPERSAL_WAIT_TIME = ...
self.LED_ON_TIME = ...
...
```

**To terminate AUTO MODE, simply press 'a' again to revert to MANUAL MODE**.

---

## Tracking Calibration (trackpycalibrationnotebook.ipynb)

It is likely during change of setup that the tracking pipeline will require recalibration to ensure its accuracy. 

To facilitate this process and the tweaking of parameters, a Jupyter Notebook has been made which allows for users to visualize each step of the tracking process, and dynamically tweak tracking parameters. To begin, upload a raw video captured of the vesicles that you wish to calibrate your tracking pipeline to. In the tracking preprocess there are 5 main calibration steps:

#### 1.  Brightness Thresholding & Mask Calibration

Once the video is uploaded, the first important feature we should look at is the scale and brightness of the particles we wish to detect on the camera. 

The trackpy.locate function takes in 3 main parameters: _diameter, invert, minmass_:
- Diameter: Here we pass in an estimate for the pixel-wise diameter of the particles we wish to track. This tells the tracking backend the scale we should look for.
- Invert: This is a boolean value which would be marked as true for the detection of relatively dark particles, and false if looking for bright spots.
- Minmass: Trackpy denotes the total brightness value of a target as its visual **mass**. To differentiate the darker background to the particles, we set a minimum brightness threshold the particles must achieve. All targets under this threshold are not considered for trajectory analysis, so this is a very important parameter.

For precise and more numerical computations, the Jupyter Notebook features a histogram visualization which aids to determine the proportion of specific brightness levels on the detection algorithms first guess. 

Visual analysis of these thresholding and mask predictions can be performed under 'Cleaning up the Data' on the lines:

```python
f_thresholded = tp.locate(frames[0], diameter=..., invert=..., minmass=...)
tp.annotate(f_thresholded, frames[0])
...
```

After a qualitatively accurate result has been achieved, transfer the chosen parameters to the 'Locate features in all frames', where a multiprocessed batch location algorithm will detect all particle locations for all frames.

```python
frames = list(frames) # load all frames into RAM
f_batch = tp.batch(frames, diameter=..., minmass=..., invert = ...)
```

#### 2.  Trajectory Linkage and Memory Tweaking

The previous f_batch function returns a datafram containing the exact pixel locations of every detected particle for all frames given. However, these positions are not particle-wise labelled and must be linked/enumerated such that each trajectory is a distinguishable characteristic for each particle. This linkage is performed under 'Link the gathered trajectories' in the notebook using the _trackpy.link_ method:

```python
trajectories = tp.link(f_batch, search_range=..., memory=...)
...
```

The two parameters to be calibrated here are the search_range (in pixels) and the frame memory. 

- The **search range** parameter characterizes the maximum pixel distance that a particle can travel in a single frame to still be considered part of the same trajectory (for faster moving particles this should be larger).

- The **memory** parameter controls the maximum number of frames that a particle can not be detected for before the particle is considered lost (for more noisy data this should be higher for longer trajectories).

#### 3.  Ephemeral Trajectory Thresholding

It is likely that there will be some detected noise in trajectories leading to single frame, or perhaps <10 frame trajectories that are unvaluable to the final radii calculations. These ephemeral trajectories can be filtered by setting a minimum frame threshold for trajectories in the **trackpy.filter_stubs** function used in the notebook under 'Filter out stub/ephemeral trajectories'. 

```python
t1 = tp.filter_stubs(trajectories, threshold=...)
```
With noisy data, it may be important to set this value higher.

#### 4.  Further cluster detection and custom Thresholding

In the case where a mixing of different particles is present with slightly different visual characteristics, it is possible to perform custom thresholding through the visualization of specific feature pairs. The trackpy module automatically provides a variety of information about each detected particle, including:

- mass (average brightness)
- size (pixel diameter)
- ecc (eccentricity)
- raw_mass (total brightness

These features can be plotted in pairs, such as (mass/size) or (raw_mass/ecc) to qualitatively discern these two types of particles graphically, which may then be separated from the trajectories dataframe through the use of an inequality. This visualization can be done in the notebook under 'Further filtering through feature detection'. For example, the following code plots the average mass vs average size of each particle across their full trajectories, and displays a plot.

```python
plt.figure()
tp.mass_size(t1.groupby('particle').mean())
...
```

![Feature Detection Example](Readme_Images/featuredetect.png)


#### 5.  Visual Pixel Density / FPS configuration

The last step is the most important for obtaining accurate radii and diffusivity results. The algorithm used must translate pixels to micrometers and frames to seconds in order to correctly estimate radii and diffusivity in SI units. To do this, the $\mu$m/pixel of microscope being used and the FPS of the video used **must be correctly passed to the imsd and emsd functions**.

```python
im = tp.imsd(tm, mpp=..., fps=...)
em = tp.emsd(tm, mpp=..., fps=...)
...
```
The imsd and emsd functions return plot data for the mean squared displacement against lag time for the individual partiles and the ensemble mean, respectively. 

For imsd plots, all trajectories are simultaneously plotted. These are often noisy due to the gaussian noise present in brownian motion.

![Feature Detection Example](Readme_Images/imsd_sample.png)

emsd plots however are quite smooth, as they are constructed as the ensemble mean of all trajectories, thereby mitigating the Gaussian noise. For diffusive particle behavior, this plot will be approximately linear.

![Feature Detection Example](Readme_Images/featuredetect.png)









