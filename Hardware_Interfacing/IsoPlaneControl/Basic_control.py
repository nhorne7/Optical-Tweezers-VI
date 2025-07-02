# -*- coding: utf-8 -*-
"""
Created on Thu Dec 17 14:05:35 2020

@author: mje059
"""

# This is a scipt module containing the basic functions needed to run the
# IsoPlane SCT320 using Python automation

# Import the .NET class library
# Import the .NET class library
#Imported in Helpers
# ipiimport clr

#Import helper functions
from Helpers import *

clr.AddReference('System')
# Import python sys module
import sys, os

# numpy import
#import numpy as np

import time as t
import datetime as dt

# Import c compatible List and String
from System import *
from System.Collections.Generic import List

# Add needed dll references
sys.path.append(os.environ['LIGHTFIELD_ROOT'])
sys.path.append(os.environ['LIGHTFIELD_ROOT']+"\\AddInViews")
clr.AddReference('PrincetonInstruments.LightFieldViewV5')
clr.AddReference('PrincetonInstruments.LightField.AutomationV5')
clr.AddReference('PrincetonInstruments.LightFieldAddInSupportServices')

# PI imports
from PrincetonInstruments.LightField.Automation import *
from PrincetonInstruments.LightField.AddIns import *

file_dir = os.getcwd()

working_dir = "\\".join(file_dir.split("\\")[0:-2])


#Initializes the automation object with lightfield visible if vis==true
def initialize(vis):
    global auto, app, exp, working_dir
    
    name = "PyAuto"
    
    auto=Automation(vis, List[String]())
    app=auto.LightFieldApplication
    #Main handler used for the setup
    exp=app.Experiment
    
    good = False
    
    saved_experiments = list(exp.GetSavedExperiments())
    
    if name in saved_experiments:
        good = exp.Load(name)
        if not good:
            print("Error reading experiment")
        else:
            spec = IsoPlane(auto,app,exp,working_dir)
            spec.set_params()
            return spec
    else:
        print("PyAuto experiment not found, creating new.")
        exp.SaveAs(name)
        
        spec = IsoPlane(auto,app,exp,working_dir)
        t.sleep(1)
        spec.set_params()
    
    
    return spec

class IsoPlane():
    def __init__(self,auto,app,exp,working_dir):
        
        self.working_dir = working_dir
        self.auto = auto
        self.app = app
        self.exp = exp
        
        #Default settings:
            
        #Common Acquisition Settings
        self.t_exposure = 100.0          #Exposure 100ms
        self.frames = 1                  #Save one frame
        self.t_stamping = [False,False]  #No time stamping
        self.f_track = False             #No frame tracking
        
        #Online Corrections
        self.corr_back = False           #No background correction, done in script
        self.corr_flat = False           #No flatfield, corrected with sensitivity curve
        self.blemish = False             #No blemish correction, done if post if needed
        self.cosmic = False              #No cosmic ray correction, done in post
        
        #Online Processes
        self.exp_per_frame = 1           #Exposures per frame, equivalent done in script
        self.comb_mode = "avg"           #Method of combination if exp_per_frame >1
        self.formula = False
        self.cross_sect = False
        
        #Save Data File
        
        #Save data in a user-specific folder in common folder (under Data)
        user_dir = working_dir+r"\Data\{}".format(os.getlogin())
        
        #Check if user folder exists, create one if not
        if not os.path.isdir(user_dir):
            os.mkdir(user_dir)
        
        #Make new save folder for a new session, labeled with the current date
        date = dt.date.today()
        self.save_dir = user_dir + r"\Session {} ({}-{}-{})".format(len(os.listdir(user_dir))+1,date.day,date.month,date.year)
        
        #Make folder
        if not os.path.isdir(self.save_dir):
            os.mkdir(self.save_dir)
        
        self.prefix = "Data"         #Prefix for each measurement
        self.increment = True        #Increment measurement name by 1, starting with 1
        self.keep_raw = False        #Do not keep raw file
        
        #Export Data
        self.export_csv = True           #Export data as .csv with format: frame|wavelength|intensity
        
        #ADC settings
        self.quality = "low noise"   #Use lowest noise alternative
        self.speed = 0            #Use slow readout, valid 0-2
        self.gain = 2           #Use high gain, valid 0-2
        
        
        #Regions of Interest
        self.custom_ROI = [0,180,1340,40] #Default ROI covering 40 pixels vertical about the center
        
        #Sensor
        self.sensor_temp = -85           #Default temperature of -85C
        
        #Shutter
        self.shuttermode = "normal"      #Defaut shutter to normal mode (open when exposing)
        
        #Spectrometer
        
        self.spec_unit = "cm-1"         #Use relative cm-1 as default unit
        self.laser = 660                 #Use 660nm as laser line
        self.spec_grating = 300          #Use the 300g/mm grating as default
        self.spec_center = 2600          #Set center to 2600cm-1 (796.7166nm)
        self.step_n_glue = False         #Disable step and glue
        
        
        #Experiment data and variables
        self.Bg = 0
        
        #Misc dictionary bufffer
        self.buffer = {}
        
    def set_params(self,holdtime = 0.1):
        
        #Set the values to the experiment
        
        #Set step and glue mode
        self.exp.SetValue(ExperimentSettings.StepAndGlueEnabled,self.step_n_glue)
        t.sleep(holdtime)
        
        #Common Acquision Settings
        
        #Set exposure
        self.exp.SetValue(CameraSettings.ShutterTimingExposureTime,Double(self.t_exposure))
        t.sleep(holdtime)
        
        #Set number of frames
        self.exp.SetValue(ExperimentSettings.AcquisitionFramesToStore,Int64(self.frames))
        t.sleep(holdtime)
        
        #Set time stamping
        self.exp.SetValue(CameraSettings.AcquisitionTimeStampingStamps, TimeStamps(1*self.t_stamping[0]+2*self.t_stamping[1]))
        t.sleep(holdtime)
        
        #Set frame tracking
        self.exp.SetValue(CameraSettings.AcquisitionFrameTrackingEnabled,self.f_track)
        t.sleep(holdtime)
        
        #Online corrections
        
        #Set background correction in software
        self.exp.SetValue(ExperimentSettings.OnlineCorrectionsBackgroundCorrectionEnabled,self.corr_back)
        t.sleep(holdtime)
        
        #Set flatfield correction
        self.exp.SetValue(ExperimentSettings.OnlineCorrectionsFlatfieldCorrectionEnabled,self.corr_flat)
        t.sleep(holdtime)
        
        #Set blemish correction
        self.exp.SetValue(ExperimentSettings.OnlineCorrectionsBlemishCorrectionEnabled,self.blemish)
        t.sleep(holdtime)
        
        #Cosmic ray correction in software
        self.exp.SetValue(ExperimentSettings.OnlineCorrectionsCosmicRayCorrectionEnabled,self.cosmic)
        t.sleep(holdtime)
        
        #Online Processes
        
        #Set number of exposures for frame combination
        self.exp.SetValue(ExperimentSettings.OnlineProcessingFrameCombinationFramesCombined,Int64(self.exp_per_frame))
        
        if self.comb_mode == "avg":
            self.exp.SetValue(ExperimentSettings.OnlineProcessingFrameCombinationMethod,FrameCombinationMethod(2))
        else:
            self.exp.SetValue(ExperimentSettings.OnlineProcessingFrameCombinationMethod,FrameCombinationMethod(1))
        
        self.exp.SetValue(ExperimentSettings.OnlineProcessingFormulasEnabled,self.formula)
        
        self.exp.SetValue(ExperimentSettings.OnlineProcessingCrossSectionEnabled,self.cross_sect)
        
        #Save Data File
        
        #Set save directory
        self.exp.SetValue(ExperimentSettings.FileNameGenerationDirectory,self.save_dir)
        t.sleep(holdtime)
                
        #Set file name
        self.exp.SetValue(ExperimentSettings.FileNameGenerationBaseFileName,self.prefix)
        t.sleep(holdtime)
            
        #Set increment file name flag
        self.exp.SetValue(ExperimentSettings.FileNameGenerationAttachIncrement,self.increment)
        t.sleep(holdtime)
            
        #If increment flag is raised, reset counter to 1
        if self.increment:
            self.exp.SetValue(ExperimentSettings.FileNameGenerationIncrementNumber,Int32(1))
            t.sleep(holdtime)
            
        #Flag for keeping the raw data
        self.exp.SetValue(ExperimentSettings.FileNameGenerationSaveRawData,self.keep_raw)
        t.sleep(holdtime)
            
        
        self.exp.SetValue(ExperimentSettings.OnlineExportEnabled,self.export_csv)
        t.sleep(holdtime)
        
        if self.export_csv:
            
            self.exp.SetValue(ExperimentSettings.OnlineExportEnabled,True)
            t.sleep(holdtime)
            
            #Set output format to .csv
            self.exp.SetValue(ExperimentSettings.OnlineExportFormat,ExportFileType.Csv)
            t.sleep(holdtime)
            
            #Specify one file per ROI
            self.exp.SetValue(ExperimentSettings.OnlineExportOutputOptionsDataPartition,ExportOutputMode.OneFilePerRoi)
            t.sleep(holdtime)
            
            #Use standard global number formatting
            self.exp.SetValue(ExperimentSettings.OnlineExportCsvFormatOptionsNumberFormatSelected,CsvNumberFormat.Global)
            t.sleep(holdtime)
            
            #Set delimiter to ','
            self.exp.SetValue(ExperimentSettings.OnlineExportCsvFormatOptionsFieldSeparatorSelected,CsvFieldSeparator.Auto)
            t.sleep(holdtime)
            
            #Set layout to table format
            self.exp.SetValue(ExperimentSettings.OnlineExportCsvFormatOptionsDataLayout,CsvLayout.Table)
            t.sleep(holdtime)
            
            #Configure table columns
            table_format = List[CsvTableFormat](3)
            table_format.Add(CsvTableFormat.Frame)
            table_format.Add(CsvTableFormat.Wavelength)
            table_format.Add(CsvTableFormat.Intensity)
            self.exp.SetValue(ExperimentSettings.OnlineExportCsvFormatOptionsTableColumns,table_format)
            t.sleep(holdtime)
            

            #Force no header on .csv file
            self.exp.SetValue(ExperimentSettings.OnlineExportCsvFormatOptionsHeaderFormat,CsvExportHeader(1))
            t.sleep(holdtime)
            
            #Set output units to nm
            self.exp.SetValue(ExperimentSettings.OnlineExportCsvFormatOptionsUnitsWavelength,LightUnit.Nanometers)
            t.sleep(holdtime)
            
            #Export files to same folder as .spe source files
            self.exp.SetValue(ExperimentSettings.OnlineExportOutputOptionsLocation,ExportOutputPathOption.InputPath)
            t.sleep(holdtime)
            
            #Keep original files as well as .csv
            self.exp.SetValue(ExperimentSettings.OnlineExportOutputOptionsExportedFilesOnly,False)
            t.sleep(holdtime)
            
        #ADC settings
        
        if self.quality == "low noise":
            #Set to low noise mode
            self.exp.SetValue(CameraSettings.AdcQuality,AdcQuality.LowNoise)
            t.sleep(holdtime)
            
            #Possible readout speeds
            speeds = [0.1,1,5]
            #Set Readout speed
            self.exp.SetValue(CameraSettings.AdcSpeed,Double(speeds[self.speed]))
            t.sleep(holdtime)
            
            #Set ADC gain
            self.exp.SetValue(CameraSettings.AdcAnalogGain,AdcGain(self.gain+1))
            t.sleep(holdtime)
            
        else:
            #Else, use high speed mode
            self.exp.SetValue(CameraSettings.AdcQuality,AdcQuality.HighSpeed)
            t.sleep(holdtime)
            
            speeds = [6.25,8.33,10]
            self.exp.SetValue(CameraSettings.AdcSpeed,Double(speeds[self.speed]))
            t.sleep(holdtime)
            
        
        #Use both ports for readout
        self.exp.SetValue(CameraSettings.ReadoutControlPortsUsed, Int32(2))
        t.sleep(holdtime)
        
        #Force full frame readout, may be changed in the future
        self.exp.SetValue(CameraSettings.ReadoutControlMode, ReadoutControlMode(1))
        t.sleep(holdtime)
        
        #Set default ROI
        self.set_ROI([self.custom_ROI])
        t.sleep(holdtime)
        
        #Set default CCD temperature
        self.exp.SetValue(CameraSettings.SensorTemperatureSetPoint,Double(self.sensor_temp))
        t.sleep(holdtime)
        
        #Shutter
        self.set_shuttermode(self.shuttermode)
        t.sleep(holdtime)
        
        
        
        #Spectrometer
        
        #Set grating to be used
        self.set_grating(self.spec_grating)
        t.sleep(holdtime)
        
        #Center the grating
        self.set_center(self.spec_center)
        t.sleep(holdtime)
        
    def reset(self):
        
        self.working_dir = working_dir
        self.auto = auto
        self.app = app
        self.exp = exp
        
        #Default settings:
            
        #Common Acquisition Settings
        self.t_exposure = 100.0          #Exposure 100ms
        self.frames = 1                  #Save one frame
        self.t_stamping = [False,False]  #No time stamping
        self.f_track = False             #No frame tracking
        
        #Online Corrections
        self.corr_back = False           #No background correction, done in script
        self.corr_flat = False           #No flatfield, corrected with sensitivity curve
        self.blemish = False             #No blemish correction, done if post if needed
        self.cosmic = False              #No cosmic ray correction, done in post
        
        #Online Processes
        self.exp_per_frame = 1           #Exposures per frame, equivalent done in script
        self.comb_mode = "avg"           #Method of combination if exp_per_frame >1
        self.formula = False
        self.cross_sect = False
        
        #Save Data File
        
        #Save data in a user-specific folder in common folder (under Data)
        user_dir = working_dir+r"\Data\{}".format(os.getlogin())
        
        #Check if user folder exists, create one if not
        if not os.path.isdir(user_dir):
            os.mkdir(user_dir)
        
        #Make new save folder for a new session, labeled with the current date
        date = dt.date.today()
        self.save_dir = user_dir + r"\Session {} ({}-{}-{})".format(len(os.listdir(user_dir))+1,date.day,date.month,date.year)
        
        #Make folder
        if not os.path.isdir(self.save_dir):
            os.mkdir(self.save_dir)
        
        self.prefix = "Data"         #Prefix for each measurement
        self.increment = True        #Increment measurement name by 1, starting with 1
        self.keep_raw = False        #Do not keep raw file
        
        #Export Data
        self.export_csv = True           #Export data as .csv with format: frame|wavelength|intensity
        
        #ADC settings
        self.quality = "low noise"   #Use lowest noise alternative
        self.speed = 0            #Use slow readout, valid 0-2
        self.gain = 2           #Use high gain, valid 0-2
        
        
        #Regions of Interest
        self.custom_ROI = [0,180,1340,40] #Default ROI covering 40 pixels vertical about the center
        
        #Sensor
        self.sensor_temp = -85           #Default temperature of -85C
        
        #Shutter
        self.shuttermode = "normal"      #Defaut shutter to normal mode (open when exposing)
        
        #Spectrometer
        
        self.spec_unit = "cm-1"         #Use relative cm-1 as default unit
        self.laser = 660                 #Use 660nm as laser line
        self.spec_grating = 300          #Use the 300g/mm grating as default
        self.spec_center = 2600          #Set center to 2600cm-1 (796.7166nm)
        self.step_n_glue = False         #Disable step and glue
        
        
    #Closes the automation instance
    def destruct(self):
        self.auto.Dispose()


    #Set the exposure time of the camera in ms
    def set_exposure(self,te):
        te=Double(np.round(float(te),4))
        if self.exp.Exists(CameraSettings.ShutterTimingExposureTime):
            self.exp.SetValue(CameraSettings.ShutterTimingExposureTime,te)
        else:
            return False
    #Gets current exposure time in ms
    def get_exposure(self):
        if self.exp.Exists(CameraSettings.ShutterTimingExposureTime):
            te=self.exp.GetValue(CameraSettings.ShutterTimingExposureTime)
            return te
        else:
            return False

    #Set grating center wavelength
    def set_center(self,center):
        
        #If units are set to relative wavenumber, convert to nm for setting the spectrometer
        if self.spec_unit == "cm-1":
            center = sh_to_nm(center,self.laser)
            
        #Check that the value exists to catch errors
        if self.exp.Exists(SpectrometerSettings.GratingCenterWavelength):
            self.exp.SetValue(SpectrometerSettings.GratingCenterWavelength,Double(center))
        else:
            return False

    #Get current center wavelength
    def get_center(self):
        if self.exp.Exists(SpectrometerSettings.GratingCenterWavelength):
            center=self.exp.GetValue(SpectrometerSettings.GratingCenterWavelength)
            
            if self.spec_unit == "cm-1":
                return nm_to_sh(center, self.laser)
            else:
                return center
        else:
            return False

    #Moves grating so image starts at wavelength given by start in nm
    def set_starting(self,start):
        
        #Convert from cm-1 to nm if cm-1 is set as the experiment unit
        if self.spec_unit == "cm-1":
            start = sh_to_nm(start,self.laser)
            
        if self.exp.Exists(SpectrometerSettings.GratingCenterWavelength):
            of=33.3554*(1-pow((start/2160.9),1.6487));
            g=self.get_grating()
            if g==600:
                of=2*of
            elif g==300:
                of=4*of
            
            self.set_center(start+of)
        else:
            return False
        
    def get_lims(self):
        if self.exp.Exists(SpectrometerSettings.GratingCenterWavelength):
            wl=self.get_wavelength()
        
        if self.spec_unit == "cm-1":
            wl = nm_to_sh(wl,self.laser)
        return [min(wl),max(wl)]

    def set_last(self,stop):
        if self.spec_unit == "cm-1":
            stop = sh_to_nm(stop,self.laser)
        if self.exp.Exists(SpectrometerSettings.GratingCenterWavelength):
            of=33.3554*(1-pow((stop/2125.3),1.5752));
            g=self.get_grating()
            if g==600:
                of=2*of
            elif g==300:
                of=4*of
            
            self.exp.SetValue(SpectrometerSettings.GratingCenterWavelength,stop-of)
        else:
            return False

    #Returns the grating density of currently used grating
    def get_grating(self):
        if self.exp.Exists(SpectrometerSettings.GratingSelected):
            raw=self.exp.GetValue(SpectrometerSettings.GratingSelected)
            if "1200" in raw:
                g=1200
            elif "600" in raw:
                g=600
            elif "300" in raw:
                g=300
            return g
        else:
            return False

    #Set the grating to be used, either 1200, 600, or 300
    def set_grating(self,g):
        if self.exp.Exists(SpectrometerSettings.GratingSelected):
            if g==1200:
                gra='[750nm,1200][0][0]'
            elif g==600:
                gra='[750nm,600][1][0]'
            elif g==300:
                gra='[750nm,300][2][0]'
            raw=self.exp.SetValue(SpectrometerSettings.GratingSelected,gra)
        else:
            return False

    #Gets the current wavelength calibration(image X-axis)
    def get_wavelength(self):
        raw=self.exp.SystemColumnCalibration
        wl=np.copy(list(raw))
        
        if self.spec_unit == "cm-1":
            return nm_to_sh(wl,self.laser)
        else:
            return wl

    #Set behavior of shutter as either static(closed/open) or dynamic(normal)
    def set_shuttermode(self,mode):
        if mode=="closed":
            self.exp.SetValue(CameraSettings.ShutterTimingMode,ShutterTimingMode.AlwaysClosed)
        elif mode=="open":
            self.exp.SetValue(CameraSettings.ShutterTimingMode,ShutterTimingMode.AlwaysOpen)
        elif mode=="normal":
            self.exp.SetValue(CameraSettings.ShutterTimingMode,ShutterTimingMode.Normal)
        else:
            self.exp.SetValue(CameraSettings.ShutterTimingMode,ShutterTimingMode.Normal)
            return False

    #Function for setting regions of interest(ROI)
    #Accepts ROIs of the format: [ROI1,ROI2,ROI3..]
    #ROI format: [X-pos,width,Y-pos,height,enable binning(1/0)]
    def set_ROI(self,ROIs):
        #Takes list of ROIs where each ROI is formatted as [X_start,Y_start,Width,Height]
        nROI=len(ROIs)
        regions = []
        for i in range(0,nROI):
        #Creates new ROI
            regions.append(
                RegionOfInterest
                (ROIs[i][0], ROIs[i][1],
                 ROIs[i][2], ROIs[i][3],
                 1,ROIs[i][3]))
       
        #Set ROIs on CCD
        self.exp.SetCustomRegions(regions)

    def grab_ROI(self,frames):
        Raw=self.exp.Capture(int(frames))
        
        Images=img_array(Raw)
        
        #Check if CCD background is declared
        if type(self.Bg) == int:
            #If not, create zero background
            self.Bg = np.zeros(Images[0].shape)
        
    
        if type(self.Bg) == np.ndarray:
            #Check that the shape of the background matches the image shape
            if np.mean(self.Bg.shape==Images[0].shape)<1:
                #If not, create zero background with same shape
                self.Bg = np.zeros(Images[0].shape)
        
        return Images.astype(np.float64)-self.Bg

    def acquire_background(self,te = 0,N=100):
        
        if te == 0:
            te = self.get_exposure()
        
        #Close shutter
        self.set_shuttermode('closed')
        #Set exposure
        self.set_exposure(te)
        #Reset background
        self.Bg = 0
        
        #Check signal on CCD
        o=self.grab_ROI(N)
        
        #Return shutter to normal mode
        self.set_shuttermode('normal')
        
        #Compute mean frames of backgrounds
        self.Bg=mean_frames(o)
        #return self.Bg

    def get_ROIsize(self):
        te=self.get_exposure()
        self.set_shuttermode("closed")
        self.set_exposure(0.0)
        Sam=self.exp.Capture(1)
        self.set_shuttermode("normal")
        self.set_exposure(te)
        Frame=Sam.GetFrame(0,0)
        [width,height]=[Frame.Width,Frame.Height]
        return (height,width)

    def auto_ROI(self,minwidth = 10,te_min = 1, te_max = 30000):
        
        te = self.get_exposure()

        self.exp.SetFullSensorRegion()

        a = np.zeros([400,1340])+2**16

        while np.max(a) > 0.9*2**16:
            if te>= te_max:
                print("Too low signal")
                break
            elif te<= te_min:
                print("Too high signal")
                break
            a = self.grab_ROI(1)[0]
            
            if np.max(a) >= 0.9*2**16 or np.max(a)< 2**14:
                te *= 2**15/np.max(a)
            else:
                break
            self.set_exposure(np.round(te,4))
            
        row = np.max(a,axis = 1)

        mid = np.argmax(row)

        width = np.sum(row>np.mean(row))

        if width < minwidth:
            width = minwidth
            
        X_start = 0
        Y_start = int(mid -width//2)
        Width = 1340
        Height = int(width)
        self.set_ROI([[X_start,Y_start,Width,Height]])
        
        
    def get_temp_setpoint(self):
        if self.exp.Exists(CameraSettings.SensorTemperatureSetPoint):
            return self.exp.GetValue(CameraSettings.SensorTemperatureSetPoint)
        else:
            return False
        
    def set_temp(self,temp):
        if self.exp.Exists(CameraSettings.SensorTemperatureSetPoint):
            self.exp.SetValue(CameraSettings.SensorTemperatureSetPoint,temp)
            while self.get_temp()!=temp:
                print("Current temperature deviates from setpoint (dT: {})".format(np.round(self.get_temp()-temp,1)))
                t.sleep(1)
        else:
            return False
        
    def get_temp(self):
        if self.exp.Exists(CameraSettings.SensorTemperatureReading):
            return self.exp.GetValue(CameraSettings.SensorTemperatureReading)
        else:
            return False
        
    def auto_exposure(self,te_init = 0,Dyn = 0.7,Ntest = 1,tol=0.1,minlim = 0.5, maxlim = 30000):
        #Function that adjusts exposure time to reach specific dynamic range use
        #Optional arguments:
            #Dyn (float 0-1): amount of dynamic range on the CCD to be used
            #Ntest (int 1-inf): number of frames to test
            #tol (float 0-1): tolerated deviance from Dyn in fraction
            #minlim (float 0-inf): minimum exposure time allowed
            #maxlim (float 0-inf): maximum exposure time allowed
        print("auto_exposure start")
        if te_init == 0:
            #Check current exposure
            te = self.get_exposure()
        else:
            te = te_init
            
        
        if te>maxlim:
            print("Initial exposure too high")
            self.set_exposure(maxlim)
        elif te<minlim:
            print("Initial exposure too low")
            self.set_exposure(minlim)
        else:
            print("Initial exposure set")
            self.set_exposure(te)
        
        #Take test-measurements
        print("measuring first")
        meas = self.grab_ROI(Ntest)
        print("begin loop")
        #Loop while dynamic range usage deviates from specification
        while np.abs(np.max(meas)-Dyn*2**16)>tol*2**16:
            print("loop continues, dynamic :{}, exposure {}".format(np.max(meas)/2**16,te))
            #Handler for too strong signal -> exposure below minlim
            if te <= minlim and np.max(meas) > (Dyn+tol)*2**16:
                print("Signal too strong!")
                return [minlim,np.max(meas)/2**16]
            #Handler for too weak signal -> exposure above maxlim
            elif te >= maxlim and np.max(meas) < (Dyn-tol)*2**16:
                print("Signal too weak!")
                return [maxlim,np.max(meas)/2**16]
            
            #Correct exposure by relative deviance for asymptotic approach
            if np.max(meas) > 100:
                te *= Dyn*2**16/np.max(meas)
                print("Adjusting exposure by factor {} ({}ms)".format(Dyn*2**16/np.max(meas), te))
     
            else:
                te *= 100
                print("Adjusting exposure by factor {} ({}ms)".format(10, te))
     
                
            #te = te_factor*te
            #print("Adjusting exposure by factor {} ({}ms)".format(te_factor, te))
            
            if te<minlim:
                te = minlim
            #Handler for too weak signal -> exposure above maxlim
            elif te>maxlim:
                te = maxlim
            
            print("Testing again...")
            #Set new exposure and test again
            self.set_exposure(te)
            meas = self.grab_ROI(Ntest)
            
            
            
        
        return [te,np.max(meas)/2**16]
    
