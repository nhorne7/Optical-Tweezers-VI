# -*- coding: utf-8 -*-
"""
Created on Fri Dec 18 15:35:49 2020

@author: mje059
"""

from Basic_control import *

#import numpy as np

import csv

from matplotlib import pyplot as plt

def config_sng(spec,start, stop, overlap=10):
    #Function for configuring the steps of a panorama acquisition
    #Take arguments:
        #spec (initialied IsoPlane class from Basic_control): Main interface object
        #start (float 0-x): starting wavelength of the sweep. NB!! if spec.spec_units is set to "cm-1" this will be in cm-1 otherwise in nm
        #stop (float 0-x): ending exposure of the sweep. NB!! if spec.spec_units is set to "cm-1" this will be in cm-1 otherwise in nm
    spec.set_starting(start)
    lims=spec.get_lims()
    steps=[spec.get_center()]
    while lims[1]<stop:
        spec.set_starting(lims[1]-overlap)
        steps.append(spec.get_center())
        lims=spec.get_lims()
        
    spec.set_last(stop)
    steps[-1] = spec.get_center()
    
    #Write the steps to the IsoPlane buffer
    spec.buffer["steps"] = steps
    #return steps

def autoconfigSNG(spec,Dyn = 0.5,tot_time= [0,10,0],n_min = 10,n_max = 200,te_max = 30000,account_for_readout = True):
    #Function for estimating the exposure and repeat exposures for a panorama acquisition
    #Takes arguments:
        #spec (initialized IsoPlane class): Main interface object
    #Optional arguments:
        #Dyn (float 0-1): desired dynamic range usage
        #tot_time (list [h:m:s]): allowed total exposure time
        #n_min (int 0-inf): minimum repeat exposures per step
        #n_max (int 0-inf): maximum repeat exposures per step
        #te_max (float 0-inf): maximum exposure per step
    
    #Check if steps have been defined in the spec-struct
    if not "steps" in list(spec.buffer.keys()):
        #Prompt user if the steps should be defined here
        resp = input("Wavelength steps not set!\n Set now? (y/n)\n")
        
        
        if resp.lower() == "y":
            #If yes, inquire about the start, overlap and stop of the acquisition
            print("NB! spectrometer units set to {}".format(spec.spec_unit))
            resp = input("Specify acquisition range: (start-overlap-stop)\n")
            
            #Catcher for incorrect input
            try:
                #Parse response
                start = float(resp.split("-")[0])
                overlap = float(resp.split("-")[1])
                stop = float(resp.split("-")[2])
                
                #Define steps
                steps = config_sng(spec,start,stop,overlap)
            except:
                #If error, exit and return False
                print("Error parsing response, check input format!")
                return False
        else:
            #If no to define steps, exit and return False
            return False
    else:
        steps = spec.buffer["steps"]
    
    if type(spec.Bg) == int:
        resp = input("Background not measured, measure now? (y/n)")
        
        if resp.lower() == "y":
            print("Acquiring CCD background,stand by.")
            Bg = spec.acquire_background(1000,N = 30)
            print("CCD background acquired")
        else:
            return False
        
    elif type(spec.Bg) == np.ndarray:
        ROI_size = spec.get_ROIsize()
        
        if spec.Bg.shape != ROI_size:
            resp = input("Measured background has incompatible shape, reacquire? (y/n")
            
            if resp.lower() == "y":
                Bg = spec.acquire_background(1000,N = 30)
            else:
                return False
        else:
            Bg = spec.Bg
    
    #tot_time fortmat (list/array) [hours,mins,secs]
    ts=tot_time[0]*3600*1000+tot_time[1]*60*1000+tot_time[2]*1000
    
    tps=ts/len(steps)
    
    teinit=10.0
    
    te=np.asarray([teinit]*len(steps))
            
    SNR_est = [0]*len(steps)
    for i in range(0,len(steps)-1):
        print("Current center: {}\n".format(steps[i]))
        spec.set_center(steps[i])
        
        print("Adjusting exposure")
        [rte,rDyn] = spec.auto_exposure(te_init = te[i],Dyn = Dyn, maxlim = te_max)
        if rte == te_max:
            print("Warning! Max exposure time reached!")
        print("Found exposure time: {}\nDynamic range usage: {}%".format(np.round(rte,1),np.round(rDyn,3)*100))
        
        #Get sample spectrum and wavelength
        nxt = spec.grab_ROI(1)
        wl = spec.get_wavelength()
        
        #Check the trend of the spectrum on the far edge
        spec_trend = np.mean(np.diff(nxt[0][-100:]))/np.mean(np.diff(wl[-100:]))
        
        #Extrapolate the expected signal for the next step
        ex_sig = np.max([(nxt[0][0][-1]+spec_trend*(steps[i+1]-wl[-1]))/2**16,0.01])
        
        SNR_est[i] = SNR_estimate(nxt[0][0])
        
        plt.plot(wl,nxt[0][0]/2**16)
        plt.show()
        
        print("Extrapolated next step signal strength: {}%\n------------------------------".format(np.round(ex_sig,3)*100))
        #Set current exposure
        te[i] = rte
        te[i+1:] = np.max([rte*Dyn/ex_sig,0.5])
    
    print("Current center: {}\n".format(steps[-1]))
    spec.set_center(steps[-1])
    
    print("Adjusting exposure")
    [rte,rDyn] = spec.auto_exposure(te_init = te[-1],Dyn = Dyn, maxlim = te_max)
    if rte == te_max:
        print("Warning! Max exposure time reached!")
    print("Found exposure time: {}\nDynamic range usage: {}%".format(np.round(rte,1),np.round(rDyn,3)*100))
    
    #Get sample spectrum and wavelength
    nxt = spec.grab_ROI(1)
    wl = spec.get_wavelength()
    
    SNR_est[-1] = SNR_estimate(nxt[0][0])
    
    plt.plot(wl,nxt[0][0]/2**16)
    plt.show()
    
    #Set current exposure
    te[-1] = rte
    
    n_balance = np.mean(SNR_est)/np.asarray(SNR_est)
    
    #If the flag for accounting for readout time is raised
    if account_for_readout:
        #Check readout timing
        readout_time = spec.exp.GetValue(CameraSettings.ReadoutControlTime)
        #And account for it in calculation
        k = ts/np.sum(n_balance*(te+readout_time))
    else:
        #Otherwise ignore readout time
        k = ts/np.sum(n_balance*te)
    
    ns = np.where(n_balance*k>=n_min,(n_balance*k).astype(np.uint16),n_min)
    
    spec.buffer["steps_te"] = te
    spec.buffer["steps_n"] = ns
    
    #return np.asarray([te,ns])
            
    

def adv_sng(spec,plot=True,clip_L=0.3,clip_H = 0.05):
    
    if not "steps" in list(spec.buffer.keys()):
        print("Steps not configured!")
        return False
    
    if not "steps_te" in list(spec.buffer.keys()):
        print("Exposure times not configured!")
        return False
    
    if not "steps_n" in list(spec.buffer.keys()):
        print("Repeat exposures not configured!")
        return False
    
    steps = spec.buffer["steps"]
    te = spec.buffer["steps_te"]
    n = spec.buffer["steps_n"]
    
    Data=np.ndarray([2,len(steps),1340])
    
    tstrc = t.localtime()
    
        
    savepath = spec.save_dir+r"\SNG {} ({}-{}-{} {}.{}.{})".format(len(os.listdir(spec.save_dir))+1,
                                                       tstrc.tm_mday,
                                                       tstrc.tm_mon,
                                                       tstrc.tm_year,
                                                       tstrc.tm_hour,
                                                       tstrc.tm_min,
                                                       tstrc.tm_sec)
    
    os.mkdir(savepath)
        
    #Estimated time of completion
    tc = 0
    readout_time = spec.exp.GetValue(CameraSettings.ReadoutControlTime)
    for i in range(0,len(steps)):
        tc += n[i]*(te[i]+readout_time)
    eta=t.localtime(t.time()+tc/1000)
    eta_delta = t.localtime(tc/1000)
    
    if eta_delta.tm_hour == 1:
        print("Expected time for scan: {}m {}s".format(eta_delta.tm_min,
                                                       eta_delta.tm_sec))
    else:
        print("Expected time for scan: {}h {}m {}s".format(eta_delta.tm_hour-1,
                                                           eta_delta.tm_min,
                                                           eta_delta.tm_sec))
    hour = str(eta.tm_hour)
    if len(hour) == 1:
        hour = "0"+hour
    minu = str(eta.tm_min)
    if len(minu) == 1:
        minu = "0"+minu
    sec = str(eta.tm_sec)
    if len(sec) == 1:
        sec = "0"+sec
    
    print("(Time of completion: {}:{}:{})".format(hour,minu,sec))
    y=input("Proceed? (Y/N)")
    if y.lower()=="y":
        for i in range(0,len(steps)):
            print("--------------------")
            print("Acquiring slice {} of {}".format(i+1,len(steps)))
            print("Taking {} exposures with exposure time {}s".format(n[i],np.round(te[i]/1000,1)))
            
            eta_slice = t.localtime(n[i]*(te[i]+readout_time)/1000)
            
            if eta_slice.tm_hour == 1:
                print("Expected time for acquisition: {}m {}s".format(eta_slice.tm_min,
                                                               eta_slice.tm_sec))
            else:
                print("Expected time for acquisition: {}h {}m {}s".format(eta_slice.tm_hour-1,
                                                                   eta_slice.tm_min,
                                                                   eta_slice.tm_sec))
            
            
            
            spec.set_center(steps[i])
            
            
            spec.set_exposure(te[i])
            raw_spect = spec.grab_ROI(n[i])
            
            mean_spect = mean_frames(raw_spect)[0]
            Data[0][i] = mean_spect
            Data[1][i]=spec.get_wavelength()
            
            with open(savepath+r"\{}.csv".format(i),'w',newline='') as csvfile:
                    writer=csv.writer(csvfile)
                    for k in range(0,len(Data[1][i])):
                        writer.writerow([Data[1][i][k],Data[0][i][k]])
            
            
            if plot:
                for k in range(0,i+1):
                    plt.plot(Data[1][k],Data[0][k])
                plt.show()
                
    
        res = overlap(Data,clip_L = clip_L, clip_H = clip_H)
        
        with open(savepath+r"\res.csv",'w',newline='') as csvfile:
                writer=csv.writer(csvfile)
                for k in range(0,len(res[0])):
                    writer.writerow([res[1][k],res[0][k]])
        
        return res
    return False
    
        
        