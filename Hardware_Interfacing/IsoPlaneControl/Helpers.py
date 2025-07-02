# -*- coding: utf-8 -*-
"""
Created on Mon Dec 21 09:55:10 2020

@author: mje059
"""

import clr
#from clr import System

#from System.Runtime.InteropServices import Marshal
from System.Runtime.InteropServices import GCHandle, GCHandleType

import ctypes

import numpy as np

import sys, os



# Add needed dll references
sys.path.append(os.environ['LIGHTFIELD_ROOT'])
sys.path.append(os.environ['LIGHTFIELD_ROOT']+"\\AddInViews")
clr.AddReference('PrincetonInstruments.LightFieldViewV5')
clr.AddReference('PrincetonInstruments.LightField.AutomationV5')
clr.AddReference('PrincetonInstruments.LightFieldAddInSupportServices')

# PI imports
from PrincetonInstruments.LightField.Automation import *
from PrincetonInstruments.LightField.AddIns import *

#Helper function to catch system.Double data from .NET
def img_array(images):
    tmp=images.GetFrame(0,0)
    [Width,Height]=[tmp.Width,tmp.Height]
    image_format=tmp.Format
    
    Output=[0]*images.Frames
    
    for i in range(0,images.Frames):
        tmp=images.GetFrame(0,i).GetData()
        src_hndl = GCHandle.Alloc(tmp, GCHandleType.Pinned)
        try:
            src_ptr = src_hndl.AddrOfPinnedObject().ToInt64()

            # Possible data types returned from acquisition
            if (image_format==ImageDataFormat.MonochromeUnsigned16):
                buf_type = ctypes.c_ushort*len(tmp)
            elif (image_format==ImageDataFormat.MonochromeUnsigned32):
                buf_type = ctypes.c_uint*len(tmp)
            elif (image_format==ImageDataFormat.MonochromeFloating32):
                buf_type = ctypes.c_float*len(tmp)
                    
            cbuf = buf_type.from_address(src_ptr)
            resultArray = np.frombuffer(cbuf, dtype=cbuf._type_)

        # Free the handle 
        finally:        
            if src_hndl.IsAllocated: src_hndl.Free()
        
        Output[i]=np.reshape(np.copy(resultArray),[Height,Width])
        
    # Make a copy of the buffer
    return np.asarray(Output)

def mean_frames(PImages):
    #Determine standard deviation of each pixel
    s=np.std(PImages,0)
    #Determine mean of each pixel
    m=np.mean(PImages,0)
    
    #Filters the input
    
    #Flags pixels that are within 3 sigma of the mean
    I = np.abs(PImages-m)<=3*s
    
    #Keep entries that are flagged, replace the other with nan
    filt = np.where(I,PImages,np.nan)
    
    #Take the mean while ignoring nans in the filtered matrix
    res = np.nanmean(filt,axis = 0)
    
    return res

def overlap(DataSet,clip_L=0.2,clip_H=0.1):
    beta = np.zeros([len(DataSet[0]),2])

    beta[:,0] = 1

    seg = [0]*(2*len(DataSet[0]))

    seg[0] = [DataSet[0][0],DataSet[1][0]]

    for i in range(1,len(DataSet[0])):
        
        #Read preceding segment
        last_wl = seg[2*i-2][1]
        last_I = seg[2*i-2][0]
        
        #Read next segment
        next_wl = DataSet[1][i]
        next_I = DataSet[0][i]
        
        #Find limits of common range
        wl_start = np.min(next_wl)
        wl_stop = np.max(last_wl)
        
        
        #Find all wavelengths in the overlapping sections
        wl_overlap_raw = np.unique(np.append(last_wl,next_wl))
        
        I = (wl_overlap_raw >= wl_start)*(wl_overlap_raw <= wl_stop)
        
        wl_overlap = wl_overlap_raw[I]
        
        #Interpolate overlapping sections to the common range
        last_overlap_interp = np.interp(wl_overlap,last_wl[last_wl>=wl_start],last_I[last_wl>=wl_start])
        next_overlap_interp = np.interp(wl_overlap,next_wl[next_wl<=wl_stop],next_I[next_wl<=wl_stop])
        
        I_valid = np.asarray(range(int(len(wl_overlap)*clip_L),int(len(wl_overlap)*(1-clip_H))))
        
        #last_overlap_smooth = movemean(last_overlap_interp[I_valid],n = 51)
        #next_overlap_smooth = movemean(next_overlap_interp[I_valid],n = 51)
        #Determine derivatives
        #last_overlap_trend = np.cumsum(last_overlap_smooth)/np.cumsum(wl_overlap[I_valid])
        #next_overlap_trend = np.cumsum(next_overlap_smooth)/np.cumsum(wl_overlap[I_valid])
        
        #rdx = np.diff(last_overlap_trend)/np.diff(next_overlap_trend)
        
        
        #Calculate the change in inclination from the gradient
        #beta[i][0] = np.nanmean(rdx[len(rdx)//6:])
        beta[i][0] = np.nansum(last_overlap_interp[I_valid])/np.nansum(next_overlap_interp[I_valid])
        #Use the found incline from gradient to calculate the offset
        beta[i][1] = np.nanmean(last_overlap_interp[I_valid]-beta[i][0]*next_overlap_interp[I_valid])
        
        keep_last = (last_wl<wl_start)
        
        seg[2*i-2] = [last_I[keep_last],last_wl[keep_last]]
        
        overlap_weight = np.linspace(1,0,len(wl_overlap))
        
        overlap_both = np.zeros([2,len(wl_overlap)])
        
        overlap_both[0] = overlap_weight*last_overlap_interp
        overlap_both[1] = (1-overlap_weight)*(beta[i][0]*next_overlap_interp+beta[i][1])
        
        overlap_joined = np.nansum(overlap_both,axis = 0)
        
        seg[2*i-1] = [overlap_joined,wl_overlap]
        
        keep_next = (next_wl>wl_stop)
        
        seg[2*i] = [beta[i][0]*next_I[keep_next]+beta[i][1],next_wl[keep_next]]
        
        
    res = np.zeros([2,1])

    for section in seg:
        if type(section) == int:
            break
        tmp = np.asarray(section)
        res = np.append(res,tmp,axis = 1)
        
    res = res[:,1:]
    
    return res


#Helper to return size of matrix, clone of MatLab size() function
def size(M):
    p=M
    dim=[]
    dim.append(len(M))
    
    while dim[-1]!=0:
        try:
            dim.append(len(M[0]))
        except:
            dim.append(0)
        finally:
            M=M[0]
            if dim[-1]==0:
                dim=dim[0:(len(dim)-1)]
                break
    return dim

def numel(M):
    s=size(M)
    return np.prod(s)

def matmax(M,i):
    flat=np.reshape(M,numel(M))
    ma=max(flat)
    return ma

def var_frames(PImages):
    #Determine standard deviation of each pixel
    s=np.std(PImages,0)[0]
    #Determine mean of each pixel
    m=np.mean(PImages,0)[0]
    
    #Filters the input
    #Uses value if it is within 3 std of the mean, otherwise replaces with mean
    res=[PImages[k][0]*np.array(abs(PImages[k][0]-m)<(3*s)).astype(int)+m*(1-np.array(abs(PImages[k][0]-m)<(3*s)).astype(int)) for k in range(0,len(PImages))]
    
    return np.std(res,0)

def nm_to_sh(nm,lam0=660):
    
    return 10**7*(1/lam0-1/nm)

def sh_to_nm(sh,lam0=660):

    return 1/(1/lam0-10**-7*sh)

def SNR_estimate(data,KL_threshold = 0.05):
    
    balancer = 1
    
    n_movmean = 101
    
    KL = 1.0
    
    while KL >KL_threshold:
        signal_est = movemean(data,n_movmean)
        
        noise_est = data-signal_est
        
        KL = KL_div(noise_est,mu = np.mean(noise_est), sigma = np.std(noise_est))
        
        if KL >KL_threshold:
            n_movmean -=2
        else:
            break
        
        if n_movmean<3:
            n_movmean = 3
            break
    
    SNR_est = np.mean(signal_est/np.std(noise_est))
    
    return SNR_est
    
        
    
def KL_div(data, mu = 0 ,sigma = 1,testpoints = 100):
    
    #Test points
    points = np.linspace(np.min(data),np.max(data),testpoints)
    
    #Evaluation of true gaussian
    pure = np.exp(-(points-mu)**2/(2*sigma**2))/np.sqrt(2*np.pi*sigma**2)
    
    pure /= np.sum(pure)
    
    dx = points[1]-points[0]
    
    observed = np.zeros(testpoints)
    for i in range(0,testpoints):
        observed[i] = np.nansum((data>=points[i])*(data<=(points[i]+dx)))
    
    
    observed = observed/np.sum(observed)
    I = (observed >0)*(pure >0)
    
    return np.sum(observed[I]*np.log(observed[I]/pure[I]))
    
    


def movemean(data,n = 3):
    
    padarray = np.zeros(len(data)+n-1)
    padarray[0:(n-1)//2] = data[0]
    padarray[-(n-1)//2:] = data[-1]
    padarray[(n-1)//2:-(n-1)//2] = data
    
    mask = np.ones(n)/n
    
    res = np.convolve(mask,padarray,mode='valid')
    
    return res
