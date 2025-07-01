"""
This program takes in the path of a video, and analyzes the tracking backends performance across different numbers of frames of the video used.

This allows me to determine the optimal number of frames for sufficient accuracy to real world measurement, and stability changes, if any.
"""
import Tracking_Backend.TrackingInterface
import trackpy as tp
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    TrackingHandler = Tracking_Backend.TrackingInterface.TrackingHandler(
                                                    invert=False,
                                                    minmass=10000,
                                                    pix_diameter=51,
                                                    traj_memory=5,
                                                    traj_search_range=10,
                                                    stub_traj_length=5,
                                                    microns_per_pix=0.07289795,
                                                    fps=30)

    video_path = r"C:\Users\nohor3086\Desktop\Optical Tweezers Automation Project\Tracking_Backend\Sample_Recording\recording_2025-06-19_10-15-43.avi"

    video_length = TrackingHandler.getVidLength(video_path)

    radius_array = []

    num_splits = 30

    trajectories = TrackingHandler.getTrajectories(video_path)

    for i in range(1,num_splits+1):
        emsd = tp.emsd(trajectories, 0.07289795, 30,max_lagtime=int((i/num_splits)*video_length)) 
        fit_results = tp.utils.fit_powerlaw(emsd, plot=False)
        A = fit_results['A'].iloc[0]
        D = (A/4)*10**(-12)
        kb = 1.380649E-23
        T_k = 20+273.15
        r = (kb*T_k)/(6*np.pi*1.002E-3*D)
        radius_array.append(r*(10**6))

    plt.plot([int((i/num_splits)*video_length) for i in range (1, num_splits+1)], 1-np.array(radius_array), linestyle='--', label=r"Absolute Error of Radius ($\mu$m)", color="red")
    plt.scatter([int((i/num_splits)*video_length) for i in range (1, num_splits+1)], 1-np.array(radius_array), color="black")

    plt.legend()
    plt.title(r"Plot of Particle Radius Absolute Error ($\mu$m) against Number of Frames Analyzed")
    plt.xlabel("Number of Frames")
    plt.ylabel(r"Absolute Error of Radius ($\mu$m)")
    plt.grid()
    plt.show()