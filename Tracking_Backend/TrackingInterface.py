from __future__ import division, unicode_literals, print_function  # for compatibility with Python 2 and 3
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pandas import DataFrame, Series
import pims
import trackpy as tp
import os
from datetime import datetime
import time
from tqdm import tqdm
import shutil
from pathlib import Path

class TrackingHandler:
    
    def __init__(self, recording_dir=r"./Recordings", invert=False, minmass = 500, pix_diameter = 21, traj_memory = 3, traj_search_range=5, stub_traj_length = 30, microns_per_pix = 4.8, fps = 60, room_temperature_c = 20, eta = 1.002E-3, only_tagged=False, zero_out_threshold = 0.005, max_frames = 100)->None:
        self.is_running = False 
        self.recording_dir = recording_dir
        self.invert = invert
        self.minmass = minmass
        self.pix_diameter = pix_diameter
        self.traj_memory = traj_memory
        self.traj_search_range = traj_search_range
        self.stub_traj_length = stub_traj_length
        self.microns_per_pix = microns_per_pix
        self.fps = fps
        self.room_temperature_c = room_temperature_c
        self.eta = eta
        self.only_tagged = only_tagged
        self.zero_out_threshold = zero_out_threshold
        self.max_frames = max_frames
        return None
    



    # NOTE TO USER! DUE TO THE MULTIPROCESSING OF THE TP.BATCH FUNCTION, THIS FUNCTION CALL MUST ALWAYS BE WRAPPED IN A IF NAME == MAIN CONDITIONAL OR IT WILL CAUSE A RUNTIME ERROR.
    def videoAnalyzeTrajectories(self, vid_path):
        start_time = time.time()
        @pims.pipeline
        def gray(image):
            return image[:, :, 1]
        tp.enable_numba()
        tp.quiet()
        frames = gray(pims.PyAVVideoReader(vid_path))
        frames = list(frames) # Load Frames to system RAM
        n_frames = len(frames)

        print(f"Detecting Particle Positions for {vid_path}")
        f_batch = tp.batch(frames, self.pix_diameter, minmass=self.minmass, invert=self.invert, processes='auto')

        print(f"Linking & Filtering Trajectories for {vid_path}")
        trajectories = tp.link(f_batch, self.traj_search_range, memory=self.traj_memory)
        long_trajectories = tp.filter_stubs(trajectories, self.stub_traj_length)
        drift = tp.compute_drift(long_trajectories)



        print(f"Cleaning Trajectories for {vid_path}")
        # Drift Subtraction
        fixed_trajectories = tp.subtract_drift(long_trajectories.copy(), drift)
        fixed_trajectories = fixed_trajectories.reset_index(drop=True)

        # IMSD NOISE FILTERING (Remove any particles with a 0 value mean squared displacement)
        im = tp.imsd(fixed_trajectories, self.microns_per_pix, self.fps)
        # Identify particles that have zero MSD at any lag time
        particles_with_zero_msd = im.columns[(im <= self.zero_out_threshold).any(axis=0)]
        # Remove these particles from the original dataframe
        fixed_trajectories = fixed_trajectories[~fixed_trajectories['particle'].isin(particles_with_zero_msd)]
        print("Removed particles with zero MSD at any lag time:", particles_with_zero_msd.tolist())

        emsd = tp.emsd(fixed_trajectories, self.microns_per_pix, self.fps,max_lagtime=self.max_frames) 
        fit_results = tp.utils.fit_powerlaw(emsd, plot=False)
        A = fit_results['A'].iloc[0]
        n = fit_results['n'].iloc[0]
        D = (A/4)*10**(-12)

        kb = 1.380649E-23
        T_k = self.room_temperature_c+273.15
        r = (kb*T_k)/(6*np.pi*self.eta*D)

        # File Creation
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        root_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Trajectory_Data", timestamp)
        os.makedirs(root_dir, exist_ok=True)

        print("Computing per-particle diffusivities and radii for HISTOGRAM PLOTS..")

        particle_ids = fixed_trajectories['particle'].unique()
        per_particle_D = []
        per_particle_r = []

        for pid in tqdm(particle_ids):
            traj = fixed_trajectories[fixed_trajectories['particle'] == pid]
            if len(traj) < self.stub_traj_length:
                continue
            imsd = tp.imsd(traj, self.microns_per_pix, self.fps)
            try:
                fit = tp.utils.fit_powerlaw(imsd, plot=False)

                A_i = fit['A'].iloc[0]
                n_i = fit['n'].iloc[0]
                D_i = (A_i/4) * 1e-12 # In meters now
                r_i = (kb * T_k) / (6 * np.pi * self.eta * D_i)
                per_particle_D.append(D_i * 1e12) # Back to micrometers^2 seconds
                per_particle_r.append(r_i * 1e6)  # micrometers units
            except Exception as e:
                print('e')
                continue


        # Remove NaNs and convert to numpy array
        per_particle_r_um = np.array(per_particle_r)
        per_particle_r_um = per_particle_r_um[~np.isnan(per_particle_r_um)]

        # Define lower and upper percentiles to exclude extreme outliers
        lower, upper = np.percentile(per_particle_r_um, [1, 98])

        fig_r, ax_r = plt.subplots()
        ax_r.hist(per_particle_r_um, bins=100, color="blue", edgecolor='black')

        # Apply percentile-based x-axis range
        ax_r.set_xlim(lower, upper)

        # Labeling
        ax_r.set_xlabel(r'Particle Radius [$\mu$m]')
        ax_r.set_ylabel('Particle Count')
        ax_r.set_title('Histogram of Estimated Particle Radii')
        ax_r.grid(True)

        fig_r.tight_layout()
        fig_r.savefig(os.path.join(root_dir, "radii_hist.png"))
        plt.close(fig_r)


        import matplotlib.ticker as ticker
        fig_D, ax_D = plt.subplots()
        bins = 150
        
        counts, bin_edges, _ = ax_D.hist(per_particle_D, bins=bins, color='orange', alpha= 0.6, label="Diffusivity Histogram")

        # Plot expected PDF Behavior
        bin_centers = (bin_edges[:-1]+bin_edges[1:]) / 2
        mean_D = np.mean(per_particle_D)
        pdf = (1/mean_D) * np.exp(-bin_centers/mean_D)
        pdf_scaled = pdf * np.diff(bin_edges)[0] * len(per_particle_D)
        ax_D.plot(bin_centers, pdf_scaled, "k--", linewidth=2, label=fr"$P(D) = \frac{{1}}{{{mean_D:.2f}}} e^{{-D/{mean_D:.2f}}}$")


        ax_D.set_xlim(0,max(10, np.nanmean(per_particle_D)+1))
        ax_D.set_xlabel(r'Diffusivity [$\mu$m$^2$/s]')
        ax_D.set_ylabel("Particle Count")
        ax_D.grid()
        ax_D.legend()
        fig_D.tight_layout()
        fig_D.savefig(os.path.join(root_dir, "diffusivity_hist.png"))
        plt.close(fig_D)

        # also store csv backup of per particle calculated radii and diffusivities
        hist_df = pd.DataFrame({
            'particle_id' : particle_ids[:len(per_particle_r)],
            'radius_um' : per_particle_r,
            'diffusivity_um2s' : per_particle_D
        })
        hist_df.to_csv(os.path.join(root_dir, "per_particle_diffusivity_radius.csv"), index=False)

        # Save trajectory CSV
        traj_csv_path = os.path.join(root_dir, "trajectory.csv")
        fixed_trajectories.to_csv(traj_csv_path, index=False)

        # Info.txt
        video_name = os.path.basename(vid_path)
        frame_shape = frames[0].shape[:2]  # (height, width)
        res_str = f"{frame_shape[1]} x {frame_shape[0]}"
        n_total_traj = trajectories['particle'].nunique()
        n_filtered_traj = fixed_trajectories['particle'].nunique()
        mean_len = fixed_trajectories.groupby('particle').size().mean()
        msd_lag1 = emsd.iloc[0]
        r_um = r * 1e6
        D_um2s = D * 1e12
        info_path = os.path.join(root_dir, "info.txt")
        with open(info_path, 'w') as f:
            f.write("TRAJECTORY ANALYSIS SUMMARY\n")
            f.write(f"Analysis Date: {datetime.now()}\n")
            f.write(f"Video File: {video_name}\n")
            f.write(f"Resolution: {res_str}\n")
            f.write(f"Total Frames: {n_frames}\n")
            f.write(f"Microns per Pixel: {self.microns_per_pix} um/px\n")
            f.write(f"Framerate: {self.fps} fps\n")
            f.write(f"Inversion Applied: {self.invert}\n\n")

            f.write("TRACKING PARAMETERS\n")
            f.write(f"Minmass: {self.minmass}\n")
            f.write(f"Particle Radius: {self.pix_diameter} px\n")
            f.write(f"Search Range: {self.traj_search_range} px\n")
            f.write(f"Memory: {self.traj_memory} frames\n")
            f.write(f"Stub Trajectory Threshold: {self.stub_traj_length} frames\n\n")

            f.write("RESULTS\n")
            f.write(f"Initial Trajectories: {n_total_traj}\n")
            f.write(f"Filtered Long Trajectories: {n_filtered_traj}\n")
            f.write(f"Mean Trajectory Length: {mean_len:.1f} frames\n")
            f.write(f"Average Radius: {r_um:.3f} um\n")
            f.write(f"Diffusion Coefficient: {D_um2s:.3f} um^2/s  ({D:.2e} m^2/s)\n")
            f.write(f"MSD Fit Parameters: A = {A:.2e}, n = {n:.2f}\n")
            f.write(f"MSD @ 1 frame lag: {msd_lag1:.3f} um^2\n\n")

            f.write("PHYSICAL CONSTANTS\n")
            f.write(f"Temperature: {self.room_temperature_c} C ({T_k:.2f} K)\n")
            f.write(f"Viscosity: {self.eta:.3e} Pa s\n")
            f.write(f"Boltzmann Constant: {kb:.6e} J/K\n\n")

            motion_type = "approx. diffusive (n = 1)"
            if n < 0.9:
                motion_type = "subdiffusive! (n < 1)"
            elif n > 1.1:
                motion_type = "superdiffusive! (n > 1)"
            f.write(f"Notes: Appears consistent with {motion_type} behavior.\n")

        
        # Image Creation
        imsd = tp.imsd(fixed_trajectories, self.microns_per_pix, self.fps, max_lagtime=self.max_frames)
        fig1, ax1 = plt.subplots()
        ax1.plot(imsd.index, imsd, 'k-', alpha=0.2)
        ax1.set_xscale('log')
        ax1.set_yscale('log')
        ax1.set_xlabel(r'Lag time $t$ [seconds]')
        ax1.set_ylabel(r'$\langle \Delta r^2 \rangle$ [$\mu$m$^2$]')
        ax1.grid(which='both', color='gainsboro')
        ax1.set_title(r"Individual MSD Trajectories with $\mu m^2$ Scale")
        fig1.tight_layout()
        fig1.savefig(os.path.join(root_dir, "individual_msd.png"))
        plt.close(fig1)

        A_fit = fit_results['A'].iloc[0]
        n_fit = fit_results['n'].iloc[0]
        fig2, ax2 = plt.subplots()
        ax2.loglog(emsd.index, emsd, 'o', label='Ensemble MSD', color='black', markersize=5)
        ax2.loglog(emsd.index, A_fit * emsd.index**n_fit, 'r--', label=fr'Fit: $A t^n$, $A={A_fit:.2e}$, $n={n_fit:.2f}$')
        ax2.set_xlabel(r'Lag time $t$ [seconds]')
        ax2.set_ylabel(r'$\langle \Delta r^2 \rangle$ [$\mu$m$^2$]')
        ax2.legend()
        ax2.grid(which='both', color='gainsboro')
        ax2.set_title(r"Ensemble Averaged MSD Trajectory with $\mu m^2$ Scale")

        fig2.tight_layout()
        fig2.savefig(os.path.join(root_dir, "ensemble_msd.png"))
        plt.close(fig2)

        fig3, ax3 = plt.subplots()
        tp.plot_traj(fixed_trajectories, ax=ax3)
        fig3.tight_layout()
        fig3.savefig(os.path.join(root_dir, "trajectories.png"))
        plt.close(fig3)
        tp.disable_numba()
        print(f"{n_frames} frames analyzed in {(time.time()-start_time):.2f} s")
        return fixed_trajectories, root_dir



# USED FOR INDIVIDUAL PARTICLE TAGGING (working)


    def tagBoxedTrajectories(self, trajectories:pd.DataFrame, bbox_start, bbox_end)->pd.DataFrame:
        boundsTL = (min(bbox_start[0], bbox_end[0]), min(bbox_start[1],bbox_end[1]))
        boundsBR = (max(bbox_start[0], bbox_end[0]), max(bbox_start[1],bbox_end[1]))
        first_frames = trajectories.sort_values('frame').groupby('particle').first().reset_index()
        in_box = ((first_frames['x'] >= boundsTL[0]) & (first_frames['x'] <= boundsBR[0]) & (first_frames['y'] >= boundsTL[1]) & (first_frames['y'] <= boundsBR[1]))
        particles_in_box = first_frames.loc[in_box, 'particle']
        trajectories['started_in_box'] = trajectories['particle'].isin(particles_in_box)
        print(f"Particles starting in box: {list(particles_in_box)}")
        return trajectories



    def analyzeTaggedTrajectories(self, trajectories:pd.DataFrame, root_dir)->None:
        filtered_trajectories = trajectories.loc[trajectories['started_in_box'] == True]
        filtered_trajectories = filtered_trajectories.reset_index(drop=True)
        emsd = tp.emsd(filtered_trajectories, self.microns_per_pix, self.fps) 
        fit_results = tp.utils.fit_powerlaw(emsd, plot=False)
        A = fit_results['A'].iloc[0]
        n = fit_results['n'].iloc[0]
        D = (A/4)*10**(-12)

        kb = 1.380649E-23
        T_k = self.room_temperature_c+273.15
        r = (kb*T_k)/(6*np.pi*self.eta*D)


        # Save trajectory CSV
        trapped_traj_csv_path = os.path.join(root_dir, "originally_trapped_trajectory.csv")
        filtered_trajectories.to_csv(trapped_traj_csv_path, index=False)
        traj_csv_path = os.path.join(root_dir, "tagged_trajectory.csv")
        trajectories.to_csv(traj_csv_path, index=False)

        # Info.txt
        n_filtered_traj = filtered_trajectories['particle'].nunique()
        mean_len = filtered_trajectories.groupby('particle').size().mean()
        msd_lag1 = emsd.iloc[0]
        r_um = r * 1e6
        D_um2s = D * 1e12
        info_path = os.path.join(root_dir, "trapped_info.txt")
        with open(info_path, 'w') as f:
            f.write("======THIS IS FOR ONLY THOSE PARTICLES STARTING IN BOUNDING BOX======\n")
            f.write("RESULTS\n")
            f.write(f"Originally Trapped Trajectories: {n_filtered_traj}\n")
            f.write(f"Mean Trajectory Length: {mean_len:.1f} frames\n")
            f.write(f"Average Radius: {r_um:.3f} um\n")
            f.write(f"Average Diameter: {r_um*2:.3f} um\n")
            f.write(f"Diffusion Coefficient: {D_um2s:.3f} um^2/s  ({D:.2e} m^2/s)\n")
            f.write(f"MSD Fit Parameters: A = {A:.2e}, n = {n:.2f}\n")
            f.write(f"MSD @ 1 frame lag: {msd_lag1:.3f} um^2\n\n")

            f.write("PHYSICAL CONSTANTS\n")
            f.write(f"Temperature: {self.room_temperature_c} C ({T_k:.2f} K)\n")
            f.write(f"Viscosity: {self.eta:.3e} Pa s\n")
            f.write(f"Boltzmann Constant: {kb:.6e} J/K\n\n")

            motion_type = "diffusive (n = 1)"
            if n < 0.9:
                motion_type = "subdiffusive! (n < 1)"
            elif n > 1.1:
                motion_type = "superdiffusive! (n > 1)"
            f.write(f"Notes: Appears consistent with {motion_type} behavior.\n")

        

        A_fit = fit_results['A'].iloc[0]
        n_fit = fit_results['n'].iloc[0]
        fig2, ax2 = plt.subplots()
        ax2.loglog(emsd.index, emsd, 'o', label='Trapped Particle MSD')
        ax2.loglog(emsd.index, A_fit * emsd.index**n_fit, '--', label=fr'Fit: $A t^n$, $A={A_fit:.2e}$, $n={n_fit:.2f}$')
        ax2.set_xlabel(r'Lag time $t$ [seconds]')
        ax2.set_ylabel(r'$\langle \Delta r^2 \rangle$ [$\mu$m$^2$]')
        ax2.legend()
        fig2.tight_layout()
        fig2.savefig(os.path.join(root_dir, "trapped_particle_msd.png"))
        plt.close(fig2)

        fig3, ax3 = plt.subplots()
        tp.plot_traj(filtered_trajectories, ax=ax3)
        fig3.tight_layout()
        fig3.savefig(os.path.join(root_dir, "trapped_particle_trajectory.png"))
        plt.close(fig3)




    """This function takes in a to_track directory, finds the oldest video file in this directory, and performs a track on it. 
    
    After it is complete, it moves this video file to the complete_track_dir.
    
    When called, it does this one time. if there are no compatible files in the directory, nothing happens."""
    def trackLatest(self, to_track_dir, complete_track_dir, window_controller): 
        allowed_extensions = {'.mp4', '.avi', '.mov', '.mkv'} # Update this later if adding more lossless video recording formats!

        # Convert inputs that we have into path objects
        to_track_path = Path(to_track_dir)
        complete_path = Path(complete_track_dir)

        # Make sure that the output directory that we will write to exists
        complete_path.mkdir(parents=True, exist_ok=True)

        # Now we create a list of all compatible video files in the directory
        video_files = [file for file in to_track_path.iterdir() if file.is_file() and file.suffix.lower() in allowed_extensions] # Ensures that the file is indeed a valid file

        if not video_files:
            print("No video files left to track.")
            return False
        
        video_files.sort(key=lambda f: f.stat().st_mtime)

        oldest_file = video_files[0] # Only take out the oldest video (smallest st_mtime)

        # Analyze the trajectories of EVERY PARTICLE for the oldest video
        trajectories, vid_path = self.videoAnalyzeTrajectories(str(oldest_file))

        # Analyze specifically the trajectories of the TRAPPED PARTICLES for the oldest video
        trapped_trajectories = self.tagBoxedTrajectories(trajectories, window_controller.bbox_start, window_controller.bbox_end)
        self.analyzeTaggedTrajectories(trapped_trajectories, vid_path)

        # After completion, move the file to the complete_track_dir
        target_path = complete_path / oldest_file.name
        shutil.move(str(oldest_file), str(target_path))

        print(f"Video {oldest_file.name} transferred to {complete_track_dir} successfully")
        return True
    
    def getAverageRadius(self, vid_path=None, frames=None, total_frames=None, verbose=True):
        if frames is None:
            if vid_path is None:
                raise ValueError("Must provide either frames or vid_path!")
            
            @pims.pipeline
            def gray(image):
                return image[:, :, 1]
            tp.enable_numba()
            tp.quiet()
            frames = gray(pims.PyAVVideoReader(vid_path))
            frames = list(frames) # Load Frames to system RAM
        else:
            if len(frames[0].shape) == 3:
                frames = [f[:, :, 1] if f.shape[2] >= 2 else f for f in frames]
        
        print(f"Detecting Particle Positions for {vid_path}") if verbose else ...
        f_batch = tp.batch(frames, self.pix_diameter, minmass=self.minmass, invert=self.invert, processes='auto')

        print(f"Linking & Filtering Trajectories for {vid_path}") if verbose else ...
        trajectories = tp.link(f_batch, self.traj_search_range, memory=self.traj_memory)
        long_trajectories = tp.filter_stubs(trajectories, self.stub_traj_length)
        drift = tp.compute_drift(long_trajectories)

        print(f"Cleaning Trajectories for {vid_path}") if verbose else ...
        fixed_trajectories = tp.subtract_drift(long_trajectories.copy(), drift)
        fixed_trajectories = fixed_trajectories.reset_index(drop=True)

        # IMSD NOISE FILTERING (Remove any particles with a 0 value mean squared displacement)
        im = tp.imsd(fixed_trajectories, self.microns_per_pix, self.fps)
        # Identify particles that have zero MSD at any lag time
        particles_with_zero_msd = im.columns[(im <= self.zero_out_threshold).any(axis=0)]
        # Remove these particles from the original dataframe
        fixed_trajectories = fixed_trajectories[~fixed_trajectories['particle'].isin(particles_with_zero_msd)]
        print("Removed particles with zero MSD at any lag time:", particles_with_zero_msd.tolist())

        emsd = tp.emsd(fixed_trajectories, self.microns_per_pix, self.fps,max_lagtime=self.max_frames) 
        fit_results = tp.utils.fit_powerlaw(emsd, plot=False)
        A = fit_results['A'].iloc[0]
        D = (A/4)*10**(-12)

        kb = 1.380649E-23
        T_k = self.room_temperature_c+273.15
        r = (kb*T_k)/(6*np.pi*self.eta*D)

        return r
    
    def getVidLength(self, vid_path):
        @pims.pipeline
        def gray(image):
            return image[:, :, 1]
        tp.enable_numba()
        tp.quiet()
        frames = gray(pims.PyAVVideoReader(vid_path))
        frames = list(frames) # Load Frames to system RAM
        n_frames = len(frames)
        return n_frames

    def getTrajectories(self, vid_path):
        start_time = time.time()
        @pims.pipeline
        def gray(image):
            return image[:, :, 1]
        tp.enable_numba()
        tp.quiet()
        frames = gray(pims.PyAVVideoReader(vid_path))
        frames = list(frames) # Load Frames to system RAM
        n_frames = len(frames)

        print(f"Detecting Particle Positions for {vid_path}")
        f_batch = tp.batch(frames, self.pix_diameter, minmass=self.minmass, invert=self.invert, processes='auto')

        print(f"Linking & Filtering Trajectories for {vid_path}")
        trajectories = tp.link(f_batch, self.traj_search_range, memory=self.traj_memory)
        long_trajectories = tp.filter_stubs(trajectories, self.stub_traj_length)
        drift = tp.compute_drift(long_trajectories)

        print(f"Cleaning Trajectories for {vid_path}")
        fixed_trajectories = tp.subtract_drift(long_trajectories.copy(), drift)
        fixed_trajectories = fixed_trajectories.reset_index(drop=True)
         
        # IMSD NOISE FILTERING (Remove any particles with a 0 value mean squared displacement)
        im = tp.imsd(fixed_trajectories, self.microns_per_pix, self.fps)
        # Identify particles that have zero MSD at any lag time
        particles_with_zero_msd = im.columns[(im <= self.zero_out_threshold).any(axis=0)]
        # Remove these particles from the original dataframe
        fixed_trajectories = fixed_trajectories[~fixed_trajectories['particle'].isin(particles_with_zero_msd)]
        print("Removed particles with zero MSD at any lag time:", particles_with_zero_msd.tolist())

        print(f"{n_frames} frames analyzed in {(time.time()-start_time):.2f} s")
        return fixed_trajectories

#df = pd.read_csv("/Users/noah/Desktop/Particle Tracking/TrackingSoftware/Trajectory_Data/FirstTrackTestData/trajectory.csv")
#tagged_df = tagBoxedTrajectories(df, (100,100), (350,350))
#analyzeTaggedTrajectories(tagged_df)