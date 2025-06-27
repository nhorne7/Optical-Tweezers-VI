import TrackingInterface
# Sample code on running a full track with file output. Parameters are tweaked to the micron scale setup.
if __name__ == "__main__":
    TrackingInterface.videoAnalyzeTrajectories(r"C:\Users\nohor3086\Desktop\KSC101_Shutter_Controller\Recordings\recording_2025-06-19_10-16-51.avi",
                                              invert=False,
                                              minmass=10000,
                                              pix_diameter=51,
                                              traj_memory=5,
                                              traj_search_range=10,
                                              stub_traj_length=5,
                                              microns_per_pix=0.07289795,
                                              fps=60)