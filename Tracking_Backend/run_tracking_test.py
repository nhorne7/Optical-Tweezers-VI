import TrackingInterface
# Sample code on running a full track with file output. Parameters are tweaked to the micron scale setup.

TrackingHandler = TrackingInterface.TrackingHandler(
                                              invert=False,
                                              minmass=95,
                                              pix_diameter=9,
                                              traj_memory=5,
                                              traj_search_range=20,
                                              stub_traj_length=50,
                                              microns_per_pix=0.07289795,
                                              fps=60,
                                              zero_out_threshold=0.003,
                                              max_frames=60)
if __name__ == "__main__":
    TrackingHandler.videoAnalyzeTrajectories(r"C:\Users\nohor3086\Desktop\Particle Recordings\Recordings\053um3.avi")