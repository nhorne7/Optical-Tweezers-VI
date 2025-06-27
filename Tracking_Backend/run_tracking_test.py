import TrackingInterface
# Sample code on running a full track with file output. Parameters are tweaked to the micron scale setup.

TrackingHandler = TrackingInterface.TrackingHandler(
                                              invert=False,
                                              minmass=10000,
                                              pix_diameter=51,
                                              traj_memory=5,
                                              traj_search_range=10,
                                              stub_traj_length=5,
                                              microns_per_pix=0.07289795,
                                              fps=60)
if __name__ == "__main__":
    TrackingHandler.videoAnalyzeTrajectories(r"Tracking_Backend\Sample_Recording\recording_2025-06-19_10-16-51.avi")