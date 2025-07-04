import TrackingInterface
# Sample code on running a full track with file output. Parameters are tweaked to the micron scale setup.

TrackingHandler = TrackingInterface.TrackingHandler(
                                              invert=False,
                                              minmass=9000,
                                              pix_diameter=45,
                                              traj_memory=3,
                                              traj_search_range=10,
                                              stub_traj_length=5,
                                              microns_per_pix=0.07289795,
                                              fps=60,
                                              zero_out_threshold=0.003,
                                              max_frames=60)
if __name__ == "__main__":
    TrackingHandler.videoAnalyzeTrajectories(r"C:\Users\nohor3086\Desktop\Optical Tweezers Automation Project\Tracking_Backend\Sample_Recording\recording_2025-06-19_10-16-43.avi")