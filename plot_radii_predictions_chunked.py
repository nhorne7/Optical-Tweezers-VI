

from Tracking_Backend.TrackingInterface import TrackingHandler
import numpy as np
def analyze_vid_chunked(vid_path, num_chunks, tracking_handler, verbose=True):
    import math
    import pims
    import matplotlib.pyplot as plt

    frames = list(pims.PyAVVideoReader(vid_path))
    total_frames = len(frames)

    if total_frames < num_chunks:
        raise ValueError("Number of chunks too large (not enough frames)")

    frames_per_chunk = math.floor(total_frames / num_chunks)
    radii = []

    for i in range(num_chunks):
        start = i * frames_per_chunk
        end = (i + 1) * frames_per_chunk if i < (num_chunks - 1) else total_frames
        chunk_frames = frames[start:end]

        print(f"\n[Chunk {i+1}/{num_chunks}] Frames {start}:{end}") if verbose else ...

        try:
            radius = tracking_handler.getAverageRadius(frames=chunk_frames, verbose=verbose)
            radii.append(radius)
        except Exception as e:
            print(f"Error processing chunk {i+1}: {e}")
            radii.append(None)

    plt.hist(radii[1:])
    plt.show()

    return radii


if __name__ == "__main__":
    vid_path = r"C:\Users\nohor3086\Desktop\Particle Recordings\Recordings\053um3.avi"
    num_chunks = 15
    
    Handler = TrackingHandler(
                                              invert=False,
                                              minmass=95,
                                              pix_diameter=9,
                                              traj_memory=20,
                                              traj_search_range=20,
                                              stub_traj_length=5,
                                              microns_per_pix=0.07289795,
                                              fps=60,
                                              zero_out_threshold=0.003)
    radii_by_chunk = analyze_vid_chunked(vid_path, num_chunks, Handler)
    print("Radii per chunk:", radii_by_chunk)
    print(np.mean(radii_by_chunk))