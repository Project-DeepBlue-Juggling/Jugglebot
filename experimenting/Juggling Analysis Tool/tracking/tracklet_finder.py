import numpy as np
import pandas as pd

def find_tracklets(wide_df, min_length=10):
    """
    For each marker (Marker1_X/Y/Z, Marker2_X/Y/Z, ...),
    find continuous segments of frames where the marker is visible.
    We'll call these naive "tracklets" for that marker.

    Returns a dictionary:
      {
        (marker_name): [
           { 'start_frame': int,
             'end_frame': int,
             'frames': np.array([...]),
             'x': np.array([...]),
             'y': np.array([...]),
             'z': np.array([...]) },
           ... possibly multiple tracklets
        ],
        ...
      }
    """
    # Identify columns that look like Marker#_X, Marker#_Y, Marker#_Z
    marker_base_names = set()
    for c in wide_df.columns:
        if "Marker" in c and c.endswith("_X"):
            base_name = c[:-2]  # e.g. "Marker1"
            marker_base_names.add(base_name)

    # Sort base names in ascending numeric order if you want consistency
    # We'll just do a naive sort
    marker_base_names = sorted(list(marker_base_names))

    results = {}
    for base in marker_base_names:
        x_col = base + "_X"
        y_col = base + "_Y"
        z_col = base + "_Z"

        # We'll go through the DataFrame frame-by-frame to find runs of "valid data"
        frames = wide_df["Frame"].values
        xs = wide_df[x_col].values
        ys = wide_df[y_col].values
        zs = wide_df[z_col].values

        valid_mask = ~np.isnan(xs)  # True if we have data (not NaN)
        # (You could also check if they are not zero if QTM sets missing to 0,
        # but sometimes it's 0 or 0.0 means something else. Adjust as needed.)

        tracklets = []
        current_tracklet = None

        for i in range(len(frames)):
            if valid_mask[i]:
                # If we have data at i
                if current_tracklet is None:
                    # start a new tracklet
                    current_tracklet = {
                        'start_frame': frames[i],
                        'frames': [frames[i]],
                        'x': [xs[i]],
                        'y': [ys[i]],
                        'z': [zs[i]],
                    }
                else:
                    # continue current tracklet
                    current_tracklet['frames'].append(frames[i])
                    current_tracklet['x'].append(xs[i])
                    current_tracklet['y'].append(ys[i])
                    current_tracklet['z'].append(zs[i])
            else:
                # No data for this marker at frame i
                if current_tracklet is not None:
                    # We just ended a tracklet
                    current_tracklet['end_frame'] = current_tracklet['frames'][-1]
                    # check length
                    if len(current_tracklet['frames']) >= min_length:
                        tracklets.append({
                            'start_frame': current_tracklet['start_frame'],
                            'end_frame': current_tracklet['end_frame'],
                            'frames': np.array(current_tracklet['frames']),
                            'x': np.array(current_tracklet['x']),
                            'y': np.array(current_tracklet['y']),
                            'z': np.array(current_tracklet['z'])
                        })
                    current_tracklet = None

        # If the last frames ended in a valid tracklet, close it out
        if current_tracklet is not None:
            current_tracklet['end_frame'] = current_tracklet['frames'][-1]
            if len(current_tracklet['frames']) >= min_length:
                tracklets.append({
                    'start_frame': current_tracklet['start_frame'],
                    'end_frame': current_tracklet['end_frame'],
                    'frames': np.array(current_tracklet['frames']),
                    'x': np.array(current_tracklet['x']),
                    'y': np.array(current_tracklet['y']),
                    'z': np.array(current_tracklet['z'])
                })

        results[base] = tracklets

    return results
