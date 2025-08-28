from dataio.tsv_loader import load_tsv
from tracking.tracklet_finder import find_tracklets
import os

def main():
    # For demonstration, we’ll point to a file in the 'data' folder:
    data_path = os.path.join("data", "5_ball_juggling_cut_balls_only.tsv")

    # 1) Load the TSV
    wide_df = load_tsv(data_path)
    print("Loaded TSV data. Columns:", wide_df.columns.tolist())

    # 2) Find naive tracklets
    tracklets_dict = find_tracklets(wide_df, min_length=10)

    # 3) Summarize results
    for marker_name, tracklets in tracklets_dict.items():
        print(f"Marker: {marker_name} -> Found {len(tracklets)} tracklet(s).")
        for i, t in enumerate(tracklets):
            frames = t['frames']
            start_f = t['start_frame']
            end_f = t['end_frame']
            length = end_f - start_f + 1
            print(f"  Tracklet {i}: frames {start_f} - {end_f} (length = {length})")

if __name__ == "__main__":
    main()
