import pandas as pd

def load_tsv(filepath):
    """
    Load a Qualisys TSV file and return a DataFrame.
    The returned DataFrame will be 'wide': each marker has its own X/Y/Z columns.
    Example columns: ['Frame', 'Time',
                      'Marker1_X', 'Marker1_Y', 'Marker1_Z',
                      'Marker2_X', 'Marker2_Y', 'Marker2_Z', ... ]

    IMPORTANT: This function assumes:
      - There's a header block of unknown length.
      - Eventually there's a header row that starts with "Frame\tTime\tX\tY\tZ..." repeated for each marker.
      - There are multiple lines of metadata before that.
    """

    # 1) Read the entire file as lines (so we can figure out where the data starts).
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    # 2) Identify the line index where columns start (the one beginning with "Frame")
    data_start_idx = None
    for i, line in enumerate(lines):
        # We look for a line that begins with "Frame" (strip then split by tab).
        cols = line.strip().split('\t')
        if len(cols) > 0 and cols[0] == "Frame":
            data_start_idx = i
            break

    if data_start_idx is None:
        raise ValueError("Could not find the header line starting with 'Frame' in the TSV.")

    # 3) Use pandas to read the file from data_start_idx
    #    That line is the column header row; the actual data starts on the next line
    #    so data_start_idx is also the row with column names (not just metadata).
    col_names_line = lines[data_start_idx].strip('\n')
    col_names = col_names_line.split('\t')

    # We read from data_start_idx+1 to the end as the actual table
    data_lines = lines[data_start_idx+1:]

    # Now parse with pandas; tell it we already have column names in 'col_names'
    df = pd.DataFrame(
        [line.strip().split('\t') for line in data_lines],
        columns=col_names
    )

    # Convert numeric columns to floats (Frame, Time, marker coords)
    # We expect columns "Frame", "Time" plus sets of X, Y, Z
    # Some columns might be empty or all zeros if no marker data is present at that slot,
    # so we should coerce them carefully.
    numeric_cols = [c for c in df.columns if c not in ["Frame"]]
    for c in numeric_cols:
        df[c] = pd.to_numeric(df[c], errors='coerce')

    # If "Frame" should be an integer, convert:
    df["Frame"] = pd.to_numeric(df["Frame"], errors='coerce').astype(int)

    return df
