# Leg-mapping calibration data (removed from the tree)

The thirteen CSV captures that lived here (Vertical Rig and Horizontal Rig, 179 MB) were
removed on 2026-11-14 so that every checkout, worktree and clone stops carrying them.
They are in git history at the tag `archive/2026-11_leg-mapping-calibration-data`:

    git checkout archive/2026-11_leg-mapping-calibration-data -- "experimenting/platform_calibration/measuring_leg_mapping/Data"

`../data_analysis.py` reads them by relative path and needs that checkout first.
