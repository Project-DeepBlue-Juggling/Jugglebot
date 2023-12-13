

# Response part contains geometry parameters
float64[] start_pos        # Array of 3x1 elements (3 total)
float64[] base_nodes       # Array of 7x3 elements (21 total)
float64[] init_plat_nodes  # Array of 7x3 elements (21 total)
float64[] init_arm_nodes   # Array of 6x3 elements (18 total)
float64[] init_hand_nodes  # Array of 3x3 elements (9 total)
float64[] init_leg_lengths # Array of 7x1 elements (7 total)
float32 leg_stroke         # How long can the legs go?
float32 hand_stroke        # How long can the hand string go?