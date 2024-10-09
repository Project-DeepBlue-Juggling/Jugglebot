# For reporting telemetry of the hand. This is useful as a standalone topic
# as the hand operates at a higher command rate than the legs

builtin_interfaces/Time timestamp  # Timestamp
float64 position # Estimated position from the ODrive
float64 velocity # Estimated velocity from the ODrive