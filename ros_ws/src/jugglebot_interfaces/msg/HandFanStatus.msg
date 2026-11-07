# The Platform Teensy's Hand fan, from its 1 Hz HAND_FAN_STATUS frame.
string mode            # auto, on or off
bool fan_on            # What the fan pin is driving now
bool reading_valid     # A usable Hand motor thermistor reading is current
float32 on_above_c     # AUTO: reading above this turns the fan on (deg C)
float32 off_below_c    # AUTO: reading below this turns the fan off (deg C)
float32 motor_temp_c   # Hand motor thermistor (deg C); 0 when reading_valid is false
