'''
This script is designed to connect to the nominated number of ODrives and run them through the
calibration sequence to re-set the encoder calibration. This is useful if the encoder has been
bumped or otherwise moved and the previous calibration is no longer valid.
'''

'''
This has only been tested (so far) on a single ODrive with one axis.
''' 

import odrive
from odrive.enums import *

num_odrives = 1
axes_per_odrive = 1

odrives = [odrive.find_any() for i in range(num_odrives)]

for odrive in odrives:
    # Note that odrive.axis is not a thing.

    # Determine if we are resetting both axes or just a single one
    if axes_per_odrive == 1:
        axes = [odrive.axis0]
    elif axes_per_odrive == 2:
        axes = [odrive.axis0, odrive.axis1]
    else:
        raise ValueError('axes_per_odrive must be 1 or 2')
    
    # Run the calibration sequence
    for axis in axes:
        # We must begin by setting the pre_calibrated flag to False
        axis.encoder.config.pre_calibrated = False
        axis.motor.config.pre_calibrated = False

        # Now run the calibration sequence
        axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        
    # Wait for the calibration to complete
    while any([axis.current_state != AXIS_STATE_IDLE for axis in axes]):
        pass

    # Now set the pre_calibrated flag to True
    for axis in axes:
        axis.encoder.config.pre_calibrated = True
        axis.motor.config.pre_calibrated = True
    
    # Save the configuration and reboot the ODrive
    odrive.save_configuration()
    odrive.reboot()
    print(f'ODrive {odrive.serial_number} rebooted')