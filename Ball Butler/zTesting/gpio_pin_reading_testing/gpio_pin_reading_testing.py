import logging
from can_interface import CANInterface
import time

# Initialize the terminal as the logger for the CAN interface
# Set up logging to print to terminal
logging.basicConfig(level=logging.INFO, format='%(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Initialize the CAN interface
can_interface = CANInterface(logger)

# Continuously request GPIO states every 100ms

start_time = time.time()
last_send_time = 0

while time.time() - start_time < 10:  # Run for 10 seconds
    current_time = time.time()
    
    # Send arbitrary parameter every 100ms
    if current_time - last_send_time >= 0.1:
        can_interface.send_arbitrary_parameter(8, "get_gpio_states", 0)
        last_send_time = current_time
    
    # Fetch messages as often as possible
    can_interface.fetch_messages()

# Close the CAN interface when done (this line will not be reached in the infinite loop)
can_interface.shutdown()