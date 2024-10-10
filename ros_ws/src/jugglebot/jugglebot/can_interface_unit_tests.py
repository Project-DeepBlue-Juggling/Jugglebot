import unittest
from unittest.mock import MagicMock, patch, call
from typing import Optional
import struct
from can_interface import CANInterface

# Mock the ROS Logger and HeartbeatMsg
class MockLogger:
    def info(self, msg):
        pass

    def error(self, msg, *args, **kwargs):
        pass

    def warning(self, msg, *args, **kwargs):
        pass

    def debug(self, msg):
        pass

    def fatal(self, msg, *args, **kwargs):
        pass

class MockHeartbeatPublisher:
    def publish(self, msg):
        pass

class MockHeartbeatMsg:
    def __init__(self, axis_id: int, axis_error: int):
        self.axis_id = axis_id
        self.axis_error = axis_error

# Import the CANInterface class (assuming it is in the module can_interface)
# from can_interface import CANInterface

class TestCANInterface(unittest.TestCase):
    def setUp(self):
        # Mock the necessary components
        self.mock_logger = MockLogger()
        self.mock_publisher = MockHeartbeatPublisher()

        # Patch the can.Bus object before instantiating CANInterface
        self.can_bus_patcher = patch('can.Bus')
        self.mock_can_bus = self.can_bus_patcher.start()
        self.addCleanup(self.can_bus_patcher.stop)

        # Set up a MagicMock for the bus instance
        self.mock_bus_instance = MagicMock()
        self.mock_can_bus.return_value = self.mock_bus_instance

        # Patch the cantools database loading
        self.cantools_patcher = patch('cantools.database.load_file')
        self.mock_db_loader = self.cantools_patcher.start()
        self.mock_db_loader.return_value = MagicMock()
        self.addCleanup(self.cantools_patcher.stop)

        # Now, patch the blocking methods in CANInterface
        with patch.object(CANInterface, 'wait_for_heartbeat'), \
             patch.object(CANInterface, 'run_encoder_search'):
            # Instantiate CANInterface inside the patch context
            self.can_interface = CANInterface(
                logger=self.mock_logger,
                heartbeat_publisher=self.mock_publisher,
                bus_name='test_can0',
                bitrate=1000000,
                interface='virtual'  # Use 'virtual' for testing
            )

        # Mock the DBC encode_message method
        self.can_interface.db.encode_message = MagicMock(return_value=b'\x00' * 8)

        # Initialize variables that depend on external data
        self.can_interface.received_heartbeats = {axis_id: True for axis_id in range(self.can_interface.num_axes)}
        self.can_interface.iq_values = [0.0] * self.can_interface.num_axes
        self.can_interface.iq_values = [0.0] * self.can_interface.num_axes

    def test_initialization(self):
        # Test that the initialization sets up the variables correctly
        self.assertEqual(self.can_interface._can_bus_name, 'test_can0')
        self.assertEqual(self.can_interface._can_bitrate, 1000000)
        self.assertEqual(self.can_interface._can_bus_type, 'virtual')
        self.assertIsNotNone(self.can_interface.db)
        self.assertEqual(len(self.can_interface.position_buffer), self.can_interface.num_axes)
        self.assertEqual(len(self.can_interface.iq_values), self.can_interface.num_axes)

    def test_setup_can_bus(self):
        # Test that the CAN bus is set up correctly
        self.can_interface.setup_can_bus()
        self.mock_can_bus.assert_called_with(channel='test_can0', bustype='virtual', bitrate=1000000)

    def test_send_message(self):
        # Test that _send_message constructs and sends the message correctly
        self.can_interface._send_message(axis_id=0, command_name='set_input_pos', data=b'\x01\x02\x03\x04\x05\x06\x07\x08')
        self.mock_bus_instance.send.assert_called_once()
        sent_message = self.mock_bus_instance.send.call_args[0][0]
        self.assertEqual(sent_message.arbitration_id, (0 << 5) | self.can_interface.COMMANDS['set_input_pos'])
        self.assertEqual(sent_message.data, b'\x01\x02\x03\x04\x05\x06\x07\x08')

    def test_send_arbitrary_parameter_invalid_param(self):
        # Test that sending an invalid parameter raises a ValueError
        with self.assertRaises(ValueError):
            self.can_interface.send_arbitrary_parameter(axis_id=0, param_name='invalid_param', param_value=1.0)

    def test_send_position_target_clipping(self):
        # Test that the setpoint is clipped to the max position
        max_position = self.can_interface._LEG_MOTOR_MAX_POSITION
        self.can_interface.send_position_target(axis_id=0, setpoint=max_position + 1)
        self.can_interface.db.encode_message.assert_called_with(
            'Axis0_Set_Input_Pos',
            {'Input_Pos': -max_position, 'Vel_FF': 0.0, 'Torque_FF': 0.0}
        )

    def test_set_control_mode(self):
        # Test that the control mode message is sent correctly
        self.can_interface.set_control_mode(axis_id=0, control_mode='POSITION_CONTROL', input_mode='TRAP_TRAJ')
        self.can_interface.db.encode_message.assert_called_with(
            'Axis0_Set_Controller_Mode',
            {'Control_Mode': 'POSITION_CONTROL', 'Input_Mode': 'TRAP_TRAJ'}
        )
        self.mock_bus_instance.send.assert_called_once()

    def test_set_trap_traj_vel_acc_limits(self):
        # Test that the trap trajectory limits are set correctly
        self.can_interface.set_trap_traj_vel_acc_limits(velocity_limit=15.0, acceleration_limit=50.0)
        # Ensure that the internal limits are updated
        self.assertEqual(self.can_interface.leg_trap_traj_limits['vel_limit'], 15.0)
        self.assertEqual(self.can_interface.leg_trap_traj_limits['acc_limit'], 50.0)
        self.assertEqual(self.can_interface.leg_trap_traj_limits['dec_limit'], 40.0)  # 80% of 50.0

    def test_handle_heartbeat(self):
        # Test that heartbeat messages are handled correctly
        data = struct.pack('<IBBB', 0, 8, 0, 1)  # axis_error=0, axis_current_state=8, procedure_result=0, flags=1
        self.can_interface._handle_heartbeat(axis_id=0, data=data)
        self.assertEqual(self.can_interface.axis_errors[0], 0)
        self.assertEqual(self.can_interface.axis_states[0], 8)
        self.assertEqual(self.can_interface.procedure_result[0], 0)
        self.assertTrue(self.can_interface.trajectory_done[0])

    def test_run_motor_until_current_limit(self):
        # Mock iq_values to simulate current readings
        self.can_interface.iq_values[0] = 0.0

        # Simulate current increasing over time
        def side_effect(*args, **kwargs):
            self.can_interface.iq_values[0] += 1.0

        self.can_interface.fetch_messages = MagicMock(side_effect=side_effect)

        # Mock set_requested_state to avoid actual calls
        self.can_interface.set_requested_state = MagicMock()

        # Run the method
        self.can_interface.run_motor_until_current_limit(
            axis_id=0,
            homing_speed=1.0,
            current_limit=5.0,
            current_limit_headroom=1.0
        )

        # Check that the axis was set to IDLE after current limit reached
        self.can_interface.set_requested_state.assert_called_with(0, requested_state='IDLE')

    def test_get_tilt_sensor_reading(self):
        # Mock the tilt sensor reading
        self.can_interface.tilt_sensor_reading = (0.1, 0.2)

        # Mock bus.send to do nothing
        self.mock_bus_instance.send = MagicMock()

        # Run the method
        tiltX, tiltY = self.can_interface.get_tilt_sensor_reading()

        # Check the values
        self.assertEqual(tiltX, 0.1)
        self.assertEqual(tiltY, 0.2)

    def test_clear_errors(self):
        # Test that clear_errors sends the correct messages
        self.can_interface.clear_errors()
        expected_calls = [call(arbitration_id=(axis_id << 5) | self.can_interface.COMMANDS['clear_errors'],
                               dlc=8, is_extended_id=False, data=None, is_remote_frame=False)
                          for axis_id in range(self.can_interface.num_axes)]
        self.assertEqual(self.mock_bus_instance.send.call_count, self.can_interface.num_axes)

    def test_handle_error(self):
        # Test that errors are logged correctly
        active_errors = 4096  # CURRENT_LIMIT_VIOLATION
        disarm_reason = 4096
        data = struct.pack('<II', active_errors, disarm_reason)
        self.can_interface._handle_error(axis_id=0, message_data=data)
        # Since logging is mocked, we can't check the output, but we ensure no exceptions are raised

    def test_shutdown(self):
        # Test that shutdown closes the bus
        self.can_interface.shutdown()
        self.mock_bus_instance.shutdown.assert_called_once()

if __name__ == '__main__':
    unittest.main()
