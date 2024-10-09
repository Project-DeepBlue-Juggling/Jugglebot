import can
import time

def send_can_messages(interface, channel, bitrate, num_messages=100):
    # Configure the CAN interface
    bus = can.interface.Bus(bustype=interface, channel=channel, bitrate=bitrate)

    try:
        for i in range(num_messages):
            msg = can.Message(arbitration_id=0x123, data=[i % 256], is_extended_id=False)

            try:
                bus.send(msg, timeout=0.1)
                print(f"Message {i} sent")
                time.sleep(0.5)
            except can.CanError as e:
                print(f"Error sending message {i}: {e}")
                break

            # Optional delay between messages
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Testing interrupted.")
    finally:
        bus.shutdown()
        print("Testing completed.")

def send_messages_high_throughput(interface='can0', bitrate=1000000):
    bus = can.interface.Bus(channel=interface, bustype='socketcan', bitrate=bitrate)
    msg = can.Message(arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7], is_extended_id=False)

    print("Starting to send messages...")
    start_time = time.time()
    count = 0

    try:
        while True:
            bus.send(msg)
            count += 1
            if count % 10000 == 0:  # Adjust this value as needed for your console output frequency
                elapsed = time.time() - start_time
                print(f"Sent {count} messages in {elapsed:.2f} seconds")

            time.sleep(0.0002)
            # time.sleep(0.0001) # Works with ~3600 -- 4700 messages per second (depending on ROS activity)
            # time.sleep(0.00009) # Works up to ~5200 messages per second. Drops to ~3600 with high ROS activity
            # time.sleep(0.00004)
            

    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        bus.shutdown()
        print(f"\nStopped. Sent {count} messages in {elapsed:.2f} seconds")

    # If there's no buffer space available (error 105), print the number of messages sent
    except can.CanError as e:
        bus.shutdown()
        print(f"Error sending message: {e}. Sent {count} messages")

    except Exception as e:
        bus.shutdown()
        print("Unexpected error:", e)
        raise


if __name__ == "__main__":
    INTERFACE = 'socketcan'  # or 'virtual', etc., depending on your setup
    CHANNEL = 'can0'  # adjust based on your CAN interface
    BITRATE = 1000000  # Set your CAN network's bitrate

    # send_can_messages(INTERFACE, CHANNEL, BITRATE)
    send_messages_high_throughput()