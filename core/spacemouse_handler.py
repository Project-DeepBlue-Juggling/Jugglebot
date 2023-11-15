import pyspacemouse
import time

class SpaceMouseHandler:
    def __init__(self):
        """Initialize and open the SpaceMouse."""
        self._is_open = pyspacemouse.open()

        if not self._is_open:
            raise ConnectionError("Failed to connect to the SpaceMouse.")

    def read(self):
        """
        Read the state of the SpaceMouse.

        Returns:
            list: A list containing the state attributes [x, y, z, roll, pitch, yaw]
        """
        state = pyspacemouse.read()
        return [state.x, state.y, state.z, state.roll, -state.pitch, -state.yaw]

    def close(self):
        """Close the connection to the SpaceMouse."""
        pyspacemouse.close()

    def __enter__(self):
        """For use with the 'with' statement."""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Automatically close the connection when done."""
        self.close()
        
if __name__ == "__main__":
    with SpaceMouseHandler() as sm:
        try:
            while True:
                data = sm.read()
                print(f'Tx: {data[0]:.2f}, Ty: {data[1]:.2f}, Tz: {data[2]:.2f}, Rx: {data[3]:.2f}, Ry: {data[4]:.2f}, Rz: {data[5]:.2f}')
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\nExiting...")