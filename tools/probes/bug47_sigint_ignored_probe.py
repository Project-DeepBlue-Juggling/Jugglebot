import signal, sys, time
import rclpy
from rclpy.executors import MultiThreadedExecutor
print('before init', signal.getsignal(signal.SIGINT), flush=True)
rclpy.init()
print('after init', signal.getsignal(signal.SIGINT), flush=True)
n = rclpy.create_node('sigign_probe')
ex = MultiThreadedExecutor(); ex.add_node(n)
try:
    ex.spin()
except KeyboardInterrupt:
    print('KeyboardInterrupt', flush=True)
print('spin returned', flush=True)
rclpy.shutdown()
