"""Live telemetry dashboard for the MuJoCo simulation.

Usage from main.py::

    from viz.dashboard import DashboardServer

    server = DashboardServer(port=8082)
    server.start()

    # In the sim loop:
    server.broadcast(record)   # non-blocking, drops if no clients

    # On shutdown:
    server.stop()
"""

from .server import DashboardServer

__all__ = ['DashboardServer']
