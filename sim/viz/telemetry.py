"""Re-export shim — canonical location is controller.telemetry."""
from controller.telemetry import (
    StepRecord,
    TelemetryLogger,
    record_from_arrays,
    plot_tracking,
    plot_solve_times,
)

__all__ = [
    'StepRecord',
    'TelemetryLogger',
    'record_from_arrays',
    'plot_tracking',
    'plot_solve_times',
]
