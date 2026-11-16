"""Translation of mpc_timing_scope.hpp using a Python with statement."""
import math
import time


class MpcTimingScope:
    """Accumulate elapsed milliseconds in attributes, including on exceptions.

    Example: with MpcTimingScope(sample, 'solve_ms'): solve().
    A None object disables its clock. CPU measurements use the calling thread.
    """
    def __init__(self, obj, field=None, cpu_obj=None, cpu_field=None):
        if obj is not None and field is None:
            raise ValueError('A wall-time attribute is required')
        if cpu_obj is not None and cpu_field is None:
            raise ValueError('A CPU-time attribute is required')
        self.obj, self.field = obj, field
        self.cpu_obj, self.cpu_field = cpu_obj, cpu_field
        self.start = time.monotonic() if obj is not None else None
        self.cpu_start = self.threadCpuMilliseconds() if cpu_obj is not None else None
        self.closed = False

    @staticmethod
    def threadCpuMilliseconds():
        try:
            return time.thread_time() * 1000.0
        except (AttributeError, NotImplementedError, OSError, ValueError):
            return math.nan

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return False

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.obj is not None:
            setattr(self.obj, self.field, getattr(self.obj, self.field) + (time.monotonic() - self.start) * 1000.0)
        if self.cpu_obj is not None:
            setattr(self.cpu_obj, self.cpu_field, getattr(self.cpu_obj, self.cpu_field) + self.threadCpuMilliseconds() - self.cpu_start)


threadCpuMilliseconds = MpcTimingScope.threadCpuMilliseconds
