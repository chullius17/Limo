"""Finish standalone CV processes after their ROS and worker cleanup."""

import os
import sys


def exit_after_cleanup():
    """Avoid old glibc's slow DSO sorting, only for opted-in launch processes.

    Call after destroying the node and shutting down rclpy. Exceptions from
    cleanup must propagate rather than being converted into successful exits.
    This bypasses interpreter/DSO finalizers, so never call it from a library
    API or before worker threads and ROS resources have been released.
    """
    if os.environ.get('LIMO_FAST_EXIT') == '1':
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
