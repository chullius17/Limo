"""Verify the standalone workaround preserves cleanup output and error exits."""

import os
from pathlib import Path
import subprocess
import sys

import pytest


PACKAGE = Path(__file__).resolve().parents[1]


def run_child(code, enabled):
    env = dict(os.environ, LIMO_FAST_EXIT=enabled,
               PYTHONPATH=str(PACKAGE) + os.pathsep + os.environ.get('PYTHONPATH', ''))
    return subprocess.run([sys.executable, '-c', code], env=env,
                          capture_output=True, text=True, timeout=3)


def test_cleanup_output_is_flushed_without_entering_native_finalization():
    result = run_child('''
import atexit, time
from cv_package.process_shutdown import exit_after_cleanup
atexit.register(lambda: time.sleep(10))
print('ROS cleanup completed')
exit_after_cleanup()
''', '1')
    assert result.returncode == 0
    assert result.stdout == 'ROS cleanup completed\n'


def test_normal_finalizers_run_when_workaround_is_disabled():
    result = run_child('''
import atexit
from cv_package.process_shutdown import exit_after_cleanup
atexit.register(lambda: print('normal finalizer'))
exit_after_cleanup()
''', '0')
    assert result.returncode == 0
    assert result.stdout == 'normal finalizer\n'


def test_cleanup_failure_is_not_reported_as_success():
    pytest.importorskip('rclpy')
    result = run_child('''
from cv_package.depth_correction import depth_correction as depth
class BrokenNode:
    def destroy_node(self):
        raise RuntimeError('cleanup failed')
def interrupt(node):
    raise KeyboardInterrupt()
depth.DepthCorrection = BrokenNode
depth.rclpy.init = lambda args: None
depth.rclpy.spin = interrupt
depth.rclpy.shutdown = lambda: None
depth.main()
''', '1')
    assert result.returncode != 0
    assert 'cleanup failed' in result.stderr
