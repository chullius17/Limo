"""Verify installed launch/YAML clock precedence through the live parameter service."""

import os
import signal
import subprocess
import tempfile

import rclpy
from rcl_interfaces.srv import GetParameters


def check(profile, override, serial):
    # New domains avoid stale discovery data between successive default-named nodes.
    os.environ['ROS_DOMAIN_ID'] = str(199 + serial)
    rclpy.init()
    node = rclpy.create_node('depth_launch_check')
    with tempfile.TemporaryFile(mode='w+') as output:
        args = ['ros2', 'launch', 'limo_depth', 'depth.launch.py', 'profile:=' + profile]
        if override:
            args += ['use_sim_time:=' + override]
        process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
        try:
            client = node.create_client(GetParameters, '/limo_depth/get_parameters')
            assert client.wait_for_service(timeout_sec=10), 'Launch did not start the node'
            request = GetParameters.Request(names=[
                'use_sim_time', 'calibration_frames', 'max_processing_rate_hz'])
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done(), 'Parameter service did not respond'
            values = future.result().values
            expected_clock = (override == 'true') if override else profile == 'sim'
            assert values[0].bool_value == expected_clock, values
            assert values[1].integer_value == (30 if profile == 'sim' else 10), values
            assert values[2].double_value == (0. if profile == 'sim' else 15.), values
            print('Live launch OK:', profile, 'clock override:', override)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            output.seek(0)
            logs = output.read()
            node.destroy_node()
            rclpy.shutdown()
            assert process.returncode == 0, logs
            assert 'LIMO depth ready' in logs, logs


if __name__ == '__main__':
    for serial, (profile, override) in enumerate([
            ('sim', None), ('real', None), ('real', 'true'), ('sim', 'false')]):
        check(profile, override, serial)
