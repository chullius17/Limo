"""Exercise the real driver on a pseudo-terminal; never open chassis hardware.

Run with the built limo_base executable as the only argument. The test uses
an isolated ROS domain and verifies the actual 0x111 serial command packets.
"""

import math
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import time

# Set before importing/initializing ROS; no publishers enter the robot domain.
os.environ['ROS_DOMAIN_ID'] = '83'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402


def run_cases(binary, left, right):
    master, slave = pty.openpty()
    with tempfile.TemporaryDirectory(prefix='limo-steering-test-') as directory:
        serial = Path(directory) / 'tty'
        serial.symlink_to(os.ttyname(slave))
        # Driver prefixes /dev/ to port_name. Resolve to this test's PTY only.
        port = '../' + str(serial).lstrip('/')
        log = open(Path(directory) / 'driver.log', 'w+')
        process = subprocess.Popen([
            str(binary), '--ros-args', '-r', '__node:=steering_serial_test',
            '-p', 'port_name:=' + port, '-p', 'motion_mode:=1',
            '-p', 'pub_odom_tf:=false', '-p', 'use_mcnamu:=false',
            '-p', 'steering_left_scale:=' + str(left),
            '-p', 'steering_right_scale:=' + str(right),
        ], stdout=log, stderr=subprocess.STDOUT)
        node = rclpy.create_node('steering_test_publisher')
        publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        try:
            deadline = time.monotonic() + 15
            while publisher.get_subscription_count() == 0:
                assert process.poll() is None, 'Driver exited during startup'
                assert time.monotonic() < deadline, 'Driver discovery timeout'
                rclpy.spin_once(node, timeout_sec=0.05)
            for v, w, expected_inner in (
                    (0.5, 1.0, math.atan(0.2 / (0.5 - 0.086))),
                    (0.5, -1.0, -math.atan(0.2 / (0.5 - 0.086))),
                    (-0.5, 1.0, -math.atan(0.2 / (0.5 - 0.086))),
                    (-0.5, -1.0, math.atan(0.2 / (0.5 - 0.086))),
                    (0.2, 1.0, 0.48869), (0.2, -1.0, -0.48869),
                    (0.0, 1.0, 0.0), (0.5, 0.0, 0.0), (0.0, 0.0, 0.0)):
                while select.select([master], [], [], 0)[0]:
                    os.read(master, 4096)
                message = Twist()
                message.linear.x, message.angular.z = v, w
                publisher.publish(message)
                buffer = b''
                deadline = time.monotonic() + 3
                found = None
                while time.monotonic() < deadline and found is None:
                    if not select.select([master], [], [], 0.1)[0]:
                        continue
                    buffer += os.read(master, 4096)
                    while len(buffer) >= 14:
                        if buffer[:2] != b'\x55\x0e':
                            buffer = buffer[1:]
                            continue
                        packet, buffer = buffer[:14], buffer[14:]
                        assert sum(packet[4:12]) & 255 == packet[13]
                        if packet[2:4] == b'\x01\x11':
                            found = struct.unpack('>hhhh', packet[4:12])
                            break
                scale = left if expected_inner > 0 else right
                expected = (int(v * 1000), 0, 0, int(expected_inner / scale * 1000))
                assert found is not None, (v, w, 'No serial command')
                assert found[:3] == expected[:3], (found, expected)
                assert abs(found[3] - expected[3]) <= 1, (found, expected)
                print('PASS scales={} cmd={} serial={}'.format((left, right), (v, w), found))
            feedback = []
            subscription = node.create_subscription(Odometry, '/odom', feedback.append, 10)

            def send_frame(identifier, payload):
                os.write(master, b'\x55\x0e' + struct.pack('>H', identifier) +
                         payload + bytes((0, sum(payload) & 255)))

            for raw_angle, scale in ((100, left), (-100, right)):
                physical_angle = raw_angle / 1000.0 * scale
                expected_yaw = math.copysign(
                    0.5 / (0.2 / math.tan(abs(physical_angle)) + 0.086), raw_angle)
                deadline = time.monotonic() + 5
                feedback.clear()
                while time.monotonic() < deadline:
                    # Raw chassis mode 1 enables Ackermann feedback decoding.
                    send_frame(0x211, bytes((0, 1, 0, 105, 0, 0, 1, 0)))
                    send_frame(0x221, struct.pack('>hhhh', 500, 0, 0, raw_angle))
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if feedback and abs(feedback[-1].twist.twist.angular.z - expected_yaw) < 1e-6:
                        break
                assert feedback and abs(feedback[-1].twist.twist.angular.z - expected_yaw) < 1e-6
                print('PASS calibrated feedback angle={} yaw={}'.format(raw_angle, expected_yaw))
            node.destroy_subscription(subscription)
            client = node.create_client(SetParameters, '/steering_serial_test/set_parameters')
            assert client.wait_for_service(timeout_sec=5)
            request = SetParameters.Request(parameters=[
                Parameter('steering_left_scale', value=1.5).to_parameter_msg()])
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done() and not future.result().results[0].successful
            print('PASS calibration is read-only while running')
        except BaseException:
            log.flush()
            log.seek(0)
            print(log.read())
            raise
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            node.destroy_node()
            log.close()
            os.close(master)
            os.close(slave)


def main():
    binary = Path(sys.argv[1]).resolve(strict=True)
    rclpy.init()
    try:
        for left, right in ((2.47, 2.47), (1.0, 1.0), (1.0, 2.47)):
            run_cases(binary, left, right)
        for name in ('steering_left_scale', 'steering_right_scale'):
            for value in ('0.0', '-1.0', '.nan', '.inf'):
                result = subprocess.run([
                    str(binary), '--ros-args', '-p', 'port_name:=disabled',
                    '-p', name + ':=' + value], capture_output=True, timeout=5)
                assert result.returncode != 0, (name, value)
                assert b'Steering scales must be finite and >= 1.0' in result.stderr
        print('PASS 27 serial cases, 6 feedback cases, 3 runtime guards, 8 invalid calibrations')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
