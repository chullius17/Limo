"""Verify queued TF, interpolation, late publishers and inactive callbacks."""

import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from limo_runtime.tf_buffer import ConditionalCallbackGroup, QueryTransformBuffer


@pytest.fixture
def nodes():
    rclpy.init()
    publisher = Node('tf_test_publisher')
    consumer = Node('tf_test_consumer')
    buffer = QueryTransformBuffer(consumer)
    yield publisher, consumer, buffer
    buffer.close()
    buffer.close()  # Cleanup is safe even on a repeated shutdown path.
    consumer.destroy_node()
    publisher.destroy_node()
    rclpy.shutdown()


def transform(parent, child, stamp, x):
    value = TransformStamped()
    value.header.frame_id = parent
    value.child_frame_id = child
    value.header.stamp.sec = stamp
    value.transform.translation.x = float(x)
    value.transform.rotation.w = 1.0
    return value


def wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(0.01)


def test_idle_queue_preserves_static_chain_and_dynamic_interpolation(nodes):
    publisher, consumer, buffer = nodes
    dynamic = publisher.create_publisher(TFMessage, '/tf', 100)
    static = publisher.create_publisher(
        TFMessage, '/tf_static',
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    wait_for(lambda: dynamic.get_subscription_count() and static.get_subscription_count())
    static.publish(TFMessage(transforms=[transform('map', 'odom', 0, 10)]))
    dynamic.publish(TFMessage(transforms=[transform('odom', 'base_link', 10, 1)]))
    dynamic.publish(TFMessage(transforms=[transform('odom', 'base_link', 12, 3)]))
    time.sleep(0.1)  # No executor has run: data must remain in DDS until queried.
    assert buffer.all_frames_as_yaml() == '[]'
    actual = buffer.lookup_transform('map', 'base_link', Time(seconds=11))
    assert actual.transform.translation.x == pytest.approx(12.0)
    latest = buffer.lookup_transform('map', 'base_link', Time())
    assert latest.header.stamp.sec == 12
    assert latest.transform.translation.x == pytest.approx(13.0)


def test_timeout_receives_tf_while_the_application_thread_is_waiting(nodes):
    publisher, consumer, buffer = nodes
    dynamic = publisher.create_publisher(TFMessage, '/tf', 100)
    wait_for(lambda: dynamic.get_subscription_count())
    timer = threading.Timer(
        0.08, lambda: dynamic.publish(TFMessage(
            transforms=[transform('map', 'base_link', 12, 3)])))
    timer.start()
    try:
        assert buffer.can_transform(
            'map', 'base_link', Time(), timeout=Duration(seconds=0.5))
    finally:
        timer.join()
    assert buffer.can_transform('map', 'base_link', Time(), return_debug_tuple=True)[0]
    started = time.monotonic()
    assert not buffer.can_transform(
        'missing', 'base_link', Time(), timeout=Duration(seconds=0.05))
    assert 0.04 <= time.monotonic() - started < 0.3


def test_query_does_not_run_parent_callbacks(nodes):
    publisher, consumer, buffer = nodes
    called = []
    consumer.create_timer(0.001, lambda: called.append(True))
    time.sleep(0.02)
    buffer.can_transform('missing', 'base_link', Time())
    assert not called


def test_receiver_has_a_unique_name_and_honors_global_tf_remapping():
    rclpy.init(args=[
        '--ros-args', '-r', '__node:=remapped_parent',
        '-r', 'tf:=/remapped_tf', '-r', 'tf_static:=/remapped_tf_static'])
    parent = Node('original_name')
    buffer = QueryTransformBuffer(parent)
    try:
        assert parent.get_name() == 'remapped_parent'
        assert buffer._receiver.get_name() == 'remapped_parent_query_tf'
        assert buffer._listener.tf_sub.topic_name == '/remapped_tf'
        assert buffer._listener.tf_static_sub.topic_name == '/remapped_tf_static'
    finally:
        buffer.close()
        parent.destroy_node()
        rclpy.shutdown()


def test_disabled_subscription_waits_in_dds_and_resumes_with_latest_sample(nodes):
    publisher, consumer, buffer = nodes
    enabled = [False]
    called = []
    group = ConditionalCallbackGroup(lambda: enabled[0])
    consumer.create_subscription(
        String, '/conditional_test', lambda msg: called.append(msg.data), 1,
        callback_group=group)
    writer = publisher.create_publisher(String, '/conditional_test', 1)
    wait_for(lambda: writer.get_subscription_count())
    executor = SingleThreadedExecutor()
    executor.add_node(consumer)
    try:
        writer.publish(String(data='old'))
        time.sleep(0.03)
        writer.publish(String(data='latest'))
        time.sleep(0.03)
        for _ in range(5):
            executor.spin_once(timeout_sec=0.005)
        assert not called
        enabled[0] = True
        executor.wake()
        wait_for(lambda: (executor.spin_once(timeout_sec=0.01), bool(called))[1])
        assert called == ['latest']
    finally:
        executor.remove_node(consumer)
        executor.shutdown()
