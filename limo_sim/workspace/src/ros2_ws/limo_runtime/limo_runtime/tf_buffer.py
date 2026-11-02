# Copyright 2026 Giulio Cataldo
# Licensed under the Apache License, Version 2.0.

"""Batch TF reception and refresh on demand for current robot pose queries."""

import time

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class ConditionalCallbackGroup(MutuallyExclusiveCallbackGroup):
    """Exclude inactive subscriptions from the executor's wait set."""

    def __init__(self, enabled):
        super().__init__()
        self.enabled = enabled

    def can_execute(self, entity):
        return self.enabled() and super().can_execute(entity)

    def beginning_execution(self, entity):
        return self.enabled() and super().beginning_execution(entity)


class QueryTransformBuffer(Buffer):
    """Drain queued TF before queries and in inexpensive idle batches.

    Use for current/latest poses. Idle maintenance at 5 Hz keeps the regular TF
    history warm without waking the application executor for every publication.
    Active queries drain the queue immediately, at their original timestamps.
    The receiver node never joins an executor. Queries take queued messages
    directly, without rebuilding Foxy's Python wait set for every TF message.
    All access and close() must run on the caller's single executor thread.
    """

    def __init__(self, parent):
        super().__init__()
        name = parent.get_name() + '_query_tf'
        self._receiver = Node(
            name, context=parent.context, namespace=parent.get_namespace(),
            cli_args=['--ros-args', '-r', '__node:=' + name],
            enable_rosout=False, start_parameter_services=False)
        self._listener = TransformListener(self, self._receiver)
        self._closed = False
        self._parent = parent
        self._last_refresh = time.monotonic()
        self._maintenance_timer = parent.create_timer(0.2, self._maintain)

    def _maintain(self):
        if time.monotonic() - self._last_refresh >= 0.18:
            self.refresh()

    @staticmethod
    def _take(subscription):
        """Isolate Foxy's take API; newer rclpy exposes it on Subscription."""
        if hasattr(subscription, 'take_message'):
            result = subscription.take_message()
        else:
            from rclpy.impl.implementation_singleton import rclpy_implementation
            with subscription.handle as handle:
                result = rclpy_implementation.rclpy_take(
                    handle, subscription.msg_type, subscription.raw)
        return result[0] if result is not None else None

    def refresh(self):
        """Process available TF without waiting or running application callbacks."""
        # Bound work even if publishers continuously outpace this consumer.
        for subscription in (self._listener.tf_static_sub, self._listener.tf_sub):
            for _ in range(2048):
                message = self._take(subscription)
                if message is None:
                    break
                subscription.callback(message)
        self._last_refresh = time.monotonic()

    def can_transform(self, target_frame, source_frame, stamp,
                      timeout=Duration(), return_debug_tuple=False):
        """Refresh first, including during a bounded wait for missing TF."""
        deadline = time.monotonic() + max(0.0, timeout.nanoseconds * 1e-9)
        while True:
            self.refresh()
            result = super().can_transform(
                target_frame, source_frame, stamp,
                return_debug_tuple=return_debug_tuple)
            available = result[0] if return_debug_tuple else result
            remaining = deadline - time.monotonic()
            if available or remaining <= 0.0 or not self._receiver.context.ok():
                return result
            time.sleep(min(0.005, remaining))

    def close(self):
        """Release the receiver and its subscriptions before ROS shutdown."""
        if not self._closed:
            self._parent.destroy_timer(self._maintenance_timer)
            self._listener.unregister()
            self._receiver.destroy_node()
            self._closed = True
