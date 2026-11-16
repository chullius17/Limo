# Copyright 2026 Giulio Cataldo
# SPDX-License-Identifier: Apache-2.0
"""Translation of controller_server_main.cpp using an injected server backend.

Nav2 Foxy does not expose its C++ ControllerServer constructor as a Python
class. The backend supplies that external dependency, its executor, and its ROS
context; this module preserves shutdown ordering and signal handling.
"""
import os
import signal
import sys
from typing import Protocol

PRIMARY_STATE_UNCONFIGURED = 1
PRIMARY_STATE_INACTIVE = 2
PRIMARY_STATE_ACTIVE = 3
stop_requested = False


class ControllerServerBackend(Protocol):
    """ROS integration contract; lifecycle transition methods return state IDs."""
    def init(self, args): ...
    def uninstall_signal_handlers(self): ...
    def ok(self) -> bool: ...
    def create_controller_server(self): ...
    def create_executor(self): ...
    def shutdown(self): ...


def request_stop(signum, frame=None):
    global stop_requested
    stop_requested = True


def _state_id(value):
    # Accept either a raw ID or an rclpy-style lifecycle state object.
    if isinstance(value, int):
        return value
    state_id = value.id
    return state_id() if callable(state_id) else state_id


def main(argv=None, backend=None):
    """Run the server with a supplied ROS host and return EXIT_SUCCESS/FAILURE.

    backend is explicit because the C++ server is not importable in Python.
    See demo.py for a standalone example of the translated MPC algorithms.
    """
    if backend is None:
        raise RuntimeError('A ControllerServerBackend is required: Nav2 ControllerServer is a C++ service. '
                           'Use python -m limo_dwb_critics.src.python_ver.demo for the standalone example.')
    global stop_requested
    stop_requested = False
    backend.init(sys.argv if argv is None else argv)
    backend.uninstall_signal_handlers()
    previous_handler = signal.signal(signal.SIGINT, request_stop)
    # SIGTERM retains its existing disposition, as in the C++ launcher.
    result, node = 0, None
    try:
        node = backend.create_controller_server()
        executor = backend.create_executor()
        executor.add_node(node)
        while not stop_requested and backend.ok():
            executor.spin_once(timeout_sec=0.1)
        executor.remove_node(node)
        if backend.ok():
            node.get_logger().info('Stopping controller before ROS shutdown')
            if _state_id(node.get_current_state()) == PRIMARY_STATE_ACTIVE:
                if _state_id(node.deactivate()) != PRIMARY_STATE_INACTIVE:
                    raise RuntimeError('Controller deactivation failed')
            if _state_id(node.get_current_state()) == PRIMARY_STATE_INACTIVE:
                if _state_id(node.cleanup()) != PRIMARY_STATE_UNCONFIGURED:
                    raise RuntimeError('Controller cleanup failed')
    except Exception as error:
        print('Controller server failed: {}'.format(error), file=sys.stderr)
        result = 1
    finally:
        backend.shutdown()
        node = None
        signal.signal(signal.SIGINT, previous_handler)
    if os.environ.get('LIMO_FAST_EXIT') == '1':
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(result)  # Opt-in immediate exit only after lifecycle/ROS cleanup.
    return result


if __name__ == '__main__':
    sys.exit(main())
