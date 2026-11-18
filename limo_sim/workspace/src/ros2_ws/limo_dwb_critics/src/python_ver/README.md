# Python translations of limo_dwb_critics

These files translate all ten C++ sources in `src/` and all eight headers in
`include/limo_dwb_critics/` into Python for reading and standalone experiments.
Comments and docstrings are in English. The existing C++ sources and build
configuration remain the implementation used by ROS.

The model, candidate generation, costs, grid guidance, telemetry
and controller state transitions are implemented. External ROS/Nav2 services
use explicit adapters: these Python classes are not loadable C++ plugins.

## Where to start

1. Read `../../include/python_ver/sampling_mpc.py` for the model's input and output
   values. In Python, `@dataclass` plays the role of a simple C++ `struct`.
2. Read `sampling_mpc.py`, starting with `yawRate`, `advance` and `rollout`, then
   `solve`. They describe the bicycle model and the candidate search.
3. Read `ackermann_kinematics_critic.py` and `mppi_cost_critics.py` to see which
   commands are rejected and how legal predictions receive costs.
4. Read `obstacle_guidance.py` for the Dijkstra field used around obstacles.
5. Read `ackermann_mpc_controller.py` to follow a complete control cycle.
6. Read the debug and timing modules for diagnostics.

## Source correspondence

The names below are relative to the package. Python counterparts retain their
original file stems and public method names, including camelCase.

| Original C++ files | Python header counterpart | Python implementation |
| --- | --- | --- |
| `ackermann_kinematics_critic.hpp` and `.cpp` | `include/python_ver/ackermann_kinematics_critic.py` | `src/python_ver/ackermann_kinematics_critic.py` |
| `ackermann_mpc_controller.hpp` and `.cpp` | `include/python_ver/ackermann_mpc_controller.py` | `src/python_ver/ackermann_mpc_controller.py` |
| `sampling_mpc.hpp` and `.cpp` | `include/python_ver/sampling_mpc.py` | `src/python_ver/sampling_mpc.py` |
| `obstacle_guidance.hpp` and `.cpp` | `include/python_ver/obstacle_guidance.py` | `src/python_ver/obstacle_guidance.py` |
| `smac_guidance.cpp` | Uses the path critic interface | `src/python_ver/smac_guidance.py` |
| `mppi_cost_critics.hpp` and `.cpp` | `include/python_ver/mppi_cost_critics.py` | `src/python_ver/mppi_cost_critics.py` |
| `mpc_debug.hpp` and `.cpp` | `include/python_ver/mpc_debug.py` | `src/python_ver/mpc_debug.py` |
| `mpc_timing.hpp` and `.cpp` | `include/python_ver/mpc_timing.py` | `src/python_ver/mpc_timing.py` |
| `mpc_timing_scope.hpp` | `include/python_ver/mpc_timing_scope.py`, including its inline implementation | Header-only in C++ |
| `mpc_prepare_timing.cpp` | Uses the controller and timing declarations | `src/python_ver/mpc_prepare_timing.py` |
| `controller_server_main.cpp` | Backend protocol is in the implementation module | `src/python_ver/controller_server_main.py` |

Python does not require headers. The two requested folders preserve the original
organization: data and interfaces in `include/python_ver`, algorithms in
`src/python_ver`. A `Protocol` describes available methods; instantiate runtime
classes from `src.python_ver`, not their header protocols. `MpcConfig.validate`
is located on its Python dataclass, keeping configuration and validation together.

## Running without ROS

The default Dijkstra backend runs standalone. With `ObstacleGuidance.use_smac`
enabled, inject a configured native Nav2 planner into
`MppiPathCritic(smac_planner=adapter, ...)`; it must expose
`createPlan(start, goal)`. Missing adapters raise an explicit error. The C++ ROS
plugin loads actual SMAC automatically; the Python helpers translate route
validation and scoring without substituting a different search for SMAC.

Use Python 3.8 or newer. No third-party Python packages are required. From the
workspace's `src` directory, which contains `ros2_ws`, run:

```bash
PYTHONPATH=ros2_ws python3 -m limo_dwb_critics.src.python_ver.demo
PYTHONPATH=ros2_ws python3 -m unittest limo_dwb_critics.src.python_ver.test_translation -v
```

Run modules with `-m` so imports between the two `python_ver` packages resolve.
To import them from another program, add the absolute path to `ros2_ws` to that
program's Python import path.

A minimal model example:

```python
from limo_dwb_critics.include.python_ver.sampling_mpc import (
    MpcConfig, MpcControl, MpcState,
)
from limo_dwb_critics.src.python_ver.sampling_mpc import SamplingMpc

mpc = SamplingMpc(MpcConfig())
initial = MpcState()
targets = [MpcControl(velocity=0.2, steering=0.1) for _ in range(50)]
prediction = mpc.rollout(initial, targets)
print(prediction.states[-1])
```

## How C++ constructs map to Python

- `struct` becomes `@dataclass`; `std::vector` becomes a list.
- `std::function` becomes a Python callable; lambdas can also become nested functions.
- `std::unique_ptr` and `shared_ptr` become object references; `nullptr` becomes `None`.
- `std::lock_guard` becomes `with lock:`. The C++ timing destructor becomes
  `with MpcTimingScope(...)`, preserving measurement during exception handling.
- `std::invalid_argument` becomes `ValueError`. Trajectory and planner exceptions
  retain named Python equivalents.
- C++ output-reference arguments become return values. `worldToMap` returns
  `(x, y)` or `None`; path projection returns a result or `None`; the tube
  adapter's `projectProgress` returns `(progress, heading)` or `None`.
- Explicit copies preserve important C++ value semantics, so predicted states,
  plans and debug snapshots do not accidentally share mutable objects.
- Trajectory time offsets are seconds; the standalone node's timestamps are
  integer nanoseconds. Diagnostic clocks use monotonic seconds internally.

## External dependencies and scope

`include/python_ver/ros_compat.py` supplies message dataclasses, a byte costmap,
parameter access, an in-process node and depth-one memory publishers.
`include/python_ver/nav2_compat.py` supplies path association and convex footprint
rasterization helpers. These support reading and testing the translated code;
they do not supply ROS transport or TF lookup.

`dwb_adapter.py` implements the external DWB calls used in the standalone
controller. Its plan clipping is a teaching substitute for Foxy's full DWB
transform and prune pipeline. A TF adapter must provide
`transform_pose(pose_stamped, target_frame, tolerance)` for frame changes.
`LocalNode` has no ROS subscriptions or DDS QoS; subscription callbacks can be
called directly in examples. A real ROS host must provide those services.

The tube layer belongs to `limo_inflation`, outside the translated package.
The `PathDistanceLayer` protocol documents its required methods. Tube mode
requires an injected compatible layer and the full-footprint obstacle critic.
The controller-server entry point similarly requires a `ControllerServerBackend`:
Python cannot directly construct Nav2's C++ `ControllerServer`. Its lifecycle and
shutdown sequence are translated, while the server and executor remain external.

Fixed-seed Python sampling is reproducible, but Python and C++ random distribution
implementations generate different perturbations. Performance and live Nav2
behavior are not inferred from these standalone tests. No Python classes are
registered in the existing plugin XML files or build configuration.

## Verification

The local suite checks all 17 translated modules can be imported, model limits,
value-copy behavior, reversal, failure resets, structured and startup families,
deduplication, grid routing, collision rejection, path detours,
debug snapshots, timing and controller lifecycle behavior.

A separate direct comparison compiled the original `sampling_mpc.cpp` and
`obstacle_guidance.cpp` and compared 2,154 scalar results: six rollouts, a
constant-curvature candidate search and three weighted obstacle grids including
subcell distance and heading queries. Those selected results matched exactly.
This comparison does not cover randomized candidate parity or live ROS execution.
