"""Run a small MPC example without ROS, from the workspace src directory.

    PYTHONPATH=ros2_ws python3 -m limo_dwb_critics.src.python_ver.demo
"""
import math
from ...include.python_ver.sampling_mpc import MpcConfig, MpcState
from .sampling_mpc import SamplingMpc


def main():
    config = MpcConfig(time_steps=20, control_segments=4, batch_size=44,
                       velocity_samples=3, curvature_samples=7)
    mpc = SamplingMpc(config)
    initial = MpcState()

    def environment_cost(rollout, remaining):
        final = rollout.states[-1]
        return 5.0 * math.hypot(1.0 - final.x, final.y) + abs(final.yaw)

    solution = mpc.solve(initial, environment_cost)
    command = solution.rollout.states[1].control
    final = solution.rollout.states[-1]
    print('Selected family: {}'.format(solution.family.name))
    print('Candidates: {}; cost: {:.6f}'.format(solution.generated_count, solution.cost))
    print('First command: velocity={:.4f} m/s, steering={:.4f} rad, yaw_rate={:.4f} rad/s'.format(
        command.velocity, command.steering, mpc.yawRate(command)))
    print('Predicted final pose: x={:.4f}, y={:.4f}, yaw={:.4f}'.format(final.x, final.y, final.yaw))
    return solution


if __name__ == '__main__':
    main()
