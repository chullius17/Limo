"""Translation of ackermann_kinematics_critic.cpp, without pluginlib export."""
from ...include.python_ver.ackermann_kinematics_critic import AckermannKinematicsParameters
from ...include.python_ver.ros_compat import TrajectoryCritic, IllegalTrajectoryException, read_parameter

# Critic for kinematic feasibility evaluation

def commandRespectsAckermann(linear_x, linear_y, angular_z, min_turning_radius,
                             lateral_tolerance, stopped_velocity):
    if abs(linear_y) > lateral_tolerance:
        return False
    if abs(angular_z) <= lateral_tolerance:
        return True
    if abs(linear_x) <= stopped_velocity:
        return False
    return abs(linear_x / angular_z) + 1e-9 >= min_turning_radius


class AckermannKinematicsCritic(TrajectoryCritic):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__dict__.update(vars(AckermannKinematicsParameters()))

    def onInit(self):
        prefix = self.dwb_plugin_name_ + '.' + self.name_ + '.'
        for name in ('min_turning_radius', 'lateral_tolerance', 'stopped_velocity'):
            setattr(self, name + '_', read_parameter(self.nh_, prefix + name, getattr(self, name + '_')))
        if self.min_turning_radius_ <= 0 or self.lateral_tolerance_ < 0 or self.stopped_velocity_ < 0:
            raise ValueError('Invalid Ackermann kinematics parameters')

    def scoreTrajectory(self, trajectory):
        v = trajectory.velocity
        if not commandRespectsAckermann(v.x, v.y, v.theta, self.min_turning_radius_,
                                        self.lateral_tolerance_, self.stopped_velocity_):
            raise IllegalTrajectoryException(self.name_, 'Command violates LIMO Ackermann constraints')
        return 0.0
