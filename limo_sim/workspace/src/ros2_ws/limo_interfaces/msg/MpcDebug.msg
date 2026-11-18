# All poses and this costmap come from the same locked controller cycle.
std_msgs/Header header
string base_frame
geometry_msgs/Pose2D robot_pose
geometry_msgs/Polygon footprint
nav2_msgs/Costmap costmap
float64 wheelbase
float64 rear_axle_to_base
float64 model_dt
uint32 time_steps
uint32 generated_count
float64 initial_velocity
float64 initial_steering
float64 command_velocity
float64 command_steering
int32 selected_id
MpcCandidate[] candidates
# Active avoidance reference from this same cycle; empty when no detour is active.
string guidance_backend
geometry_msgs/Pose2D[] guidance_path
bool guidance_target_valid
geometry_msgs/Pose2D guidance_target
# Evaluated reference points, independent of planner activation or success.
bool path_lookahead_valid
geometry_msgs/Pose2D path_lookahead
bool guidance_lookahead_valid
geometry_msgs/Pose2D guidance_lookahead
bool guidance_candidate_target_valid
geometry_msgs/Pose2D guidance_candidate_target
