# Family is assigned by the sampler, not inferred from trajectory geometry.
uint8 BRAKING=0
uint8 NOMINAL=1
uint8 CONSTANT_CURVATURE=2
uint8 PERTURBED_NOMINAL=3
uint8 BROAD_EXPLORATION=4

# SCORED can include DWB cost short-circuiting; it is not a safety certificate.
uint8 COST_PRUNED=0
uint8 SCORED=1
uint8 REJECTED=2

uint32 candidate_id
uint8 family
uint8 score_status
float64 total_cost
geometry_msgs/Pose2D[] poses
