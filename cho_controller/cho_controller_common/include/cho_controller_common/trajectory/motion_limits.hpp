#pragma once

#include <Eigen/Core>

namespace cho_controller {
namespace common {
namespace trajectory {

// Bounds for a point-to-point motion; 0 marks a bound that is not set. Loaded
// from parameters by motion_limits_params.hpp.

// Per joint.
struct JointMotionLimits
{
    Eigen::VectorXd max_velocity, max_acceleration, max_jerk;
};

// Translation along the straight line and rotation about the geodesic axis.
struct CartesianMotionLimits
{
    double max_trans_vel {0.0}, max_trans_acc {0.0}, max_trans_jerk {0.0};
    double max_rot_vel {0.0}, max_rot_acc {0.0}, max_rot_jerk {0.0};
};

} // namespace trajectory
} // namespace common
} // namespace cho_controller
