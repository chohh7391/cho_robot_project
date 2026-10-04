// Copyright 2026 Hyunho Cho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
