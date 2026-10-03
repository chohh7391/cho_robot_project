//
// Copyright (c) 2017 CNRS
//
// SPDX-License-Identifier: BSD-2-Clause
//
// Derived from TSID (https://github.com/stack-of-tasks/tsid) and modified for
// cho_robot_project. Full license text: LICENSES/BSD-2-Clause-TSID.txt
//
#pragma once

#include <memory>

#include "cho_controller_common/trajectory/motion_limits.hpp"
#include "cho_controller_common/trajectory/trajectory_base.hpp"

#include <pinocchio/spatial/se3.hpp>


namespace cho_controller {
namespace common {
namespace trajectory {

class PointToPoint;

// Rest-to-rest pose motion: a straight line in translation and the geodesic in
// rotation, R(t) = R_init exp(axis * angle(t)). Ruckig plans two coordinates,
// the distance along the line and the angle about the axis, within the
// Cartesian limits, phase-synchronised so both arrive together and in
// proportion, and the plan is slowed uniformly to the requested duration (see
// point_to_point.hpp). The duration is a minimum, as for the joint version.
//
// Sample layout: pos = [translation; rotation column-major (9)], vel/acc =
// [linear; angular], all world-aligned: the twist TaskSE3Equality expects.
class TrajectorySE3Ruckig : public TrajectoryBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef pinocchio::SE3 SE3;

	TrajectorySE3Ruckig(const std::string & name);
	TrajectorySE3Ruckig(const std::string & name, const SE3 & init_M, const SE3 & goal_M, const double & duration,
						const double & stime);
	~TrajectorySE3Ruckig();

	unsigned int size() const;
	const TrajectorySample & operator()(double time);
	const TrajectorySample & computeNext();
	void getLastSample(TrajectorySample & sample) const;
	bool has_trajectory_ended() const;
	void setReference(const SE3 & ref);
	void setInitSample(const SE3 & init_M);
	void setGoalSample(const SE3 & goal_M);
	void setDuration(const double & duration);
	void setCurrentTime(const double & time);
	void setStartTime(const double & time);
	// Unset (0) bounds leave the requested duration exact.
	void setLimits(const CartesianMotionLimits & limits);
	// The duration the motion will take; plans first if anything changed.
	double getDuration();
	const std::vector<Eigen::VectorXd> & getWholeTrajectory();

protected:
	void plan();

	SE3 m_init {SE3::Identity()}, m_goal {SE3::Identity()};
	double m_duration {0.0}, m_stime {0.0}, m_time {0.0};
	CartesianMotionLimits m_limits;
	bool m_dirty {true};
	// The line and the geodesic of the current plan, unit vectors.
	Eigen::Vector3d m_direction {Eigen::Vector3d::Zero()}, m_axis {Eigen::Vector3d::Zero()};
	std::unique_ptr<PointToPoint> m_motion;
	// Scratch for sampling the two coordinates without allocating.
	Eigen::Vector2d m_s, m_sd, m_sdd;
	std::vector<Eigen::VectorXd> traj_;
};

} // namespace trajectory
} // namespace common
} // namespace cho_controller
