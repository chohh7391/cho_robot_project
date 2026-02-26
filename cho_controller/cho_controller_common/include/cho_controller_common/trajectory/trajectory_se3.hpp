#pragma once

#include "cho_controller_common/trajectory/trajectory_base.hpp"

#include <pinocchio/spatial/se3.hpp>
#include <cho_trajectory_smoother/Path.h>
#include <cho_trajectory_smoother/Trajectory.h>

using namespace cho_trajectory;

namespace cho_controller {
namespace common {
namespace trajectory {
class TrajectorySE3Constant : public TrajectoryBase
{
	public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef pinocchio::SE3 SE3;

	TrajectorySE3Constant(const std::string & name);

	TrajectorySE3Constant(const std::string & name, const SE3 & M);

	unsigned int size() const;

	void setReference(const SE3 & M);

	const TrajectorySample & operator()(double time);

	const TrajectorySample & computeNext();

	void getLastSample(TrajectorySample & sample) const;

	bool has_trajectory_ended() const;

	const std::vector<Eigen::VectorXd> & getWholeTrajectory();

	protected:
		SE3    m_M;
		std::vector<Eigen::VectorXd> traj_;

};

class TrajectorySE3Cubic : public TrajectoryBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef pinocchio::SE3 SE3;

	TrajectorySE3Cubic(const std::string & name);
	TrajectorySE3Cubic(const std::string & name, const SE3 & init_M, const SE3 & goal_M, const double & duration, const double & stime);

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
	const std::vector<Eigen::VectorXd> & getWholeTrajectory();

	double cubic(double time,    ///< Current time
				double time_0,  ///< Start time
				double time_f,  ///< End time
				double x_0,     ///< Start state
				double x_f,     ///< End state
				double x_dot_0, ///< Start state dot
				double x_dot_f  ///< End state dot
	)
	{
		double x_t;

		if (time < time_0)
		{
		x_t = x_0;
		}
		else if (time > time_f)
		{
		x_t = x_f;
		}
		else
		{
		double elapsed_time = time - time_0;
		double total_time = time_f - time_0;
		double total_time2 = total_time * total_time;  // pow(t,2)
		double total_time3 = total_time2 * total_time; // pow(t,3)
		double total_x = x_f - x_0;

		x_t = x_0 + x_dot_0 * elapsed_time

				+ (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time * elapsed_time

				+ (-2 * total_x / total_time3 +
				(x_dot_0 + x_dot_f) / total_time2) *
					elapsed_time * elapsed_time * elapsed_time;
		}
		return x_t;
	}
	
protected:
	SE3 m_ref;
	SE3 m_init, m_goal, m_cubic;
	double m_duration, m_stime, m_time;
	std::vector<Eigen::VectorXd> traj_;

};

class TrajectorySE3Timeopt : public TrajectoryBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef pinocchio::SE3 SE3;

	TrajectorySE3Timeopt(const std::string & name);
	TrajectorySE3Timeopt(const std::string & name, Eigen::Vector3d & MaxVel, Eigen::Vector3d & MaxAcc);

	unsigned int size() const;
	const TrajectorySample & operator()(double time);
	const TrajectorySample & computeNext();
	void getLastSample(TrajectorySample & sample) const;
	bool has_trajectory_ended() const;
	void setReference(const SE3 & ref);
	void addWaypoint(const SE3 &  waypoints);
	void clearWaypoints();
	void setCurrentTime(const double & time);
	void setStartTime(const double & time);
	void setMaxVelocity(const Eigen::Vector3d  &  MaxVel);
	void setMaxAcceleration(const Eigen::Vector3d  &  MaxAcc);
	const std::vector<Eigen::VectorXd> & getWholeTrajectory();

	double cubic(double time,    ///< Current time
				double time_0,  ///< Start time
				double time_f,  ///< End time
				double x_0,     ///< Start state
				double x_f,     ///< End state
				double x_dot_0, ///< Start state dot
				double x_dot_f  ///< End state dot
	)
	{
		double x_t;

		if (time < time_0)
		{
		x_t = x_0;
		}
		else if (time > time_f)
		{
		x_t = x_f;
		}
		else
		{
		double elapsed_time = time - time_0;
		double total_time = time_f - time_0;
		double total_time2 = total_time * total_time;  // pow(t,2)
		double total_time3 = total_time2 * total_time; // pow(t,3)
		double total_x = x_f - x_0;

		x_t = x_0 + x_dot_0 * elapsed_time

				+ (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time * elapsed_time

				+ (-2 * total_x / total_time3 +
				(x_dot_0 + x_dot_f) / total_time2) *
					elapsed_time * elapsed_time * elapsed_time;
		}
		return x_t;
	}
	
protected:
	SE3 m_ref;
	SE3 m_init, m_goal, m_cubic;
	std::vector<SE3> m_SE3_waypoints;
	int m_size;
	Eigen::Vector3d m_maxvel, m_maxacc;
	double m_duration, m_stime, m_time;
	stdlist_Eigenvec m_waypoints;
	bool m_calc;
	Trajectory* m_traj;
	std::vector<Eigen::VectorXd> traj_;

};

} // namespace trajectory
} // namespace common
} // namespace cho_controller
