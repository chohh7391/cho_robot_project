
#pragma once

#include <Eigen/Core>
#include "cho_trajectory_smoother/Path.h"

namespace cho_trajectory {
class Trajectory
{
public:
	// Generates a time-optimal trajectory
	Trajectory(const Path &path, const Eigen::VectorXd &maxVelocity, const Eigen::VectorXd &maxAcceleration, double timeStep = 0.001);
	
	~Trajectory(void);

	// Call this method after constructing the object to make sure the trajectory generation succeeded without errors.
	// If this method returns false, all other methods have undefined behavior.
	bool isValid() const;

	// Returns the optimal duration of the trajectory
	double getDuration() const;

	// Return the position/configuration or velocity vector of the robot for a given point in time within the trajectory.
	Eigen::VectorXd getPosition(double time) const;
	Eigen::VectorXd getVelocity(double time) const;

	// Outputs the phase trajectory and the velocity limit curve in 2 files for debugging purposes.
	void outputPhasePlaneTrajectory() const;

private:
	struct TrajectoryStep {
		TrajectoryStep() {}
		TrajectoryStep(double pathPos, double pathVel) :
			pathPos(pathPos),
			pathVel(pathVel)
		{}
		double pathPos;
		double pathVel;
		double time;
	};

	bool getNextSwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration);
	bool getNextAccelerationSwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration);
	bool getNextVelocitySwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration);
	bool integrateForward(std::list<TrajectoryStep> &trajectory, double acceleration);
	void integrateBackward(std::list<TrajectoryStep> &startTrajectory, double pathPos, double pathVel, double acceleration);
	double getMinMaxPathAcceleration(double pathPosition, double pathVelocity, bool max);
	double getMinMaxPhaseSlope(double pathPosition, double pathVelocity, bool max);
	double getAccelerationMaxPathVelocity(double pathPos) const;
	double getVelocityMaxPathVelocity(double pathPos) const;
	double getAccelerationMaxPathVelocityDeriv(double pathPos);
	double getVelocityMaxPathVelocityDeriv(double pathPos);
	
	std::list<TrajectoryStep>::const_iterator getTrajectorySegment(double time) const;
	
	Path path;
	Eigen::VectorXd maxVelocity;
	Eigen::VectorXd maxAcceleration;
	unsigned int n;
	bool valid;
	std::list<TrajectoryStep> trajectory;
	std::list<TrajectoryStep> endTrajectory; // non-empty only if the trajectory generation failed.

	static const double eps;
	const double timeStep;

	mutable double cachedTime;
	mutable std::list<TrajectoryStep>::const_iterator cachedTrajectorySegment;
};

} // namespace cho_trajectory
