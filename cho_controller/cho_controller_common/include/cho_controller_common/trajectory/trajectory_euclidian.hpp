#pragma once

#include "cho_controller_common/trajectory/trajectory_base.hpp"
#include <cho_trajectory_smoother/Path.h>
#include <cho_trajectory_smoother/Trajectory.h>

using namespace cho_trajectory;

namespace cho_controller {
namespace common {
namespace trajectory {

class TrajectoryEuclidianConstant : public TrajectoryBase
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        typedef math::Vector         Vector;
        typedef math::ConstRefVector ConstRefVector;

        TrajectoryEuclidianConstant(const std::string & name);

        TrajectoryEuclidianConstant(const std::string & name, ConstRefVector ref);

        unsigned int size() const;

        void setReference(ConstRefVector ref);

        const TrajectorySample & operator()(double time);

        const TrajectorySample & computeNext();

        void getLastSample(TrajectorySample & sample) const;

        bool has_trajectory_ended() const;

        const std::vector<Eigen::VectorXd> & getWholeTrajectory();

    protected:
        Vector m_ref;
        std::vector<Eigen::VectorXd> traj_;
};

class TrajectoryEuclidianCubic : public TrajectoryBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef math::Vector         Vector;
    typedef math::ConstRefVector ConstRefVector;

    TrajectoryEuclidianCubic(const std::string & name);
    TrajectoryEuclidianCubic(const std::string & name, ConstRefVector init_M, ConstRefVector goal_M, const double & duration, const double & stime);

    unsigned int size() const;
    const TrajectorySample & operator()(double time);
    const TrajectorySample & computeNext();
    void getLastSample(TrajectorySample & sample) const;
    bool has_trajectory_ended() const;
    void setReference(ConstRefVector ref);
    void setInitSample(ConstRefVector init_M);
    void setGoalSample(ConstRefVector goal_M);
    void setDuration(const double & duration);
    void setCurrentTime(const double & time);
    void setStartTime(const double & time);
    const std::vector<Eigen::VectorXd> & getWholeTrajectory();

protected:
    Vector m_ref;
    Vector m_init, m_goal, m_cubic;
    double m_duration, m_stime, m_time;
    std::vector<Eigen::VectorXd> traj_;
};

class TrajectoryEuclidianTimeopt : public TrajectoryBase
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef math::Vector         Vector;
        typedef math::ConstRefVector ConstRefVector;

        TrajectoryEuclidianTimeopt(const std::string & name);
        TrajectoryEuclidianTimeopt(const std::string & name, ConstRefVector MaxVel, ConstRefVector MaxAcc);

        unsigned int size() const;
        const TrajectorySample & operator()(double time);
        const TrajectorySample & computeNext();

        void getLastSample(TrajectorySample & sample) const;
        bool has_trajectory_ended() const;
        void setReference(ConstRefVector ref);
        void addWaypoint(ConstRefVector waypoints);
        void clearWaypoints();
        void setCurrentTime(const double & time);
        void setStartTime(const double & time);
        void setMaxVelocity(ConstRefVector MaxVel);
        void setMaxAcceleration(ConstRefVector MaxAcc);
        const std::vector<Eigen::VectorXd> & getWholeTrajectory();

    protected:
        Vector m_ref;
        int m_size;
        Vector m_maxvel, m_maxacc;
        double m_duration, m_stime, m_time;
        stdlist_Eigenvec m_waypoints;
        bool m_calc;
        Trajectory* m_traj;
        std::vector<Eigen::VectorXd> traj_;

};

} // namespace trajectory
} // namespace common
} // namespace cho_controller
