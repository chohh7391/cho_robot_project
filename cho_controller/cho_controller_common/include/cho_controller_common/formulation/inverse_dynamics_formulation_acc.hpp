#pragma once

#include "cho_controller_common/formulation/inverse_dynamics_formulation_base.hpp"
#include "cho_controller_common/math/constraint_equality.hpp"
#include "cho_controller_common/math/constraint_inequality.hpp"
#include "cho_controller_common/math/constraint_bound.hpp"
#include "cho_controller_common/formulation/contact_level.hpp"

#include <vector>
#include <iostream>

namespace cho_controller {
namespace common {
namespace formulation {

using namespace cho_controller::common::math;
using namespace cho_controller::common::tasks;

class ContactTransitionInfo
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    double time_start;
    double time_end;
    double fMax_start;  /// max normal force at time time_start
    double fMax_end;    /// max normal force at time time_end
    std::shared_ptr<ContactLevel> contactLevel;
};

class InverseDynamicsFormulationAccForce: public InverseDynamicsFormulationBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef pinocchio::Data Data;
    typedef math::Vector Vector;
    typedef math::Matrix Matrix;
    typedef math::ConstRefVector ConstRefVector;
    typedef tasks::TaskBase TaskBase;
    typedef tasks::TaskMotion TaskMotion;
    typedef solver::HQPOutput HQPOutput;
    typedef tasks::TaskContactForce TaskContactForce;


    InverseDynamicsFormulationAccForce(const std::string & name, RobotWrapper & robot, bool verbose=false);

    Data & data() ;

    unsigned int nVar() const;
    unsigned int nEq() const;
    unsigned int nIn() const;

    bool addMotionTask(TaskMotion & task, double weight, unsigned int priorityLevel, double transition_duration=0.0);

    bool updateTaskWeight(const std::string & task_name, double weight);

    bool removeTask(const std::string & taskName, double transition_duration=0.0);

    const HQPData & computeProblemData(double time, ConstRefVector q, ConstRefVector v);

    const Vector & getAccelerations(const HQPOutput & sol);

    const Vector & getActuatorForces(const HQPOutput & sol);

    bool addForceTask(TaskContactForce & task, double weight, unsigned int priorityLevel, double transition_duration=0.0);

    bool addRigidContact(ContactBase & contact, double force_regularization_weight, double motion_weight=1.0, unsigned int motion_priority_level=0);

    bool addRigidContact(ContactBase & contact);

    bool updateRigidContactWeights(const std::string & contact_name, double force_regularization_weight, double motion_weight=-1.0);

    bool removeRigidContact(const std::string & contactName, double transition_duration=0.0);

    const Vector & getContactForces(const HQPOutput & sol);

    Vector getContactForces(const std::string & name, const HQPOutput & sol);

    bool getContactForces(const std::string & name, const HQPOutput & sol, RefVector f);

public:

    template<class TaskLevelPointer>
    void addTask(TaskLevelPointer task, double weight, unsigned int priorityLevel);

    void resizeHqpData();

    bool removeFromHqpData(const std::string & name);

    bool decodeSolution(const HQPOutput & sol);

    void resetHqpData();

    Data m_data;
    HQPData m_hqpData;
    std::vector<std::shared_ptr<TaskLevel> >        m_taskMotions;
    std::vector<std::shared_ptr<ContactLevel> >     m_contacts;
    std::vector<std::shared_ptr<TaskLevelForce> >   m_taskContactForces;

    double m_t;         /// time
    unsigned int m_v;   /// number of acceleration variables
    unsigned int m_k;
    unsigned int m_eq;  /// number of equality constraints
    unsigned int m_in;  /// number of inequality constraints
    Matrix m_Jc;        /// contact force Jacobian
    std::shared_ptr<math::ConstraintEquality> m_baseDynamics;

    bool m_solutionDecoded;
    Vector m_dv;
    Vector m_f;
    Vector m_tau;

    bool m_mobile;

    std::vector<std::shared_ptr<ContactTransitionInfo> > m_contactTransitions;
};

} // namespace formulation
} // namespace common
} // namespace cho_controller