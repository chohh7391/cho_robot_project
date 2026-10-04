// Copyright (c) 2017 CNRS
// Copyright 2026 Hyunho Cho
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Derived from TSID (https://github.com/stack-of-tasks/tsid) and modified for
// cho_robot_project; see NOTICE.

#pragma once

#include "cho_controller_common/math/fwd.hpp"
#include "cho_controller_common/robot/robot_wrapper.hpp"
#include "cho_controller_common/tasks/task_motion.hpp"
#include "cho_controller_common/solver/solver_HQP_base.hpp"
#include "cho_controller_common/contact/contact_base.hpp"
#include "cho_controller_common/tasks/task_contact_force.hpp"
//#include "cho_controller_common/solver/util.hpp"

#include <string>

namespace cho_controller {
namespace common {
namespace formulation {

using namespace cho_controller::common::math;
using namespace cho_controller::common::tasks;
using namespace cho_controller::common::solver;
using namespace cho_controller::common::robot;
using namespace cho_controller::common::contact;

struct TaskLevel
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    tasks::TaskBase & task;
    std::shared_ptr<math::ConstraintBase> constraint;
    unsigned int priority;

    TaskLevel(tasks::TaskBase & t, unsigned int priority);
};

struct TaskLevelForce
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    tasks::TaskContactForce & task;
    std::shared_ptr<math::ConstraintBase> constraint;
    unsigned int priority;

    TaskLevelForce(tasks::TaskContactForce & task, unsigned int priority);
};

class InverseDynamicsFormulationBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef pinocchio::Data Data;
    typedef math::Vector Vector;
    typedef math::RefVector RefVector;
    typedef math::ConstRefVector ConstRefVector;
    typedef tasks::TaskMotion TaskMotion;
    typedef tasks::TaskBase TaskBase;
    typedef solver::HQPData HQPData;
    typedef solver::HQPOutput HQPOutput;
    typedef robot::RobotWrapper RobotWrapper;
    typedef contact::ContactBase ContactBase;
    typedef tasks::TaskContactForce TaskContactForce;

    InverseDynamicsFormulationBase(const std::string & name, RobotWrapper & robot, bool verbose=false);

    // virtual Data & data() = 0;

    // virtual unsigned int nVar() const = 0;
    // virtual unsigned int nEq() const = 0;
    // virtual unsigned int nIn() const = 0;

    // virtual bool addMotionTask(TaskMotion & task, double weight, unsigned int priorityLevel, double transition_duration=0.0) = 0;

    // virtual bool updateTaskWeight(const std::string & task_name, double weight) = 0;

    // virtual bool removeTask(const std::string & taskName, double transition_duration=0.0) = 0;


    // virtual const HQPData & computeProblemData(double time, ConstRefVector q, ConstRefVector v) = 0;

    // virtual const Vector & getAccelerations(const HQPOutput & sol) = 0;

    // // 
    // virtual bool addForceTask(TaskContactForce & task,
    //                       double weight,
    //                       unsigned int priorityLevel,
    //                       double transition_duration=0.0) = 0;

    // virtual bool addRigidContact(ContactBase & contact);

    // virtual bool addRigidContact(ContactBase & contact, double force_regularization_weight,
    //                          double motion_weight=1.0, unsigned int motion_priority_level=0) = 0;


    // virtual bool updateRigidContactWeights(const std::string & contact_name,
    //                                    double force_regularization_weight,
    //                                    double motion_weight=-1.0) = 0;

    // virtual bool removeRigidContact(const std::string & contactName,
    //                             double transition_duration=0.0) = 0;

    // virtual const Vector & getContactForces(const HQPOutput & sol) = 0;
    // virtual bool getContactForces(const std::string & name, const HQPOutput & sol, RefVector f) = 0;

protected:
    std::string m_name;
    RobotWrapper m_robot;
    bool m_verbose;
};

} // namespace formulation
} // namespace common
} // namespace cho_controller