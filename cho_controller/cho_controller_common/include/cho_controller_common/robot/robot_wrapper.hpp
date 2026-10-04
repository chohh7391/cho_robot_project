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
#include "cho_controller_common/robot/fwd.hpp"

#include <pinocchio/multibody.hpp>
#include <pinocchio/spatial/fwd.hpp>

#include <string>
#include <vector>


namespace cho_controller {
namespace common {
namespace robot {

class RobotWrapper{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef pinocchio::Model Model;
    typedef pinocchio::Data Data;
    typedef pinocchio::Motion Motion;
    typedef pinocchio::Frame Frame;
    typedef pinocchio::SE3 SE3;
    typedef math::Vector  Vector;
    typedef math::Vector3 Vector3;
    typedef math::Vector6 Vector6;
    typedef math::Matrix Matrix;
    typedef math::Matrix3x Matrix3x;
    typedef math::Matrix6d Matrix6d;
    typedef math::RefVector RefVector;
    typedef math::ConstRefVector ConstRefVector;
    

    RobotWrapper(const std::string & filename, bool verbose=false);
    RobotWrapper(const std::string & xml_string, bool is_xml, bool verbose = false);
    ~RobotWrapper(){};
    
    virtual int nq() const;
    virtual int nv() const;
    virtual int na() const;

    const Model & model() const;
    Model & model();

    void computeAllTerms(Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v);
    
    const Eigen::Vector3d & com(const Data & data) const;

    // References into `data`, valid until the next computeAllTerms(); no copy
    // in the control loop.
    const Eigen::VectorXd & nonLinearEffects(const Data & data) const;

    const Eigen::VectorXd & GeneralizedGravity(const Data & data) const;

    const SE3 & position(const Data & data, const Model::JointIndex index) const;

    const Eigen::MatrixXd & mass(const Data & data);

    const Eigen::MatrixXd & mass_inverse(const Data & data);

    const Motion & velocity(const Data & data, const Model::JointIndex index) const;

    const Motion & acceleration(const Data & data, const Model::JointIndex index) const;

    void jacobianWorld(const Data & data, const Model::JointIndex index, Data::Matrix6x & J);

    SE3 framePosition(const Data & data, const Model::FrameIndex index) const;

    void framePosition(const Data & data, const Model::FrameIndex index, SE3 & framePosition) const;

    Motion frameVelocity(const Data & data, const Model::FrameIndex index) const;

    void frameVelocity(const Data & data, const Model::FrameIndex index, Motion & frameVelocity) const;

    Motion frameAcceleration(const Data & data, const Model::FrameIndex index) const;

    void frameAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const;

    Motion frameClassicAcceleration(const Data & data, const Model::FrameIndex index) const;

    void frameClassicAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const;

    void frameJacobianLocal(Data & data, const Model::FrameIndex index, Data::Matrix6x & J);

    void frameJacobianWorldAligned(Data & data, const Model::FrameIndex index, Data::Matrix6x & J);


protected:
    Model m_model;
    std::string m_model_filename;
    bool m_verbose;
    int m_na;
    Eigen::MatrixXd m_M, m_Minv;
    Eigen::MatrixXd m_S, m_S_dot;
    Matrix6d m_Rot;
    double r_, b_, d_, c_;
    Eigen::VectorXd m_q;
};

} // namespace robot
} // namespace common
} // namespace cho_controller
