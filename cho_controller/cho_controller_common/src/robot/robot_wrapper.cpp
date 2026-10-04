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

#include "cho_controller_common/robot/robot_wrapper.hpp"

#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

using namespace pinocchio;
using namespace std;


namespace cho_controller {
    namespace common {
        namespace robot {
            RobotWrapper::RobotWrapper(const std::string & filename, bool verbose)
            :m_verbose(verbose)
            {
                pinocchio::urdf::buildModel(filename, m_model, m_verbose);
                m_model_filename = filename;
                m_na = nv();
            }

            RobotWrapper::RobotWrapper(const std::string & xml_string, bool is_xml, bool verbose)
            :m_verbose(verbose)
            {
                if (is_xml) {
                    pinocchio::urdf::buildModelFromXML(xml_string, m_model, m_verbose);
                    m_model_filename = "Loaded_from_ROS_Parameter";
                } else {
                    pinocchio::urdf::buildModel(xml_string, m_model, m_verbose);
                    m_model_filename = xml_string;
                }
                m_na = nv();
            }

            const Model & RobotWrapper::model() const { return m_model; }
            Model & RobotWrapper::model() { return m_model; }

            int RobotWrapper::nq() const { return m_model.nq; }
            int RobotWrapper::nv() const { return m_model.nv; }
            int RobotWrapper::na() const { return m_na; }
            

            void RobotWrapper::computeAllTerms(Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v) 
            {
                m_q = q;
                pinocchio::computeAllTerms(m_model, data, q, v);
                data.M.triangularView<Eigen::StrictlyLower>()
                        = data.M.transpose().triangularView<Eigen::StrictlyLower>();
                // computeAllTerms already fills com/vcom and the centroidal terms
                // (Ag, dAg, hg) but not the frame placements (oMf).
                pinocchio::updateFramePlacements(m_model, data);
            }

            const Eigen::Vector3d & RobotWrapper::com(const Data & data) const
            {
                return data.com[0];
            }

            const Eigen::VectorXd & RobotWrapper::nonLinearEffects(const Data & data) const
            {
                return data.nle;
            }

            const Eigen::VectorXd & RobotWrapper::GeneralizedGravity(const Data & data) const
            {
                return data.g;
            }

            const SE3 & RobotWrapper::position(const Data & data, const Model::JointIndex index) const
            {
                assert(index<data.oMi.size());
                return data.oMi[index];
            }

            const Motion & RobotWrapper::velocity(const Data & data, const Model::JointIndex index) const
            {
                assert(index<data.v.size());
                return data.v[index];
            }

            const Motion & RobotWrapper::acceleration(const Data & data, const Model::JointIndex index) const
            {
                assert(index<data.a.size());
                return data.a[index];
            }

            const Eigen::MatrixXd & RobotWrapper::mass(const Data & data)
            {            
                return data.M;
            }

            const Eigen::MatrixXd & RobotWrapper::mass_inverse(const Data & data){
                m_Minv = this->mass(data).inverse();
                return m_Minv;
            }

            void RobotWrapper::jacobianWorld(const Data & data, const Model::JointIndex index, Data::Matrix6x & J)
            {
                return pinocchio::getJointJacobian(m_model, data, index, pinocchio::WORLD, J) ;
            }

            SE3 RobotWrapper::framePosition(const Data & data, const Model::FrameIndex index) const
            {
                assert(index<m_model.frames.size());
                const Frame & f = m_model.frames[index];
                return data.oMi[f.parentJoint].act(f.placement);
            }

            void RobotWrapper::framePosition(const Data & data, const Model::FrameIndex index, SE3 & framePosition) const
            {
                assert(index<m_model.frames.size());
                const Frame & f = m_model.frames[index];
                framePosition = data.oMi[f.parentJoint].act(f.placement);
            }

            Motion RobotWrapper::frameVelocity(const Data & data, const Model::FrameIndex index) const
            {
                assert(index<m_model.frames.size());
                const Frame & f = m_model.frames[index];
                return f.placement.actInv(data.v[f.parentJoint]);
            }
        
            void RobotWrapper::frameVelocity(const Data & data, const Model::FrameIndex index, Motion & frameVelocity) const
            {
                assert(index<m_model.frames.size());
                const Frame & f = m_model.frames[index];
                frameVelocity = f.placement.actInv(data.v[f.parentJoint]);
            }
        
            Motion RobotWrapper::frameAcceleration(const Data & data, const Model::FrameIndex index) const
            {
                assert(index<m_model.frames.size());
                const Frame & f = m_model.frames[index];
                return f.placement.actInv(data.a[f.parentJoint]);
            }

            void RobotWrapper::frameAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const
            {
                assert(index<m_model.frames.size());
                const Frame & f = m_model.frames[index];
                frameAcceleration = f.placement.actInv(data.a[f.parentJoint]);
            }

            Motion RobotWrapper::frameClassicAcceleration(const Data & data, const Model::FrameIndex index) const
            {
                assert(index<m_model.frames.size());
                const Frame & f = m_model.frames[index];
                Motion a = f.placement.actInv(data.a[f.parentJoint]);
                Motion v = f.placement.actInv(data.v[f.parentJoint]);
                a.linear() += v.angular().cross(v.linear());
                return a;
            }

            void RobotWrapper::frameClassicAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const
            {
                assert(index<m_model.frames.size());
                const Frame & f = m_model.frames[index];
                frameAcceleration = f.placement.actInv(data.a[f.parentJoint]);
                Motion v = f.placement.actInv(data.v[f.parentJoint]);
                frameAcceleration.linear() += v.angular().cross(v.linear());
            }

            void RobotWrapper::frameJacobianLocal(Data & data, const Model::FrameIndex index, Data::Matrix6x & J)
            {
                assert(index<m_model.frames.size());

                return pinocchio::getFrameJacobian(m_model, data, index, pinocchio::LOCAL, J) ;
            }

            void RobotWrapper::frameJacobianWorldAligned(Data & data, const Model::FrameIndex index, Data::Matrix6x & J)
            {
                return pinocchio::getFrameJacobian(m_model, data, index, pinocchio::LOCAL_WORLD_ALIGNED, J);
            }
        }
    }
}
