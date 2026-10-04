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

namespace cho_controller {
namespace common {
namespace trajectory {
    
class TrajectorySample
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW                
        math::Vector pos, vel, acc;

        TrajectorySample(unsigned int size=0)
        {
            resize(size, size);
        }

        TrajectorySample(unsigned int size_pos, unsigned int size_vel)
        {
            resize(size_pos, size_vel);
        }

        void resize(unsigned int size)
        {
            resize(size, size);
        }

        void resize(unsigned int size_pos, unsigned int size_vel)
        {
            pos.setZero(size_pos);
            vel.setZero(size_vel);
            acc.setZero(size_vel);
        }
};


class TrajectoryBase
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TrajectoryBase(const std::string & name):
            m_name(name){}

        virtual unsigned int size() const = 0;

        virtual const TrajectorySample & operator()(double time) = 0;

        virtual const TrajectorySample & computeNext() = 0;

        virtual const TrajectorySample & getLastSample() const { return m_sample; }

        virtual void getLastSample(TrajectorySample & sample) const = 0;

        virtual bool has_trajectory_ended() const = 0;

        virtual const std::vector<Eigen::VectorXd> & getWholeTrajectory() = 0;

    protected:
        std::string m_name;
        TrajectorySample m_sample;
};

} // namespace trajectory
} // namespace common
} // namespace cho_controller
