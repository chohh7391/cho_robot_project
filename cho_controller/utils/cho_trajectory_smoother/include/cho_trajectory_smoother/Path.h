/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * Modified for cho_robot_project (namespace and packaging changes).
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <list>
#include <vector>
#include <Eigen/Core>

namespace cho_trajectory {

class PathSegment
{
public:
	PathSegment(double length = 0.0) :
		length(length)
	{
	}
	
	virtual ~PathSegment() {}

	double getLength() const {
		return length;
	}
	virtual Eigen::VectorXd getConfig(double s) const = 0;
	virtual Eigen::VectorXd getTangent(double s) const = 0;
	virtual Eigen::VectorXd getCurvature(double s) const = 0;
	virtual std::list<double> getSwitchingPoints() const = 0;
	virtual PathSegment* clone() const = 0;

	double position;
protected:
	double length;
};

class stdlist_Eigenvec{
	public:
		stdlist_Eigenvec(){};
		~stdlist_Eigenvec(){};
		
		void extend(Eigen::VectorXd val){
			Path_list.push_back(val);
		};
		std::list<Eigen::VectorXd> get_stdlist() const{
			return Path_list;
		};
		void clear(){
			Path_list.clear();
		}

	protected:
		std::list<Eigen::VectorXd> Path_list;
};

class Path
{
public:
	Path(const stdlist_Eigenvec &path, double maxDeviation = 0.0);
	Path(const Path &path);
	~Path();
	double getLength() const;
	Eigen::VectorXd getConfig(double s) const;
	Eigen::VectorXd getTangent(double s) const;
	Eigen::VectorXd getCurvature(double s) const;
	double getNextSwitchingPoint(double s, bool &discontinuity) const;
	std::list<std::pair<double, bool> > getSwitchingPoints() const;
private:
	PathSegment* getPathSegment(double &s) const;
	double length;
	std::list<std::pair<double, bool> > switchingPoints;
	std::list<PathSegment*> pathSegments;
};

} // namespace cho_trajectory
