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
