#pragma once

#include "cho_controller_common/math/fwd.hpp"
#include "cho_controller_common/robot/fwd.hpp"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
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

    Eigen::VectorXd nonLinearEffects(const Data & data);

    Eigen::VectorXd GeneralizedGravity(const Data & data);

    const SE3 & position(const Data & data, const Model::JointIndex index) const;

    const Eigen::MatrixXd & mass(const Data & data);

    const Eigen::MatrixXd & mass_inverse(const Data & data);

    const Eigen::MatrixXd & coriolis(const Data & data);

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

    void frameJacobianLocal(Data & data, const Model::FrameIndex index, Data::Matrix6x & J)  ;


protected:
    Model m_model;
    std::string m_model_filename;
    bool m_verbose;
    int m_na;
    Eigen::MatrixXd m_M, m_Minv, m_C;
    Eigen::MatrixXd m_S, m_S_dot;
    Matrix6d m_Rot;
    double r_, b_, d_, c_; 
    Eigen::VectorXd m_q;
};

} // namespace robot
} // namespace common
} // namespace cho_controller
