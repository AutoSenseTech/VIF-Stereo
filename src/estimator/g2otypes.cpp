//
// Created by wangweihan on 2019/8/22.
//

#include "g2otypes.h"
namespace g2o
{
using namespace ORB_SLAM2;

void EdgeStatePRVB::computeError()
{
    const VertexPR* vPRi = static_cast<const VertexPR*>(_vertices[0]);
    const VertexPR* vPRj = static_cast<const VertexPR*>(_vertices[1]);
    const VertexV* vVi = static_cast<const VertexV*>(_vertices[2]);
    const VertexV* vVj = static_cast<const VertexV*>(_vertices[3]);
    const VertexBias* vBiasi = static_cast<const VertexBias*>(_vertices[4]);
    const VertexBias* vBiasj = static_cast<const VertexBias*>(_vertices[5]);

    // terms need to computer error in vertex i, except for bias error
    const VertexState & NSPRi = vPRi->estimate();
    const Vector3d Pi = NSPRi.P;
    const Eigen::Quaterniond Qi = Eigen::Quaterniond(NSPRi.Rc0bk);

    const VertexState& NSVi = vVi->estimate();
    const Vector3d Vi = NSVi.V;
    // Bias from the bias vertex
    const VertexState& NSBiasi = vBiasi->estimate();
    const Vector3d Bai = NSBiasi.Bas;
    const Vector3d Bgi = NSBiasi.Bgs;


    // terms need to computer error in vertex j, except for bias error
    const VertexState& NSPRj = vPRj->estimate();
    const Vector3d Pj = NSPRj.P;
    const Eigen::Quaterniond Qj = Eigen::Quaterniond(NSPRj.Rc0bk);

    const VertexState& NSVj = vVj->estimate();
    const Vector3d Vj = NSVj.V;

    const VertexState& NSBiasj = vBiasj->estimate();
    const Vector3d Baj = NSBiasi.Bas;
    const Vector3d Bgj = NSBiasi.Bgs;

    // IMU Preintegration measurement
    Integration* M = _measurement;
    Vector15d err;
    err = M->evaluate(Pi, Qi,Vi, Bai, Bgi, Pj, Qj, Vj, Baj, Bgj, GravityVec);
    _error = err;
}

void EdgeStatePRVB::linearizeOplus()
{
    const VertexPR* vPRi = static_cast<const VertexPR*>(_vertices[0]);
    const VertexPR* vPRj = static_cast<const VertexPR*>(_vertices[1]);
    const VertexV* vVi = static_cast<const VertexV*>(_vertices[2]);
    const VertexV* vVj = static_cast<const VertexV*>(_vertices[3]);
    const VertexBias* vBiasi = static_cast<const VertexBias*>(_vertices[4]);
    const VertexBias* vBiasj = static_cast<const VertexBias*>(_vertices[5]);

    // terms need to computer error in vertex i, except for bias error
    const VertexState & NSPRi = vPRi->estimate();
    const Vector3d Pi = NSPRi.P;
    const Eigen::Quaterniond Qi = Eigen::Quaterniond(NSPRi.Rc0bk);

    const VertexState& NSVi = vVi->estimate();
    const Vector3d Vi = NSVi.V;
    // Bias from the bias vertex
    const VertexState& NSBiasi = vBiasi->estimate();
    const Vector3d Bai = NSBiasi.Bas;
    const Vector3d Bgi = NSBiasi.Bgs;

    // terms need to computer error in vertex j, except for bias error
    const VertexState& NSPRj = vPRj->estimate();
    const Vector3d Pj = NSPRj.P;
    const Eigen::Quaterniond Qj = Eigen::Quaterniond(NSPRj.Rc0bk);

    const VertexState& NSVj = vVj->estimate();
    const Vector3d Vj = NSVj.V;

    // IMU Preintegration measurement
    Integration* pre_integration = _measurement;
    int O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG =12;

    double sum_dt = pre_integration->sum_dt;
    Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(O_P, O_BG);

    Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(O_R, O_BG);

    Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(O_V, O_BG);

    // calculate the jacobian for Keyframe i
    Matrix<double, 15, 6> jacobian_pose_i;
    jacobian_pose_i.setZero();
    jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
    jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * GravityVec * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

    Eigen::Quaterniond corrected_delta_q_i = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
    jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q_i)).bottomRightCorner<3, 3>();
    jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (GravityVec * sum_dt + Vj - Vi));

    Matrix<double, 15, 3> jacobian_velocity_i;
    jacobian_velocity_i.setZero();
    jacobian_velocity_i.block<3, 3>(O_P, 0) = -NSPRi.Rc0bk.transpose() * sum_dt;
    jacobian_velocity_i.block<3, 3>(O_V, 0) = -NSPRi.Rc0bk.transpose();

    Matrix<double, 15, 3> jacobian_bas_i;
    jacobian_bas_i.setZero();
    jacobian_bas_i.block<3, 3>(O_P, 0) = -dp_dba;
    jacobian_bas_i.block<3, 3>(O_V, 0) = -dv_dba;
    jacobian_bas_i.block<3, 3>(O_BA, 0) = -Eigen::Matrix3d::Identity();

    Matrix<double, 15, 3> jacobian_bgs_i;
    jacobian_bgs_i.setZero();
    jacobian_bgs_i.block<3, 3>(O_P, 0) = -dp_dbg;
    jacobian_bgs_i.block<3, 3>(O_R, 0) = -Utility::Qleft(Qj.inverse() * Qi * pre_integration->delta_q).bottomRightCorner<3, 3>() * dq_dbg;
    jacobian_bgs_i.block<3, 3>(O_V, 0) = -dv_dbg;
    jacobian_bgs_i.block<3, 3>(O_BG, 0) = -Eigen::Matrix3d::Identity();

// calculate the jacobian for Keyframe j
    Matrix<double, 15, 6> jacobian_pose_j;
    jacobian_pose_j.setZero();
    jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

    Eigen::Quaterniond corrected_delta_q_j = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
    jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(corrected_delta_q_j.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();

    Matrix<double, 15, 3> jacobian_velocity_j;
    jacobian_velocity_j.setZero();
    jacobian_velocity_j.block<3, 3>(O_V, 0) = NSPRi.Rc0bk.transpose();


    Matrix<double, 15, 3> jacobian_bas_j;
    jacobian_bas_j.setZero();
    jacobian_bas_j.block<3, 3>(O_BA, 0) = Eigen::Matrix3d::Identity();

    Matrix<double, 15, 3> jacobian_bgs_j;
    jacobian_bgs_j.setZero();
    jacobian_bgs_i.block<3, 3>(O_BG, 0) = Eigen::Matrix3d::Identity();


    // Evaluate _jacobianOplus
    _jacobianOplus[0] = jacobian_pose_i;
    _jacobianOplus[1] = jacobian_pose_i;
    _jacobianOplus[2] = jacobian_velocity_i;
    _jacobianOplus[3] = jacobian_velocity_j;
    _jacobianOplus[4] = jacobian_bas_i;
    _jacobianOplus[5] = jacobian_bas_j;
}

}