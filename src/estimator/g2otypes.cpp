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
    const VertexVBias* vVBiasi = static_cast<const VertexVBias*>(_vertices[1]);
    const VertexPR* vPRj = static_cast<const VertexPR*>(_vertices[2]);
    const VertexVBias* vVBiasj = static_cast<const VertexVBias*>(_vertices[3]);

    // terms need to computer error in vertex i, except for bias error
    const VertexState & NSPRi = vPRi->estimate();
    //cout<<"ID computer error: "<<NSPRi.id<<endl;
    const Vector3d Pi = NSPRi.P;
    //cout<<"computer error Pi: "<<Pi.transpose()<<endl;
    const Eigen::Matrix3d Ri = NSPRi.Rc0bk;
    //cout<<"computer error Ri: "<<Ri<<endl;

    const VertexState& NSVBiasi = vVBiasi->estimate();
    const Vector3d Vi = NSVBiasi.V;
    //cout<<"computer error Vi: "<<Vi.transpose()<<endl;
    // Bias from the bias vertex

    const Vector3d Bai = NSVBiasi.Bas;
    //cout<<"computer error Bai: "<<Bai.transpose()<<endl;
    const Vector3d Bgi = NSVBiasi.Bgs;
    //cout<<"computer error Bgi: "<<Bgi.transpose()<<endl;

    // terms need to computer error in vertex j, except for bias error
    const VertexState& NSPRj = vPRj->estimate();
    const Vector3d Pj = NSPRj.P;
    //cout<<"computer error Pj: "<<Pj.transpose()<<endl;
    const Eigen::Matrix3d Rj = NSPRj.Rc0bk;
    //cout<<"computer error Rj: "<<Rj<<endl;

    const VertexState& NSVBiasj = vVBiasj->estimate();
    const Vector3d Vj = NSVBiasj.V;
    //cout<<"computer error Vj: "<<Vj.transpose()<<endl;
    const Vector3d Baj = NSVBiasj.Bas;
    //cout<<"computer error Baj: "<<Baj.transpose()<<endl;
    const Vector3d Bgj = NSVBiasj.Bgs;
    //cout<<"computer error Bgj: "<<Bgj.transpose()<<endl;
    // IMU Preintegration measurement
    Integration M = _measurement;
    Vector15d err;
    err = M.evaluate(Pi, Ri,Vi, Bai, Bgi, Pj, Rj, Vj, Baj, Bgj, GravityVec);
    //cout<<"error: "<<endl;
    _error = err;
}

void EdgeStatePRVB::linearizeOplus()
{
    const VertexPR* vPRi = static_cast<const VertexPR*>(_vertices[0]);
    const VertexVBias* vVBiasi = static_cast<const VertexVBias*>(_vertices[1]);
    const VertexPR* vPRj = static_cast<const VertexPR*>(_vertices[2]);
    const VertexVBias* vVBiasj = static_cast<const VertexVBias*>(_vertices[3]);

    // terms need to computer error in vertex i, except for bias error
    const VertexState & NSPRi = vPRi->estimate();
    const Vector3d Pi = NSPRi.P;
    const Eigen::Matrix3d Ri = NSPRi.Rc0bk;


    const VertexState& NSVBiasi = vVBiasi->estimate();
    const Vector3d Vi = NSVBiasi.V;
    // Bias from the bias vertex
    const Vector3d Bai = NSVBiasi.Bas;
    const Vector3d Bgi = NSVBiasi.Bgs;

    // terms need to computer error in vertex j, except for bias error
    const VertexState& NSPRj = vPRj->estimate();
    const Vector3d Pj = NSPRj.P;
    const Eigen::Matrix3d Rj = NSPRj.Rc0bk;

    const VertexState& NSVBiasj = vVBiasj->estimate();
    const Vector3d Vj = NSVBiasj.V;

    // IMU Preintegration measurement
    Integration pre_integration = _measurement;
    const double dTij = pre_integration.sum_dt;
    const double dT2 = dTij * dTij;
    Eigen::Matrix3d RiT = Ri.transpose();
    Eigen::Matrix3d RjT = Rj.transpose();
    Eigen::Vector3d rPhij = _error.segment<3>(3);
    Eigen::Matrix3d JrInv_rPhi = Sophus::SO3::JacobianRInv(rPhij);
    int O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG =12;

    Eigen::Matrix3d dp_dba = pre_integration.jacobian.block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = pre_integration.jacobian.block<3, 3>(O_P, O_BG);

    Eigen::Matrix3d dq_dbg = pre_integration.jacobian.block<3, 3>(O_R, O_BG);

    Eigen::Matrix3d dv_dba = pre_integration.jacobian.block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = pre_integration.jacobian.block<3, 3>(O_V, O_BG);

    Eigen::Matrix3d ExprPhiijTrans = Sophus::SO3::exp(rPhij).inverse().matrix();
    Eigen::Vector3d dBgi = Bgi - pre_integration.linearized_bg;
    Eigen::Matrix3d JrBiasGCorr = Sophus::SO3::JacobianR(dq_dbg*dBgi);

    // calculate the jacobian for Keyframe i
    Matrix<double, 15, 6> jacobian_pose_i;
    jacobian_pose_i.setZero();
    jacobian_pose_i.block<3, 3>(O_P, O_P) = -RiT;
    jacobian_pose_i.block<3, 3>(O_P, O_R) = Sophus::SO3::hat(RiT * (Pj - Pi - Vi * dTij + 0.5 * GravityVec * dT2));
            //Utility::skewSymmetric(Qi.inverse() * (0.5 * GravityVec * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

    //Eigen::Quaterniond corrected_delta_q_i = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
    jacobian_pose_i.block<3, 3>(O_R, O_R) = - JrInv_rPhi * RjT * Ri;
            //-(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q_i)).bottomRightCorner<3, 3>();
    jacobian_pose_i.block<3, 3>(O_V, O_R) = Sophus::SO3::hat(RiT * ( Vj - Vi + GravityVec * dTij));
            //Utility::skewSymmetric(Qi.inverse() * (GravityVec * sum_dt + Vj - Vi));
    //jacobian_pose_i.setZero();

    Matrix<double, 15, 9> jacobian_velocity_bias_i;
    jacobian_velocity_bias_i.setZero();
    jacobian_velocity_bias_i.block<3, 3>(O_P, 0) = -RiT * dTij;
    jacobian_velocity_bias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
    jacobian_velocity_bias_i.block<3, 3>(O_P, O_BG - O_V) = -dv_dba;

    jacobian_velocity_bias_i.block<3, 3>(O_R, O_BG - O_V) = -JrInv_rPhi * ExprPhiijTrans * JrBiasGCorr * dq_dbg;

    jacobian_velocity_bias_i.block<3, 3>(O_V, 0) = -RiT;
    jacobian_velocity_bias_i.block<3, 3>(O_V, 3) = -dv_dba;
    jacobian_velocity_bias_i.block<3, 3>(O_V, 6) = -dv_dbg;

    jacobian_velocity_bias_i.block<3, 3>(O_BA, 3) = -Eigen::Matrix3d::Identity();

    jacobian_velocity_bias_i.block<3, 3>(O_BG, 6) = -Eigen::Matrix3d::Identity();

// calculate the jacobian for Keyframe j
    Matrix<double, 15, 6> jacobian_pose_j;
    jacobian_pose_j.setZero();
    jacobian_pose_j.block<3, 3>(O_P, O_P) = RiT;

    //Eigen::Quaterniond corrected_delta_q_j = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
    jacobian_pose_j.block<3, 3>(O_R, O_R) = JrInv_rPhi;
            //Utility::Qleft(corrected_delta_q_j.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
    //jacobian_pose_j.setZero();

    Matrix<double, 15, 9> jacobian_velocity_bias_j;
    jacobian_velocity_bias_j.setZero();
    jacobian_velocity_bias_j.block<3, 3>(O_V, 0) = RiT;

    jacobian_velocity_bias_j.block<3, 3>(O_BA, 3) = Eigen::Matrix3d::Identity();

    jacobian_velocity_bias_j.block<3, 3>(O_BG, 6) = Eigen::Matrix3d::Identity();

    //jacobian_bias_j.setZero();

    // Evaluate _jacobianOplus
    //cout<<"jacobian: "<<endl;
    _jacobianOplus[0] = jacobian_pose_i;
    //cout<<"error pose i"<<jacobian_pose_i<<endl;
    _jacobianOplus[1] = jacobian_velocity_bias_i;
   // cout<<"error pose j"<<jacobian_pose_j<<endl;
    _jacobianOplus[2] = jacobian_pose_j;
    //cout<<"error velocity i"<<jacobian_velocity_i<<endl;
    _jacobianOplus[3] = jacobian_velocity_bias_j;
}

}