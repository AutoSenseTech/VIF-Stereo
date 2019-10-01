//
// Created by wangweihan on 19-8-26.
//

#include "VertexState.h"

namespace ORB_SLAM2
{
    VertexState::VertexState()
    {
        P.setZero();
        V.setZero();
        Bgs.setZero();
        Bas.setZero();
    }

    VertexState::VertexState(KeyFrame* KF):
    P(KF->Ps), V(KF->Vs), Rc0bk(KF->Rc0bk),
    Bas(KF->Bas), Bgs(KF->Bgs),id(KF->mnId)
    {
        R = Sophus::SO3(KF->Rc0bk);
    }

    void VertexState::IncSmallPR(Vector6d dPR)
    {
        Eigen::Vector3d upd_P = dPR.segment<3>(0);
        Eigen::Vector3d upd_Phi = dPR.segment<3>(3);
        P += upd_P;
        R = R * Sophus::SO3::exp(upd_Phi);
    }

    void VertexState::IncSmallVBias(Vector9d dvbias)
    {
        Eigen::Vector3d upd_dv = dvbias.segment<3>(0);
        Eigen::Vector3d upd_dBa = dvbias.segment<3>(3);
        Eigen::Vector3d upd_dBg = dvbias.segment<3>(6);
        V += upd_dv;
        dBias_a += upd_dBa;
        dBias_g+= upd_dBg;
    }

    void VertexState::IncSmallBias(ORB_SLAM2::Vector6d dBias)
    {
        Eigen::Vector3d upd_dBa = dBias.segment<3>(0);
        Eigen::Vector3d upd_dBg = dBias.segment<3>(3);
        dBias_a += upd_dBa;
        dBias_g+= upd_dBg;
    }
}