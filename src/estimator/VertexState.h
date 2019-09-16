////
//// Created by wangweihan on 19-8-26.
////
//
#ifndef ORB_SLAM2_VERTEXSTATE_H
#define ORB_SLAM2_VERTEXSTATE_H
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include "so3.h"
#include "KeyFrame.h"
namespace ORB_SLAM2
{

typedef Eigen::Matrix<double, 15, 1> Vector15d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class VertexState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexState();
    VertexState(KeyFrame* KF);

    void IncSmallPR(Vector6d dPR);  //small rao dong
    void IncSmallV(Eigen::Vector3d dv); //small velocity
    void IncSmallBias(Vector6d dBias);


    Eigen::Matrix3d Rc0bk;

    Eigen::Vector3d P;         // position Pwbk  w=c0
    Eigen::Vector3d V;         // velocity
    Sophus::SO3 R;         // rotation

    // keep unchanged during optimization
    Eigen::Vector3d Bgs;   // bias of gyroscope
    Eigen::Vector3d Bas;   // bias of accelerometer

    // update below term during optimization
    Eigen::Vector3d dBias_a;  // delta bias of accelerometer
    Eigen::Vector3d dBias_g;  // delta bias of gyroscope, correction term computed in optimization

};
}
#endif //ORB_SLAM2_VERTEXSTATE_H
