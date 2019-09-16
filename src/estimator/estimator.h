//
// Created by wangweihan on 7/24/19.
//

#ifndef ORB_SLAM2_ESTIMATOR_H
#define ORB_SLAM2_ESTIMATOR_H

#include <eigen3/Eigen/Dense>
#include "integration.h"
#include <iostream>
using namespace std;
class Estimator
{
public:
    Estimator();
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }


    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;
    vector<Eigen::Vector3d> Ps;  //Pwi
    vector<Eigen::Vector3d> Vs;
    vector<Eigen::Matrix3d> Rs;  // Rwi
    //vector<double> intervalTime; // the interval time between bk and bk+1
    Eigen::Vector3d Bas;
    Eigen::Vector3d Bgs;
    Eigen::Vector3d g;

    //bk-bk+1 imu data
    Eigen::Quaterniond delta_q; //qbkbk
    Eigen::Vector3d delta_p; // pbkbk
    Eigen::Vector3d delta_v; // vbkbk
    bool first_imu;

};

#endif //ORB_SLAM2_ESTIMATOR_H
