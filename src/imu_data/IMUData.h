//
// Created by wangweihan on 7/26/19.
//

#ifndef ORB_SLAM2_IMUDATA_H
#define ORB_SLAM2_IMUDATA_H

#include <eigen3/Eigen/Dense>

typedef struct IMU
{
public:
    double wx;
    double wy;
    double wz;
    double ax;
    double ay;
    double az;
    double time;
    bool isFirstIMU;
}IMUData;
#endif //ORB_SLAM2_IMUDATA_H
