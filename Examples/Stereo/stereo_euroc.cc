/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "../../src/imu_data/IMUData.h"
using namespace std;

typedef struct imageData_
{
    string imgName;
    double time;
} ImageData;


//void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
//                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);


void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathImage,
        vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<ImageData> &vstrImage);
void LoadIMUs(const string &strPathIMU, vector<IMUData> &vstrIMU);

vector<pair<vector<IMUData>, int>> getMeasurement(vector<IMU> &vstrIMU, vector<ImageData> &vstrImage);

int main(int argc, char **argv)
{
    if(argc != 7)
    {
        cerr << endl << "Usage: ./stereo_euroc path_to_vocabulary path_to_settings path_to_left_folder path_to_right_folder path_to_times_file" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<ImageData> vstrImage;

    vector<IMUData> vstrIMU;

    LoadImages(string(argv[3]), string(argv[4]), string(argv[5]), vstrImageLeft, vstrImageRight, vstrImage);
    LoadIMUs(string(argv[6]), vstrIMU);
    if(vstrImageLeft.empty() || vstrImageRight.empty())
    {
        cerr << "ERROR: No images in provided path." << endl;
        return 1;
    }

    if(vstrImageLeft.size()!=vstrImageRight.size())
    {
        cerr << "ERROR: Different number of left and right images." << endl;
        return 1;
    }

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
       rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);


    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
    vector<pair<vector<IMU>, int>> measurements = getMeasurement(vstrIMU, vstrImage);
    for(auto measurement : measurements)
    {
        int ni = measurement.second;

        vector<IMU> vimu_data = measurement.first;
        // Read left and right images from file

        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if(imRight.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageRight[ni]) << endl;
            return 1;
        }

        cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);



#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        //SLAM.TrackStereo(imLeftRect,imRightRect,tframe);
        double tframe = vstrImage[ni].time;
        SLAM.TrackStereoIMU(imLeftRect, imRightRect, tframe, vimu_data);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

       // cout<<ni<<endl;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    return 0;
}
void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathImage, vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<ImageData> &vstrImage)
{
    ifstream fTimes;
    fTimes.open(strPathImage.c_str());
    bool first_line = true;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(first_line)
        {
            first_line = false;
            continue;
        }
        int count = 0;
        if(!s.empty())
        {
            string tmp;
            stringstream ss;
            double t;
            ImageData tmp_img;
            for(int i = 0 ; i< s.size(); i++)
            {
                if(s[i] != ',')
                {
                    tmp.push_back(s[i]);
                }
                else if(s[i] == ',')
                {
                    ss << tmp;
                    ss >> t;
                    t = t/ 1e9;
                    tmp_img.time = t;
                    ss.clear();
                    tmp.clear();
                }
            }
            tmp = tmp.substr(0, tmp.size()-1);
            tmp_img.imgName = tmp;
            vstrImageLeft.push_back(strPathLeft + "/" + tmp);
            vstrImageRight.push_back(strPathRight + "/" + tmp);
            vstrImage.push_back(tmp_img);
        }
    }
    fTimes.close();
}


void LoadIMUs(const string &strPathIMU, vector<IMUData> &vstrIMU)
{
    ifstream fTimes;
    fTimes.open(strPathIMU.c_str());
    bool first_line = true;
    while(!fTimes.eof())
    {

        string s;
        getline(fTimes,s);
        if(first_line)
        {
            first_line = false;
            continue;
        }

        vector<double> angular;
        vector<double> acceleration;
        int count = 0;
        if(!s.empty())
        {

            string tmp;
            stringstream ss;
            double t;
            IMUData tmp_imu;
            for(int i = 0; i < s.size(); i++)
            {
                if(s[i] != ',')
                {
                    tmp.push_back(s[i]);
                }
                else
                {
                    ++count;
                    if(count == 1)
                    {
                        ss << tmp;
                        ss >> t;
                        t = t/ 1e9;
                        tmp_imu.time = t;
                    }
                    else if(count >=2 && count <= 4)
                    {
                        ss << tmp;
                        ss >> t;
                        angular.push_back(t);
                    }
                    else
                    {
                        ss << tmp;
                        ss >> t;
                        acceleration.push_back(t);
                    }
                    ss.clear();
                    tmp.clear();
                }
            }
            ss << tmp;
            ss >> t;
            acceleration.push_back(t);

            tmp_imu.wx = angular[0];
            tmp_imu.wy = angular[1];
            tmp_imu.wz = angular[2];

            tmp_imu.ax = acceleration[0];
            tmp_imu.ay = acceleration[1];
            tmp_imu.az = acceleration[2];

            vstrIMU.push_back(tmp_imu);
        }
    }
    fTimes.close();

}



vector<pair<vector<IMUData>, int>> getMeasurement(vector<IMU> &vstrIMU, vector<ImageData> &vstrImage)
{

    //为了使allimuData中的imu的第一帧和图像的第一帧对齐，去除了vstrImageLeft中图像的第一帧时刻之前的imu数据
    double firstImageTime = vstrImage[0].time;
    for (int i = 0; i < vstrIMU.size() - 1; i++)
    {
        if (firstImageTime - vstrIMU[i].time < 1 / 1e4)
        {

            vstrIMU.erase(vstrIMU.begin(), vstrIMU.begin() + i);
            break;
        }
    }

    vector<pair<vector<IMUData>, int>> measurements;

    int n_imu = vstrIMU.size(), index_imu = 0;
    int n_image = vstrImage.size();


    measurements.emplace_back(vector<IMUData>(), 0);
    for(int index_image = 1; index_image < n_image; index_image++)
    {
        vector<IMUData> imu_cur;
        while(vstrIMU[index_imu].time < vstrImage[index_image].time)
        {
            imu_cur.emplace_back(vstrIMU[index_imu]);
            ++index_imu;
        }
        measurements.emplace_back(imu_cur, index_image);
    }
    return measurements;
}










//void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
//                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<ImageData> &vstrImage)
//{
//    ifstream fTimes;
//    fTimes.open(strPathTimes.c_str());
//    vstrImageLeft.reserve(5000);
//    vstrImageRight.reserve(5000);
//    while(!fTimes.eof())
//    {
//        string s;
//        getline(fTimes,s);
//        if(!s.empty())
//        {
//            stringstream ss;
//            ss << s;
//            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
//            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
//            double t;
//            ss >> t;
//            ImageData tmpImg;
//            tmpImg.time = t;
//            vstrImage.push_back(tmpImg);
//        }
//
//    }
//    fTimes.close();
//}



