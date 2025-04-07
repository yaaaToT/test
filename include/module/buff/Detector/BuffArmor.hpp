#ifndef BUFFARMOR_HPP
#define BUFFARMOR_HPP

#include <iostream>
#include <vector>
#include <utility>
#include <atomic>
#include <thread>
#include <functional>

#include "Detector.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/core/types.hpp"

#include "eigen3/Eigen/Eigen"


class BuffArmor
{
public:
    BuffArmor()=default; // 默认构造函数
    static std::vector<cv::Point2f>getNewBuffPoints(const BuffArmor&armor);
    static std::vector<cv::Point3d>getWorldPoints();
    static void updateDistanceAndPose(BuffArmor& armor, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);  // 更新距离和姿态
    static double getDistanceToCenter(const BuffArmor&armor,const cv::Mat& cameraMatrix);
    std::vector<cv::Point2f>CenterPoints;
    cv::Point2f buff_center;
    Detector detect;

    Eigen::Affine3d Pose;
    Eigen::Affine3d PoseTransform;

    float Distance;
    float Pitch;
    float Yaw;

};


#endif //BUFFARMOR_HPP
