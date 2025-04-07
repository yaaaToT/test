#include "../../../../include/module/buff/Detector/BuffArmor.hpp"
#include "../../../../include/module/buff/Detector/Detector.hpp"


#include <array>
#include <iostream>
#include <cmath>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

std::vector<cv::Point2f>BuffArmor::getNewBuffPoints(const BuffArmor &armor)
{
    std::vector<cv::Point2f>result;
    result.push_back(armor.buff_center/*detect.GetBuffCenter()*/);
//    result.push_back(detect.GetRectPoints()[1]);
//    result.push_back(detect.GetRectPoints()[2]);
//    result.push_back(detect.GetRectPoints()[3]);
    return result;
}

std::vector<cv::Point3d>BuffArmor::getWorldPoints()
{
    std::vector<cv::Point3d>result;
    //圆的外接矩形
    double R,D;
    R=150,D=300;
    result.push_back(cv::Point3d(-R/1.0,R/1.0,0));  //-x+y左下
    result.push_back(cv::Point3d(-R/1.0,-R/1.0,0));  //-x-y左上
    result.push_back(cv::Point3d(R/1.0,-R/1.0,0));  //+x-y右上
    result.push_back(cv::Point3d(R/1.0,R/1.0,0));  //+x+y右下
    return result;
}

void BuffArmor::updateDistanceAndPose(BuffArmor &armor, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs)
{
    // 定义旋转矩阵和转换矩阵
    cv::Mat rotateMat, transMat;
    // 使用 solvePnP 函数求解 PnP 问题
    // getWorldPoints(armor.mType) 获取世界坐标系中的点
    // armor.mArmorPoints 是图像中的点
    // cameraMatrix 和 distCoefs 分别是相机内参矩阵和畸变系数
    // solvePnP 方法使用 AP3P 算法求解旋转和平移矩阵

    cv::solvePnP(getWorldPoints(),armor.CenterPoints/*detect.GetRectPoints()*/,cameraMatrix,distCoeffs,rotateMat,transMat,false,cv::SOLVEPNP_AP3P);

    // 提取平移矩阵中的 x, y, z 值
    double x = transMat.ptr<double>(0)[0];
    double y = transMat.ptr<double>(0)[1];
    double z = transMat.ptr<double>(0)[2];

    armor.Pose.translation().x()=transMat.ptr<double>(0)[0]; // x
    armor.Pose.translation().y()=transMat.ptr<double>(0)[1]; // y
    armor.Pose.translation().z()=transMat.ptr<double>(0)[2]; // z

    // 定义一个旋转矩阵
    cv::Mat rotateMatrix;
    // 使用 Rodrigues 函数将旋转向量转换为旋转矩阵
    Rodrigues(rotateMat, rotateMatrix);
    // 创建一个 Eigen 矩阵来存储转换后的旋转矩阵
    Eigen::Matrix3d tf2_rotation_Matrix;
    tf2_rotation_Matrix <<
        rotateMatrix.at<double>(0, 0), rotateMatrix.at<double>(0, 1), rotateMatrix.at<double>(0, 2),
        rotateMatrix.at<double>(1, 0), rotateMatrix.at<double>(1, 1), rotateMatrix.at<double>(1, 2),
        rotateMatrix.at<double>(2, 0), rotateMatrix.at<double>(2, 1), rotateMatrix.at<double>(2, 2);
    // 将 Eigen 矩阵赋值给 armor 对象的 mPose 的线性部分
    armor.Pose.linear()=tf2_rotation_Matrix;
    // 计算装甲板到相机的距离
    armor.Distance=sqrtf(x*x+y*y+z*z);
}

double BuffArmor::getDistanceToCenter(const BuffArmor &armor, const cv::Mat &cameraMatrix)
{
    return norm(armor.buff_center/*detect.GetBuffCenter()*/-cv::Point2f(cameraMatrix.at<double>(0,2),cameraMatrix.at<double>(1,2)));
}
