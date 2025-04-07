#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <iostream>
#include <vector>
#include <utility>
#include <atomic>
#include <thread>
#include <functional>
#include <chrono>

#include "../../../pod/multiton/package/DataPacket.hpp"
#include "../../../utils/MessageUtils.hpp"
//#include "../PredictSolver/tracker.hpp"
#include "../PredictSolver/DFtransform.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/core/types.hpp"

#include "eigen3/Eigen/Eigen"

//typedef std::chrono::system_clock::time_point Time_;

typedef std::chrono::high_resolution_clock::time_point Tp;

// rm_rune 打符机制
// 运动方向随机

// small rune 小符
// v=1/3*PI rad/s 固定转速

// big rune 大符
// V-> spd=a*sin(w*t)+b rad/s 速度目标函数
// t-s a[0.780,1.045] w[1.884,2.000] b=2.090-a 范围

// V(real) V(function) error[500ms]

// 旋转半径1400
// 72度

enum colorType{
    RED,
    BLUE,
};

enum Mode{
   SMALL=0,
   LARGE=1,
};


class Detector
{
public:
    Detector()=default;
    bool findCenterPoints(const cv::Mat&src,const cv::Mat&mask);
    bool isComplete();
    void calAngleSpeed();
    bool LargeBuffPosition();
    bool SmallBuffPosition();
    bool judgeRotation();
    bool SpeedState(double dt,double stdThreshold);
    int CircleFitting(double &dRadius);
    static cv::Mat getSpliteImg(const cv::Mat& src,const ColorType& colorType);
    static cv::Mat fillHole(const cv::Mat&src);
    static void anglesProcessing(std::vector<double>& angles);
    static float getDistance(const cv::Point2f &point_1,const cv::Point2f &point_2);
    static cv::Point2f getCenter(const std::vector<cv::Point2f>& points);

    double runPredictor(double t);
private:
    // 辅助方法
    struct ContourInfo{
        int max_area_index = -1;
        int max_child_index = -1;
        double max_contour_length = 0;
        double max_contour_area = 0;
        int max_child_count = 0;
        double lenth_Area = 0;
    };
    static ContourInfo analyzeContours(const cv::Mat& mask);
    bool validateContour(const ContourInfo& info) const;
    void processValidContour(const cv::Mat& src,const std::vector<std::vector<cv::Point>>& contours,
                             const ContourInfo& info);
    void updateTrackingState();
    void drawCrucialInfo(const cv::Mat& src) const;
    void predictPosition();
public:
    bool GetMode()const;
    cv::Point2f GetBuffCenter()const;
    cv::Point2f GetLargeBuffPointsCenter()const;
    cv::Point2f GetSmallBuffPointsCenter()const;
    std::vector<cv::Point2f> GetRectPoints()const;
    std::vector<cv::Point2f> GetLargeBuffPoints()const;
    std::vector<cv::Point2f> GetSmallBuffPoints()const;   
private:
    // 检测目标相关成员变量
    std::vector<cv::Point2f> rect_points;
    cv::Point2f buff_center;
    double theta;
    cv::Point2d fit_center;

    // 状态跟踪相关成员变量
    bool is_clockwise;
    Tp Start_time;
    Tp currentTime;
    Tp prevTime;
    double time_diff;
    //double curr_seconds;
    //double prev_seconds;

    bool ModeType = 0; // 0 大 1 小
    int activation_count = 0;
    cv::Point2f prev_point;
    cv::Point2f large_buff_points_center;
    cv::Point2f small_buff_points_center;
    std::vector<cv::Point2d> tracking_points;
    std::vector<double> angles;
    std::vector<cv::Point2f> large_buff_points;
    std::vector<cv::Point2f> small_buff_points;

};



#endif // DETECTOR_HPP
