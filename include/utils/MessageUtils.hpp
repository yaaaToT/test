#ifndef MESSAGEUTILS_HPP
#define MESSAGEUTILS_HPP
#include <iostream>
#include <opencv2/opencv.hpp>
typedef std::chrono::steady_clock::time_point TIME;
class MessageUtils {
public:
    // 删除构造函数和析构函数，禁止实例化
    MessageUtils() = delete;
    ~MessageUtils() = delete;

    static void circle(cv::Mat& outSrc, const cv::Point2f& point);
    static void circle(cv::Mat& outSrc, const cv::Point2f& point, const int& color);
    static void putText(cv::Mat& outSrc, const std::string& info, const cv::Point2f& point);
    static void line(cv::Mat& outSrc, const cv::Point2f& point1, const cv::Point2f& point2);
    static TIME getTimePoint();
    static double getTimeByPoint(const TIME& startTime, const TIME& endTime);
    static double getTimeByPoint(const TIME& startTime);
    static double getFpsByTime(const double& time);
    static double getFpsByTimePoint(const TIME& startTime);

};

#endif // MESSAGEUTILS_HPP
