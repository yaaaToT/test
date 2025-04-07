#include "../../include/utils/MessageUtils.hpp"


void MessageUtils::putText(cv::Mat& outSrc, const std::string& info, const cv::Point2f& point) {
    cv::putText(outSrc, info, point, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(165, 155, 255), 1);
}

void MessageUtils::line(cv::Mat& outSrc, const cv::Point2f& point1, const cv::Point2f& point2) {
    cv::line(outSrc, point1, point2, cv::Scalar(0, 255, 255), 2, 8);
}

void MessageUtils::circle(cv::Mat& outSrc, const cv::Point2f& point) {
    // cv::circle(outSrc, point, 8, cv::Scalar(0, 255, 255), 3);
    cv::circle(outSrc, point, 5, cv::Scalar(0, 255, 255), -1);
}

void MessageUtils::circle(cv::Mat& outSrc, const cv::Point2f& point, const int& color) {
    if(color == 1) cv::circle(outSrc, point, 8, cv::Scalar(0, 255, 0), 3);
    else if(color == 2) cv::circle(outSrc, point, 8, cv::Scalar(0, 0, 255), 3);
    else cv::circle(outSrc, point, 8, cv::Scalar(0, 255, 255), 3);
}

TIME MessageUtils::getTimePoint() {
    return std::chrono::steady_clock::now();
}

double MessageUtils::getTimeByPoint(const TIME& startTime, const TIME& endTime) {
    std::chrono::duration<double> timeUsed = endTime - startTime;
    return (double)(timeUsed.count());
}


double MessageUtils::getTimeByPoint(const TIME& startTime) {
    std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> timeUsed = endTime - startTime;
    return (double)(timeUsed.count());
}
double MessageUtils::getFpsByTime(const double& time) {
    return 1.0 / time;
}
double MessageUtils::getFpsByTimePoint(const TIME& startTime) {
    std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> timeUsed = endTime - startTime;
    double time = (double)(timeUsed.count());
    return 1.0 / time;
}


