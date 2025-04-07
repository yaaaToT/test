#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include "Eigen/Dense"
#include <opencv2/core/eigen.hpp>


enum CameraType {
    DAHUA = 0,
    MIND = 1,
    DAHENG = 2,
    VIRTUAL = 3,
    VIDEO = 4
};

class Camera {
public:
    Camera(const cv::FileStorage& fileReader, const int& id);
    // 更新现实原始图像
    virtual void updateSrc();
    virtual void startCamera();
    // 获取原始图像的备份
    void cloneSrc(cv::Mat& outSrc) const;
    void setCameraMatrix(const std::vector<double>& matrix);
    void setDistCoeffs(const std::vector<double>& matrix);
    cv::Mat& getCameraMatrix();
    cv::Mat& getDistCoeffs();
    CameraType getType();
    virtual ~Camera() = default;
    cv::Point2f matrixToPoint2f(Eigen::Vector3d& matrix);
    int getId() const;
protected:
    cv::Mat mSrc;
    int mId;
    cv::Mat mCameraMatrix;
    cv::Mat mDistCoeffs;
    CameraType mCameraType;
    int mTimeOff;
};


#endif //CAMERA_HPP
