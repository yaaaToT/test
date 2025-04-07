#include "../../../../include/pod/multiton/camera/Camera.hpp"
#include <iostream>

Camera::Camera(const cv::FileStorage& fileReader, const int& id) : mId(id) {
    std::string cameraName = std::to_string(id);

    //设置相机内参矩阵
    std::vector<double> cameraMatrix;
    fileReader["CameraMatrix"] >> cameraMatrix;
    setCameraMatrix(cameraMatrix);

    //设置相机畸变矩阵
    std::vector<double> distCoeffs;
    fileReader["DistCoeffs"] >>distCoeffs;
    setDistCoeffs(distCoeffs);

    //设置相机模式
    fileReader["CameraType"] >> mCameraType;

    fileReader["TimeOff"] >> mTimeOff;
}

void Camera::updateSrc() {
    std::cout<<"camera update src" <<std::endl;
}

void Camera::startCamera() {
    std::cout<<"camera start"<<std::endl;
}

// 获取原始图像的备份
void Camera::cloneSrc(cv::Mat& outSrc) const {
    //outSrc = this->mSrc.clone();
    try {
        if(!mSrc.empty()) {outSrc = this->mSrc.clone();}
        else {throw std::runtime_error("camera open");}
    }catch(const std::runtime_error& e) {
        std::cerr<<e.what()<<std::endl;
    }
}

cv::Point2f Camera::matrixToPoint2f(Eigen::Vector3d& matrix) {
    cv::Mat cameraMatrix = this->mCameraMatrix;
    Eigen::Matrix3d mat_intrinsic;
    cv2eigen(cameraMatrix, mat_intrinsic);
    auto result = (1.f / matrix[2]) * mat_intrinsic * (matrix); // 解算前进行单位转换
    return cv::Point2f(result[0], result[1]);
}


void Camera::setCameraMatrix(const std::vector<double>& matrix) {
    mCameraMatrix = cv::Mat(3, 3, CV_64F);
    for(int i =0;i<3;i++) {
        for(int j =0;j<3;j++) {
            mCameraMatrix.at<double>(i,j) = matrix.at(i*3+j);
        }
    }

}

void Camera::setDistCoeffs(const std::vector<double>& matrix) {
    mDistCoeffs = cv::Mat(5, 1, CV_64F);
    for(int i =0;i<5;i++) {
        mDistCoeffs.at<double>(i,0) = matrix.at(i);
    }
}

cv::Mat& Camera::getCameraMatrix() {
    return this->mCameraMatrix;
}
cv::Mat& Camera::getDistCoeffs() {
    return this->mDistCoeffs;
}
int Camera::getId() const{
    return this->mId;
}

CameraType Camera::getType() {
    return this->mCameraType;
}
