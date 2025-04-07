#ifndef VIRTUALCAMERA_HPP
#define VIRTUALCAMERA_HPP
#include "Camera.hpp"

class VirtualCamera:public Camera {
public:
    VirtualCamera(const cv::FileStorage& fileReader, const int& id);
    void updateSrc() override;
    void startCamera() override;

private:
    cv::VideoCapture mVideoCapture;
};




#endif // VIRTUALCAMERA_HPP
