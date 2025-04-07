#ifndef VIDEOCAMERA_HPP
#define VIDEOCAMERA_HPP

#include "Camera.hpp"

class VideoCamera:public Camera {
public:
    VideoCamera(const cv::FileStorage& fileReader, const int& id);
    void updateSrc() override;
    void startCamera() override;

private:
    cv::VideoCapture mVideoCapture;
    std::string mFileName;
};






#endif // VIDEOCAMERA_HPP
