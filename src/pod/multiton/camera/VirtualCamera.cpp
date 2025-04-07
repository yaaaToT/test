#include "../../../../include/pod/multiton/camera/VirtualCamera.hpp"



VirtualCamera::VirtualCamera(const cv::FileStorage& fileReader, const int& id) :
    Camera(fileReader, id){
}

void VirtualCamera::updateSrc() {
    while(true) {
        this->mVideoCapture.read(this->mSrc);
        cv::waitKey(mTimeOff);
    }
}


void VirtualCamera::startCamera() {
    this->mVideoCapture = cv::VideoCapture(this->getId());
}
