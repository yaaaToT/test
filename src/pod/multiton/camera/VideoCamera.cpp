#include "../../../../include/pod/multiton/camera/VideoCamera.hpp"



VideoCamera::VideoCamera(const cv::FileStorage& fileReader, const int& id) :
    Camera(fileReader, id){
    fileReader["FileName"] >> mFileName;
    fileReader["TimeOff"] >> mTimeOff;
}

void VideoCamera::updateSrc() {
    while(true) {
        this->mVideoCapture.read(this->mSrc);
        cv::waitKey(mTimeOff);
    }
}


void VideoCamera::startCamera() {
    this->mVideoCapture = cv::VideoCapture("../resource/video/" + mFileName);
    //this->mVideoCapture = cv::VideoCapture("../../resource/video/" + mFileName);

    if(this->mVideoCapture.isOpened()) std::cout<<"successfully opened the file : "<<mFileName<<std::endl;
    else std::cout<<"fail to open file : "<<mFileName<<std::endl;
}
