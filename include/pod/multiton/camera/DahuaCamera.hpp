//#ifndef DAHUACAMERA_HPP
//#define DAHUACAMERA_HPP
//#include "Camera.hpp"
//#include "GenICam/Camera.h"
//#include <opencv2/opencv.hpp>

//enum CameraTypr {
//    LeftRaw,
//    RightRaw,
//    LeftDetect,
//    RightDetect
//};

//class DahuaCamera : public Camera{
//public:
//    DahuaCamera(const cv::FileStorage& fileReader, const int& id);
//    void updateSrc() override;
//    void startCamera() override;
//    bool findDevice();
//    void displayDeviceInfo();
//    void setDeviceConnectMode();
//    bool autoSetCameraIP();
//    bool setGrabMode(const bool& bContious);
//    bool setExposureTime(const bool& bAutoExposure);
//    void createStreamObj();
//    bool setROI(const int64_t& nX, const int64_t& nY, const int64_t& nWidth, const int64_t& nHeight);
//private:
//    static Dahua::Infra::TVector<Dahua::GenICam::ICameraPtr> mCameraPtrList;
//    Dahua::GenICam::ICameraPtr mCameraSptr;
//    Dahua::GenICam::IStreamSourcePtr mImgGetThread;
//    int mExposure = 3000;
//    int mWidth   = 960;
//    int mHeight  = 768;
//    double mRgbBlue    = 1;
//    double mRgbGreen = 1;
//    double mRgbRed    = 0.9;
//    int mTimeOff = 10;
//};


//#endif // DAHUACAMERA_HPP
