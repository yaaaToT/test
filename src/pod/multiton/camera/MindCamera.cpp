#include "../../../../include/pod/multiton/camera/MindCamera.hpp"

#include "../../../../resource/core/MindCamera/CameraApi.h"
#include "../../../../resource/core/MindCamera/CameraStatus.h"
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/usbdevice_fs.h>

MindCamera::MindCamera(const cv::FileStorage& fileReader, const int& id):
    Camera(fileReader,id) {
    // 曝光时间
    fileReader["ExposureTime"] >> mExposureTime;
    // 曝光增益
    fileReader["ExposureValue"] >> mExposureValue;
    // RGB gain
    std::vector<int> cameraGain;
    fileReader["Gain"] >> cameraGain;
    mGainRed = cameraGain.at(0);
    mGainGreen = cameraGain.at(1);
    mGainBlue = cameraGain.at(2);
    // 相机伽马
    fileReader["Gamma"] >> mCameraGamma;

    fileReader["Sharpness"] >> mSharpness;

    fileReader["FrameSpeed"] >> mFrameSpeedInt;
    memset(&mResolution, 0, sizeof(tSdkImageResolution));
    std::vector<int> imageSize;
    fileReader["ImageSize"] >> imageSize;
    mResolution.iIndex = 0xff;
    mResolution.iWidth = imageSize.at(0);//
    mResolution.iHeight = imageSize.at(1);
    mResolution.iWidthFOV = imageSize.at(0);
    mResolution.iHeightFOV = imageSize.at(1);
    mResolution.iHOffsetFOV = (1280-imageSize.at(0))/2;
    mResolution.iVOffsetFOV = (1024-imageSize.at(1))/2;

}





void MindCamera::initParamsBySDK() {
    CameraSdkInit(1);
    // 获取相机句柄
    CameraEnumerateDevice(mCameraEnumList, &mCameraCountsInt);
    // 设置相机句柄   mCameraHandle之后用来设置相机参数
    CameraInit(&mCameraEnumList[this->mId], -1, -1, &mCameraHandle);
    std::cout<<this->mId<<" "<<&mCameraHandle<<" "<<mCameraHandle<<std::endl;


    CameraGetCapability(mCameraHandle, &mCapability);

    mGpRgbBuffer = (unsigned char *)malloc(mCapability.sResolutionRange.iHeightMax * mCapability.sResolutionRange.iWidthMax * 3);
    if(CameraPlay(mCameraHandle)==CAMERA_STATUS_SUCCESS) std::cout<<"SDK mode start"<<std::endl;

    // 设置参数
    if (setCameraParam() != CAMERA_STATUS_SUCCESS) {
        std::cout<<"fail to init mind camera params by SDK"<<std::endl;
        return;
    }
    if (mCapability.sIspCapacity.bMonoSensor) {
        mChannel = 1;
        CameraSetIspOutFormat(mCameraHandle, CAMERA_MEDIA_TYPE_MONO8);
    }
    else {
        mChannel = 3;
        CameraSetIspOutFormat(mCameraHandle, CAMERA_MEDIA_TYPE_BGR8);
    }

}

void MindCamera::startCamera() {

    // 相机初始化，才能改变参数 1表示选择中文界面
    CameraSdkInit(1);
    initParamsBySDK();
}

void MindCamera::releaseCamera() {
    CameraUnInit(mCameraHandle);
    free(mGpRgbBuffer);
    std::cout<<"release camera: "<<this->mId<<std::endl;
    std::cout<<"USB Reset"<<std::endl;
}

void MindCamera::restartCamera() {
    releaseCamera();
    /*
        Bus 004 Device 002: ID f622:0001 MindVision SUA133GC
        Bus 003 Device 003: ID f622:0001 MindVision SUA133GC
        */
    const char *filename;
    if(this->mId) filename = "/dev/bus/usb/004/002";//usb的ID,1号头
    else filename = "/dev/bus/usb/003/003";//usb的ID,1号头

    int fd = open(filename,O_WRONLY);
    int rc = ioctl(fd,USBDEVFS_RESET,0);
    if(rc >= 0) std::cout<<"reset successful"<<std::endl;
    close(fd);

    CameraSdkInit(1);
    initParamsBySDK();
    if(CameraConnectTest(mCameraHandle) != CAMERA_STATUS_SUCCESS) CameraReConnect(mCameraHandle);
}

void MindCamera::updateSrc() {
    while(true) {
        if (CameraGetImageBuffer(mCameraHandle, &mFrameHead, &mPbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
            CameraSetIspOutFormat(mCameraHandle, CAMERA_MEDIA_TYPE_BGR8);
            CameraImageProcess(mCameraHandle, mPbyBuffer, mGpRgbBuffer, &mFrameHead);
            this->mSrc = cv::Mat(cv::Size(mFrameHead.iWidth, mFrameHead.iHeight), CV_8UC3, mGpRgbBuffer);
            CameraReleaseImageBuffer(mCameraHandle, mPbyBuffer);
            cv::waitKey(mTimeOff);
        }else restartCamera();
    }
}


bool MindCamera::setCameraParam() {
    std::vector<std::string> errorOptions;

    if (!setFrameSpeed()) errorOptions.push_back("frameSpeed");
    if (!setExposureTime()) errorOptions.push_back("exposureTime");
    if (!setGamma()) errorOptions.push_back("gamma");
    if (!setGain()) errorOptions.push_back("gain");
    if (!setSize()) errorOptions.push_back("size");
    if (!setSharpness()) errorOptions.push_back("sharpness");
    if(errorOptions.empty()) {return false;}
    std::cout<<"fail to set mind camera param options: ";
    for(const std::string& option : errorOptions) std::cout<<option<<" ";
    std::cout<<std::endl;
    return true;
}

bool MindCamera::setSharpness(){
    if (CameraSetSharpness(mCameraHandle, mSharpness) != CAMERA_STATUS_SUCCESS){
        std::cout << "fail to set sharpness" << std::endl;
        return false;
    }
    std::cout << "set sharpness successfully, sharpness : " << mSharpness << std::endl;
    return true;
}

bool MindCamera::setExposureTime() const {
    if (CameraSetAeState(mCameraHandle, FALSE) != CAMERA_STATUS_SUCCESS) {
        std::cout << "fail to set exposure mode" << std::endl;
        return false;
    }
    std::cout << "set exposure mode successfully" << std::endl;

    CameraSetAnalogGain(mCameraHandle,mExposureValue);
    if (CameraSetExposureTime(mCameraHandle, mExposureTime) != CAMERA_STATUS_SUCCESS) {
        std::cout << "fail to set exposure value" << std::endl;
        return false;
    }
    std::cout << "set exposure value successfully, time : " << mExposureTime << std::endl;
    std::cout << ", value : " << mExposureValue << std::endl;
    return true;
}

bool MindCamera::setSize() {
    if (CameraSetImageResolution(mCameraHandle, &mResolution) != CAMERA_STATUS_SUCCESS) {
        std::cout << "fail to set camera size" << std::endl;
        return false;
    }
    std::cout << "set camera size successfully, width : " << mResolution.iWidth << " height : " << mResolution.iHeight << std::endl;
    return true;
}

bool MindCamera::setGain() const {

    if (CameraSetGain(mCameraHandle, mGainRed, mGainGreen, mGainBlue) != CAMERA_STATUS_SUCCESS) {
        std::cout << "fail to set gain" << std::endl;
        return false;
    }
    std::cout << "set gain successfully, R : " << mGainRed << " G : " << mGainGreen << " B : " << mGainBlue << std::endl;
    return true;
}
bool MindCamera::setGamma() const {
    if (CameraSetGamma(mCameraHandle, mCameraGamma) != CAMERA_STATUS_SUCCESS) {
        std::cout << "fail to set gamma" << std::endl;
        return false;
    }
    std::cout << "set gamma successfully, gamma : " << mCameraGamma << std::endl;
    return true;
}

bool MindCamera::setFrameSpeed() const {
    if (CameraSetFrameSpeed(mCameraHandle, mFrameSpeedInt) != CAMERA_STATUS_SUCCESS) {
        std::cout << "fail to set frame speed" << std::endl;
        return false;
    }
    std::cout << "set frame speed successfully, frame speed : " << mFrameSpeedInt << std::endl;
    return true;
}

