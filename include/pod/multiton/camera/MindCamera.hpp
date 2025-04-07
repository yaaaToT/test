#ifndef MINDCAMERA_HPP
#define MINDCAMERA_HPP
#include "Camera.hpp"
#include "../../../../resource/core/MindCamera/CameraDefine.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/types_c.h>

class MindCamera : public Camera {
public:
    MindCamera(const cv::FileStorage& fileReader, const int& id);
    // 初始化
    void initParamsBySDK();
    // 启动相机
    void startCamera() override;
    void releaseCamera();
    // 重启摄像机
    void restartCamera();
    // 更新现实原始图像
    void updateSrc() override;
    //设置锐度
    bool setSharpness();
    bool setCameraParam();
    // 设置曝光时间
    bool setExposureTime() const;
    // 设置图像增益
    bool setGain() const;
    // 设置伽马
    bool setGamma() const;
    bool setSize();
    bool setFrameSpeed() const;

private:
    // 曝光时间
    double mExposureTime;
    // R gain
    int mGainRed;
    // G gain
    int mGainGreen;
    // B Gain
    int mGainBlue;
    // 相机伽马
    int mCameraGamma;
    // 相机的句柄
    int mCameraHandle;
    int mFrameSpeedInt;
    int mCameraCountsInt = 10;
    int mChannel = 3;
    // 锐度
    int mSharpness;
    // 曝光增益
    int mExposureValue;

    BYTE* mPbyBuffer;
    // 处理后数据缓存区
    unsigned char* mGpRgbBuffer;
    IplImage* mIplImage;

    tSdkImageResolution mResolution;
    tSdkCameraDevInfo mCameraEnumList[10];
    tSdkFrameHead mFrameHead;
    // 设备描述信息
    tSdkCameraCapbility mCapability;
};


#endif //MINDCAMERA_HPP
