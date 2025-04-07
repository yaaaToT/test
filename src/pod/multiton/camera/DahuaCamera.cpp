//#include "../../../../include/pod/multiton/camera/DahuaCamera.hpp"
//#include <arpa/inet.h>
//#include <csignal>
//#include <opencv2/opencv.hpp>
//#include "GenICam/System.h"
//#include "GenICam/GigE/GigECamera.h"
//#include "GenICam/GigE/GigEInterface.h"
//#include <Media/ImageConvert.h>
//Dahua::Infra::TVector<Dahua::GenICam::ICameraPtr> DahuaCamera::mCameraPtrList;

//DahuaCamera::DahuaCamera(const cv::FileStorage& fileReader, const int& id) :
//    Camera(fileReader, id){
//    fileReader["Exposure"] >> mExposure;
//    fileReader["Width"] >> mWidth;
//    fileReader["Height"] >> mHeight;
//    fileReader["RgbBlue"] >> mRgbBlue;
//    fileReader["RgbGreen"] >> mRgbGreen;
//    fileReader["RgbRed"] >> mRgbRed;
//    fileReader["TimeOff"] >> mTimeOff;

//    /* 发现设备 */
//    while (true) {
//        if(!findDevice()) continue;
//        displayDeviceInfo();
//        mCameraSptr = mCameraPtrList[LeftRaw];
//        setDeviceConnectMode();

//        /* 连接相机 */
//        if (mCameraSptr->connect()) {
//            std::cout << "connect camera successfully" << std::endl;
//            break;
//        }

//        std::cout << "fail to connect camera" << std::endl;
//    }

//}


//bool DahuaCamera::findDevice() {
//    if (!Dahua::GenICam::CSystem::getInstance().discovery(mCameraPtrList)) {
//        std::cout << "device error" << std::endl;
//        return false;
//    }

//    if (mCameraPtrList.empty()) {
//        std::cout << "not found device" << std::endl;
//        return false;
//    }
//    return true;
//}

//void DahuaCamera::updateSrc() {
//    while (true) {
//        Dahua::GenICam::CFrame frame;

//        //获取一帧
//        if (!mImgGetThread) {
//            std::cout << "not found img getter thread" << std::endl;
//            continue;
//        }

//        if (!mImgGetThread->getFrame(frame, 150)) {
//            std::cout << "fail to get frame" << std::endl;
//            continue;
//        }

//        //判断帧的有效性
//        if (!frame.valid()) {
//            std::cout << "frame is invalid" << std::endl;
//            continue;
//        }

//        int nBGRBufferSize = frame.getImageWidth() * frame.getImageHeight() * 3;
//        unsigned char* pBGRbuffer = ( unsigned char* )malloc(sizeof(unsigned char) * nBGRBufferSize);

//        IMGCNV_SOpenParam convParam;
//        convParam.width       = frame.getImageWidth();
//        convParam.height      = frame.getImageHeight();
//        convParam.paddingX    = frame.getImagePadddingX();
//        convParam.paddingY    = frame.getImagePadddingY();
//        convParam.dataSize    = frame.getImageSize();
//        convParam.pixelForamt = frame.getImagePixelFormat();

//        // 转换为 BGR24。 转换为其他格式时,调用相应的接口
//        // 不等于则释放内存
//        if (IMGCNV_SUCCESS != IMGCNV_ConvertToBGR24(( unsigned char* )frame.getImage(), &convParam, pBGRbuffer, &nBGRBufferSize)) {
//            std::cout<<"fail to convert IMGCNV to BGRA24_Ex"<<std::endl;
//            free(pBGRbuffer);
//        }

//        mSrc = cv::Mat(cv::Size(frame.getImageWidth(), frame.getImageHeight()), CV_8UC3, pBGRbuffer);
//        if (!mSrc.empty()) {
//            std::vector<cv::Mat> imageRGB;

//            // RGB三通道分离
//            split(mSrc, imageRGB);

//            //调整RGB三个通道各自的值#include "GenICam/GigE/GigEInterface.h"
//            imageRGB.at(0) *= mRgbBlue;
//            imageRGB.at(1) *= mRgbGreen;
//            imageRGB.at(2) *= mRgbRed;

//            // RGB三通道图像合并
//            merge(imageRGB, mSrc);

//        }
//        cv::waitKey(mTimeOff);

//        /* 释放内存 */
//        free(pBGRbuffer);
//    }

//    /* 停止采图 */
//    mImgGetThread->stopGrabbing();



//    /* 断开相机 */
//    if (!mCameraSptr->disConnect()) {
//        std::cout<<"disConnect camera failed"<<std::endl;
//        return;
//    }

//}


//void DahuaCamera::setDeviceConnectMode() {
//    /* GigE相机时，连接前设置相机Ip与网卡处于同一网段上 */
//    if (Dahua::GenICam::ICamera::typeGige != mCameraSptr->getType()) {
//        std::cout << "connect mode : USB" << std::endl;
//        return;
//    }


//    if (!autoSetCameraIP()) {
//        std::cout << "fail to set camera IP" << std::endl;
//    } else {
//        std::cout << "set camera IP successfully" << std::endl;
//    }

//}

//void DahuaCamera::createStreamObj() {
//    mImgGetThread = Dahua::GenICam::CSystem::getInstance().createStreamSource(mCameraSptr);
//    if (NULL == mImgGetThread) {
//        std::cout << "create stream obj fail" << std::endl;
//        return;
//    }
//    std::cout << "create stream obj successfully" << std::endl;
//}

//void DahuaCamera::startCamera() {
//    /* 设置相机为连续拉流模式 */
//    setGrabMode(true);

//    /*设置主动曝光*/
//    setExposureTime(false);

//    /*设置图像分辨率*/
//    setROI((1920 - mWidth) / 2, (1200 - mHeight) / 2, mWidth, mHeight);

//    /* 创建流对象 */
//    createStreamObj();

//    /* 开始取图 */
//    if (!mImgGetThread->startGrabbing()) {
//        std::cout << "fail to start grabbing" << std::endl;
//    }
//}
//bool DahuaCamera::autoSetCameraIP() {
//    Dahua::GenICam::IGigECameraPtr gigeCameraPtr = Dahua::GenICam::IGigECamera::getInstance(mCameraSptr);

//    if (NULL == gigeCameraPtr) return false;

//    //获取Gige相机相关信息
//    Dahua::Infra::CString ip         = gigeCameraPtr->getIpAddress();
//    Dahua::Infra::CString subnetMask = gigeCameraPtr->getSubnetMask();
//    Dahua::Infra::CString gateway    = gigeCameraPtr->getGateway();
//    Dahua::Infra::CString macAddress = gigeCameraPtr->getMacAddress();
//    std::cout<<"ip address : "<<ip.c_str()<<std::endl;
//    std::cout<<"subnet mask : "<<subnetMask.c_str()<<std::endl;
//    std::cout<<"gateway : "<<gateway.c_str()<<std::endl;
//    std::cout<<"mac address : "<<macAddress.c_str()<<std::endl;

//    unsigned long devIpValue = ntohl(inet_addr(gigeCameraPtr->getIpAddress().c_str()));
//    unsigned long devSubMaskValue = ntohl(inet_addr(gigeCameraPtr->getSubnetMask().c_str()));

//    //获取对应接口的网卡信息
//    Dahua::GenICam::IGigEInterfacePtr gigeInterfaceSPtr = Dahua::GenICam::IGigEInterface::getInstance(mCameraSptr);

//    if (NULL == gigeInterfaceSPtr) return false;

//    Dahua::Infra::CString interfaceIp         = gigeInterfaceSPtr->getIpAddress();
//    Dahua::Infra::CString interfaceSubnetMask = gigeInterfaceSPtr->getSubnetMask();
//    Dahua::Infra::CString interfaceGateway    = gigeInterfaceSPtr->getGateway();
//    Dahua::Infra::CString interfaceMacAddress = gigeInterfaceSPtr->getMacAddress();
//    std::cout<<"ip address of interface : "<<interfaceIp.c_str()<<std::endl;
//    std::cout<<"subnet mask of interface : "<<interfaceSubnetMask.c_str()<<std::endl;
//    std::cout<<"gateway of interface : "<<interfaceGateway.c_str()<<std::endl;
//    std::cout<<"mac address of interface : "<<interfaceMacAddress.c_str()<<std::endl;


//    unsigned long InterfaceIpValue      = ntohl(inet_addr(gigeInterfaceSPtr->getIpAddress().c_str()));
//    unsigned long InterfaceSubMaskValue = ntohl(inet_addr(gigeInterfaceSPtr->getSubnetMask().c_str()));

//    if ((devIpValue & devSubMaskValue) != (InterfaceIpValue & InterfaceSubMaskValue)) {
//        //设备与网卡不在同一网段，强制设置设备与网卡在同一网段
//        unsigned char newIPStr[20] = { 0 };

//        while (true) {
//            unsigned long newIpValue = rand() % 254 + 1;  // 1~254
//            if (newIpValue != (InterfaceIpValue & 0xff)) {
//                newIpValue = (InterfaceIpValue & 0xffffff00) + newIpValue;
//                struct in_addr stInAddr;
//                stInAddr.s_addr = htonl(newIpValue);
//                memcpy(newIPStr, inet_ntoa(stInAddr), strlen(inet_ntoa(stInAddr)));
//                break;
//            }
//        }

//        if (!gigeCameraPtr->forceIpAddress(( const char* )newIPStr, gigeInterfaceSPtr->getSubnetMask().c_str(), gigeInterfaceSPtr->getGateway().c_str())) {
//            std::cout<<"set device ip failed"<<std::endl;
//            return false;
//        }
//    }

//    return true;
//}

//void DahuaCamera::displayDeviceInfo() {
//    for (int cameraIndex = 0; cameraIndex < mCameraPtrList.size(); cameraIndex++) {
//        mCameraSptr = mCameraPtrList[cameraIndex];
//        /* Idx 设备列表的相机索引 最大表示字数：3 */
//        std::cout<<"camera device index" <<cameraIndex + 1<<" : ";

//        /* Type 相机的设备类型（GigE，U3V，CL，PCIe）*/
//        switch (mCameraSptr->getType())
//        {
//        case Dahua::GenICam::ICamera::typeGige:
//            std::cout<<"GigE ";
//            break;
//        case Dahua::GenICam::ICamera::typeU3v:
//            std::cout<<"U3V ";
//            break;
//        case Dahua::GenICam::ICamera::typeCL:
//            std::cout<<"CL ";
//            break;
//        case Dahua::GenICam::ICamera::typePCIe:
//            std::cout<<"PCIe ";
//            break;
//        default:
//            break;
//        }
//        std::cout<<std::endl;

//        /* VendorName 制造商信息 最大表示字数：10 */
//        const char* vendorName = mCameraSptr->getVendorName();
//        char vendorNameCat[11];
//        if (strlen(vendorName) > 10) {
//            strncpy(vendorNameCat, vendorName, 7);
//            vendorNameCat[7] = '\0';
//            strcat(vendorNameCat, "...");
//            std::cout<<vendorNameCat<<" ";
//        }


//        /* ModeName 相机的型号信息 最大表示字数：10 */
//        std::cout<<mCameraSptr->getModelName()<<" ";
//        /* Serial Number 相机的序列号 最大表示字数：15 */
//        std::cout<<mCameraSptr->getSerialNumber()<<" ";


//        /* deviceUserID 自定义用户ID 最大表示字数：15 */
//        const char* deviceUserID = mCameraSptr->getName();
//        char deviceUserIDCat[16];
//        if (strlen(deviceUserID) > 15){
//            strncpy(deviceUserIDCat, deviceUserID, 12);
//            deviceUserIDCat[12] = '\0';
//            strcat(deviceUserIDCat, "...");
//        }else { //防止console显示乱码,UTF8转换成ANSI进行显示
//            memcpy(deviceUserIDCat, deviceUserID, sizeof(deviceUserIDCat));
//        }

//        /* IPAddress GigE相机时获取IP地址 */
//        Dahua::GenICam::IGigECameraPtr gigeCameraPtr = Dahua::GenICam::IGigECamera::getInstance(mCameraSptr);
//        if (NULL != gigeCameraPtr.get()) {
//            Dahua::Infra::CString ip = gigeCameraPtr->getIpAddress();
//            std::cout<<ip.c_str();
//        }

//    }
//}

//bool DahuaCamera::setGrabMode(const bool& bContious) {
//    Dahua::GenICam::IAcquisitionControlPtr sptrAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(mCameraSptr);
//    if (NULL == sptrAcquisitionControl) return false;

//    if (!sptrAcquisitionControl->triggerSelector().setValueBySymbol("FrameStart")) {
//        std::cout<<"set triggers selector fail."<<std::endl;
//        return false;
//    }

//    if (bContious) {
//        if (!sptrAcquisitionControl->triggerMode().setValueBySymbol("Off")) {
//            std::cout<<"set trigger mode fail."<<std::endl;
//            return false;
//        }
//    } else {
//        if (!sptrAcquisitionControl->triggerMode().setValueBySymbol("On")) {
//            std::cout<<"set trigger mode fail."<<std::endl;
//            return false;
//        }

//        /* 设置触发源为软触发（硬触发为Line1） */
//        if (!sptrAcquisitionControl->triggerSource().setValueBySymbol("Software")) {
//            std::cout<<"set trigger source fail."<<std::endl;
//            return false;
//        }
//    }
//    return true;
//}


//bool DahuaCamera::setExposureTime(const bool& bAutoExposure) {
//    Dahua::GenICam::IAcquisitionControlPtr sptrAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(mCameraSptr);
//    if (NULL == sptrAcquisitionControl) return false;

//    if (bAutoExposure) {
//        if (!sptrAcquisitionControl->exposureAuto().setValueBySymbol("Continuous")) {
//            std::cout<<"set exposureAuto fail."<<std::endl;
//            return false;
//        }
//    } else {
//        if (!sptrAcquisitionControl->exposureAuto().setValueBySymbol("Off")) {
//            std::cout<<"set exposureAuto fail."<<std::endl;
//            return false;
//        }
//        if (!sptrAcquisitionControl->exposureTime().setValue(mExposure)) {
//            std::cout<<"set exposureTime fail."<<std::endl;
//            return false;
//        }
//    }
//    return true;
//}


//bool DahuaCamera::setROI(const int64_t& nX, const int64_t& nY, const int64_t& nWidth, const int64_t& nHeight){
//    Dahua::GenICam::IImageFormatControlPtr sptrImageFormatControl = Dahua::GenICam::CSystem::getInstance().createImageFormatControl(mCameraSptr);
//    if (NULL == sptrImageFormatControl) return false;

//    /* width */
//    if (!sptrImageFormatControl->width().setValue(nWidth)) {
//        std::cout<<"set width fail."<<std::endl;
//        return false;
//    }

//    /* height */
//    if (!sptrImageFormatControl->height().setValue(nHeight)) {
//        std::cout<<"set height fail."<<std::endl;
//        return false;
//    }

//    /* OffsetX */
//    if (!sptrImageFormatControl->offsetX().setValue(nX)) {
//        std::cout<<"set offsetX fail."<<std::endl;
//        return false;
//    }

//    /* OffsetY */
//    if (!sptrImageFormatControl->offsetY().setValue(nY)) {
//        std::cout<<"set offsetY fail."<<std::endl;
//        return false;
//    }

//    return true;
//}
