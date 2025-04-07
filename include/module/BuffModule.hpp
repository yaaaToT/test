#ifndef BUFFMODULE_H
#define BUFFMODULE_H

#include <iostream>
#include "opencv2/opencv.hpp"

#include "inference/YOLO11module.hpp"
#include "TorosamyModule.hpp"
#include "buff/Detector/Detector.hpp"
#include "buff/Detector/BuffArmor.hpp"
#include "buff/Manager/BuffManager.hpp"
#include "buff/Manager/EkfManager.hpp"
#include "buff/PredictSolver/ExtendedKalman.hpp"
#include"../pod/multiton/package/SendDataPacket.hpp"
#include"../pod/multiton/package/ReceiveDataPacket.hpp"
#include"../pod/multiton/package/DataPacket.hpp"

struct TargetPoint{
    cv::Point2f target_point;
    double tx,ty,tz;
    float yaw;
    float pitch;
};

class BuffModule:public TorosamyModule{
public:
    BuffModule();
    void run() override;
    void runfailed(std::shared_ptr<SendDataPacket>& dataPacket,std::shared_ptr<ReceiveDataPacket>&RdataPacket);
    void updateSendBuffArmor(std::shared_ptr<SendDataPacket>& sendDataPacket,std::shared_ptr<ReceiveDataPacket>& receiveDataPacket);
    void drawParams(cv::Mat& outSrc, const TIME& inputTime,
                        const std::shared_ptr<ReceiveDataPacket>& receiveDataPacket,
                        const std::shared_ptr<SendDataPacket>& sendDataPacket);
    bool IsFirstFan(std::shared_ptr<SendDataPacket>& sendDataPacket);
    auto time();


    Detector detector;// Traditional vision
    Yolo11Module yolo11module;// Neural networks

    BuffArmor armor;

    EkfManage EKF;


};

#endif //BUFFMODULE_H
