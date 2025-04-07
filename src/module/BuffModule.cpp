#include "../../include/module/BuffModule.hpp"
#include "../../include/Robot.hpp"
#include "../../include/pod/multiton/camera/VideoCamera.hpp"
#include "../../include/pod/singleton/manager/CameraManager.hpp"
#include "../../include/module/inference/YOLO11module.hpp"
#include "../../include/utils/BulletUtils.hpp"
#include "../../include/utils/MessageUtils.hpp"

BuffModule::BuffModule(){
    std::string configLocation=getConfigLocation("test");
    cv::FileStorage fileReader(configLocation, cv::FileStorage::READ);
    //可以写入相关调控操作手的参数
    fileReader.release();
}

TargetPoint tPoint;

void BuffModule::run(){
     cv::Mat src;
     std::shared_ptr<Robot> robot = Robot::getInstance();
     PacketManager& receivePacketManager = robot->getReceivePacketManager();
     PacketManager& sendPacketManager = robot->getSendPacketManager();
     std::shared_ptr<VideoCamera> cameraPtr = Robot::getInstance()->getCameraManager().getCameraById<VideoCamera>(0);

     TIME Ktime;
     double TimeLogs=0;
     bool flag=0;
     cv::namedWindow("src",cv::WINDOW_NORMAL);
     cv::resizeWindow("src",720,480);
     cv::namedWindow("mask",cv::WINDOW_NORMAL);
     cv::resizeWindow("mask",720,480);

     EKF.EkfInit();
     while(true){
        TIME time=MessageUtils::getTimePoint();
        auto current=std::chrono::high_resolution_clock::now();
        if(!src.empty())
//            imshow("src",src);
        {
            std::shared_ptr<ReceiveDataPacket> receiveDataPacket = receivePacketManager.getPacketById<ReceiveDataPacket>(0);
            std::shared_ptr<SendDataPacket> sendDataPacket = sendPacketManager.getPacketById<SendDataPacket>(0);
            cv::Mat mask=detector.getSpliteImg(src,ColorType::Blue);
            imshow("mask",mask);
            if(!detector.findCenterPoints(src,mask)&&detector.isComplete())
            {
                flag=0;
                cv::putText(src,"State: All activated",cv::Point2f(0,122),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,0),1);
            }
            else
                flag=1;

            // 判断大小赋值
            if(detector.GetMode()==1)
            {
                //std::cout<<"small"<<std::endl;/*detector.GetRectPoints();*/
                armor.CenterPoints=detector.GetSmallBuffPoints();
                armor.buff_center=detector.GetSmallBuffPointsCenter();
            }
            else
            {
                //std::cout<<"large"<<std::endl;
                armor.CenterPoints=detector.GetLargeBuffPoints();
                armor.buff_center=detector.GetLargeBuffPointsCenter();
            }

            armor.updateDistanceAndPose(armor,cameraPtr->getCameraMatrix(),cameraPtr->getDistCoeffs());

//            if(!yolo11module.run(src))
//            {
//               std::cout<<"not find"<<std::endl;
//            }

            if(!yolo11module.GetArmorType()&&flag==1)
            {
               cv::putText(src,"State: Not activated",cv::Point2f(0,122),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,0),1);
               // 选择目标中心点
               EKF.RunKalman(detector.GetBuffCenter());
               // 选择预判点
//               EKF.RunKalman(detector.GetLargeBuffPointsCenter());
//               EKF.RunKalman(detector.GetSmallBuffPointsCenter());
               circle(src,EKF.GetPredicrPoint(),8,cv::Scalar(0,255,255),2,8);//圈
            }
            imshow("src",src);
        }
        std::shared_ptr<ReceiveDataPacket> receiveDataPacket = receivePacketManager.getPacketById<ReceiveDataPacket>(0);
        std::shared_ptr<SendDataPacket> sendDataPacket = sendPacketManager.getPacketById<SendDataPacket>(0);

        if(cameraPtr!=nullptr)
            cameraPtr->cloneSrc(src);

       BuffModule::updateSendBuffArmor(sendDataPacket,receiveDataPacket);

       drawParams(src,time,receiveDataPacket,sendDataPacket);

     }

}


void BuffModule::runfailed(std::shared_ptr<SendDataPacket>& dataPacket,std::shared_ptr<ReceiveDataPacket>&RdataPacket)
{
     // 刷新数据
     // 重置
     dataPacket->yaw.f=0.f;

     dataPacket->pitch.f=0.f;

     dataPacket->ifShoot=0;

}

void BuffModule::updateSendBuffArmor(std::shared_ptr<SendDataPacket>& sendDataPacket,std::shared_ptr<ReceiveDataPacket>&receiveDataPacket)
{

    double x_=armor.Pose.translation().x();
    double y_=armor.Pose.translation().y();
    double z_=armor.Pose.translation().z();

//    if(IsFirstFan(sendDataPacket))
//    {
//        if(sendDataPacket->ifFirstArmor==0){
            tPoint.target_point=detector.GetBuffCenter();
            tPoint.tx=armor.Pose.translation().x();//坐标x
            tPoint.ty=armor.Pose.translation().y();//坐标y
            tPoint.tz=armor.Pose.translation().z();//坐标z
            //sendDataPacket->ifFirstArmor=1;
//        }
//    }
//    double x_=armor.Pose.translation().x();
//    double y_=armor.Pose.translation().y();
//    double z_=armor.Pose.translation().z();

    // 得出偏航角
    float sendYaw=(-atan(x_/z_))*180/CV_PI;
    // 俯仰角
    float sendPitch=(-atan(y_/z_))*180/CV_PI;
    // 得出距离
    float sendDistance=sqrtf(tPoint.tx * tPoint.tx +tPoint.ty * tPoint.ty + tPoint.tz * tPoint.tz);

    // 计算调整
    // 角度
    //float sendPitch=0,offsets_Angle=0/*,Fly_Time=0,Actural_pitch=0*/;
    //float sendYaw,offsets_Angle=0;

    //float adjust_params=10;

//    if(sendDataPacket->ifFirstArmor==1)
//    {
//        offsets_Angle=10;

//    }

    //sendYaw=offsets_Angle+(-atan(x_/z_))*180/CV_PI;

    //float sendYaw=(-atan(x_/z_))*180/CV_PI;
    //sendPitch=offsets_Angle+(-atan(y_/z_)) * 180 /CV_PI;

    // 更新目标属性

    //设置发送数据包
    sendDataPacket->distance.f=sendDistance / 1000.0;
    sendDataPacket->x.f=x_*1.0f;
    sendDataPacket->y.f=y_*1.0f;
    sendDataPacket->z.f=z_*1.0f;

//    if(cv::norm(EKF.GetPredicrPoint()-tPoint.target_point)<3){ //4 准的一般是2.多
//        sendDataPacket->ifRealShoot=1;
//        sendDataPacket->ifRealShoot=0;
//    }
//    else
//        sendDataPacket->ifRealShoot=0;


    // 调整
    // 角度补偿
    std::vector<float>offsets;
    offsets={1.0f,-0.8f,0.8f,1.0f,8.0f,1.0f};

    // 给出偏航角
    sendDataPacket->yaw.f=sendYaw;
    // yaw要往右多补偿

//    if(sendDataPacket->ifFirstArmor==0){
//    sendDataPacket->yaw.f=sendYaw * offsets[0] + offsets[1];
//    sendDataPacket->yaw.f=sendDataPacket->yaw.f * offsets[2];

//    // 给出俯仰角
    sendDataPacket->pitch.f = sendPitch;
//    // pitch往上稍补偿
//    sendDataPacket->pitch.f = sendPitch  * offsets[3] + offsets[4];
//    }


}

void BuffModule::drawParams(cv::Mat &outSrc, const TIME &inputTime, const std::shared_ptr<ReceiveDataPacket> &receiveDataPacket, const std::shared_ptr<SendDataPacket> &sendDataPacket)
{
    double time=MessageUtils::getTimeByPoint(inputTime);
    double fps=MessageUtils::getFpsByTime(time);

    MessageUtils::putText(outSrc, "FPS : " + std::to_string(fps), cv::Point2f(0, 22));
    MessageUtils::putText(outSrc, "Pitch:" + std::to_string(sendDataPacket->pitch.f), cv::Point2f(0, 62));
    MessageUtils::putText(outSrc, "Yaw:" + std::to_string(sendDataPacket->yaw.f), cv::Point2f(0, 82));
    MessageUtils::putText(outSrc, "distance:" + std::to_string(sendDataPacket->distance.f), cv::Point2f(0, 102));
    MessageUtils::putText(outSrc, "Time:" + std::to_string(time), cv::Point2f(0, 42));
//    MessageUtils::putText(outSrc, "if_shoot:" + std::to_string(sendDataPacket->ifShoot), cv::Point2f(0, 122));
//    MessageUtils::putText(outSrc, "if_real_shoot:" + std::to_string(sendDataPacket->ifRealShoot), cv::Point2f(0, 142));
//    MessageUtils::putText(outSrc, "gain_yaw:" + std::to_string(receiveDataPacket->gainYaw.f), cv::Point2f(0, 162));
//    MessageUtils::putText(outSrc, "gain_pitch:" + std::to_string(receiveDataPacket->gainPitch.f), cv::Point2f(0, 182));

//    MessageUtils::putText(outSrc, "speed:" + std::to_string(receiveDataPacket->speed), cv::Point2f(0, 202));
//    MessageUtils::putText(outSrc, "ifFirstArmor:" + std::to_string(sendDataPacket->ifFirstArmor), cv::Point2f(0, 222));
//    cv::line(outSrc, cv::Point2f(outSrc.size().width / 2, 0), cv::Point2f(outSrc.size().width / 2, outSrc.size().height), cv::Scalar(0, 255, 0), 1, 8);
//    cv::line(outSrc, cv::Point2f(0, outSrc.size().height / 2), cv::Point2f(outSrc.size().width, outSrc.size().height / 2), cv::Scalar(0, 255, 0), 1, 8);

}



bool BuffModule::IsFirstFan(std::shared_ptr<SendDataPacket> &sendDataPacket)
{
    if(sendDataPacket->pitch.f!=0)
        if(sendDataPacket->yaw.f!=0)
            return true;
    return false;
}
