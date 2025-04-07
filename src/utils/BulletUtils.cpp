#include "../../include/utils/BulletUtils.hpp"
#include <opencv2/opencv.hpp>

const float BulletUtils::K = 0.019f;  // 空气阻力相关系数 球体 0.019
float BulletUtils::V = 24.0f;   // 弹丸初始速度v0 上限25 m/s
const float BulletUtils::PI = 3.1415926535;  // 派
const float BulletUtils::GRAVITY = 9.78f;  //9.8   // 重力加速度
const float BulletUtils::e = 2.718281828;    //自然常数

float BulletUtils::bulletModel(const float& depth,const float& speed, const float &pitch,float& outFlyTime){
    outFlyTime = ( float )((exp(K * depth) - 1) / (K * speed * cos(pitch)));
    return ( float )((speed * sin(pitch) * outFlyTime - GRAVITY * outFlyTime * outFlyTime / 2));

}

float BulletUtils::getPitch(const float& depth, const float& height, const float& speed,float& time) {
    //未计算y轴空气阻力
    //用tempHeight更新result 再用result更新dy 再用dy更新tempHeight
    float tempHeigh = height;
    float result = 0;
    float dy;


    for (int i = 0; i < 20; i++) {
        result = (float)atan2(tempHeigh, depth);
        dy = height - bulletModel(depth, speed, result, time);
        tempHeigh += dy;

        if (fabsf(dy) < 0.001f) break;
    }
    return result;
}

double BulletUtils::transform(const float& depth, const float& height ,float& time) {
    double pitch = -( float )getPitch(depth / 1000, -height / 1000, V, time);
    return pitch;
}


float BulletUtils::AdjustPitch(const float& distance,const float& pitch){
    //垂直位移
    float Dy= distance*std::sin(pitch*CV_PI/180) / 1000;
    //水平位移
    float Dx=distance*std::cos(pitch*CV_PI/180) / 1000;
    //下落高度
    float h=GRAVITY*Dx*Dx/(2*V*V);
    float tan_phi=(Dx-std::sqrt(Dx*Dx-4*h*(h+Dy)))/(2*h);
    //补偿角
    float phi=std::atan(tan_phi)*180/CV_PI;
    return phi;
}


// 发射角度调整

//float BulletUtils::AdjustPitch(const float& distance,const float& pitch){
//    //垂直位移
//    float Dy= distance*std::sin(pitch*CV_PI/180) / 1000;
//    //水平位移
//    float Dx=distance*std::cos(pitch*CV_PI/180) / 1000;
//    //下落高度
//    float h=GRAVITY*Dx*Dx/(2*V*V);
//    float tan_phi=(Dx-std::sqrt(Dx*Dx-4*h*(h+Dy)))/(2*h);
//    //补偿角
//    float phi=std::atan(tan_phi)*180/CV_PI;
//    return phi;

//}
