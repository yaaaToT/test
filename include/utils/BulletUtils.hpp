//
// Created by torosamy on 24-11-12.
//

#ifndef BULLETUTILS_HPP
#define BULLETUTILS_HPP



class BulletUtils
{
public:
    // 删除构造函数和析构函数，禁止实例化
    BulletUtils() = delete;
    ~BulletUtils() = delete;
    struct Bullets{
        float x;//yaw位置
        float y;//pitch位置
        float vx;//x速度
        float vy;//y速度
    };

    // k1
    // d
    // v
    // theta

    static float bulletModel(const float &depth,const float& speed, const float &pitch,float& outFlyTime);
    static float getPitch(const float& depth, const float& height, const float& speed,float& time);
    static double transform(const float& depth, const float& height ,float& time);
    static float AdjustPitch(const float& distance,const float& pitch);

    static  float V;  //24 25 弹速
    static const float Rho;//空气密度(kg/m^3)
    static const float Cd;//阻力系数
    static const float diameter;//弹丸直径(m)
    //用0.02510132乘当前空气密度
    //23.7摄氏度下为1.192
    //15摄氏度下为1.225
    static const float K; //0.0300f,0269f,0.020
    static const float PI; //π
    static const float  GRAVITY;        //重力g
    static const float e;
};



#endif //BULLETUTILS_HPP
