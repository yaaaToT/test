#ifndef DESIGN_RESOLVER_H
#define DESIGN_RESOLVER_H
//#include "../armor_detector/armor.h"
#include "../inference/inference.hpp"
#include "eigen3/Eigen/Eigen"
#include "GimbalControl.h"

class Resolver : public GimbalControl{
public:
    Resolver();
//    void DistanceMeasurer(Armor &armor);
//    void SetWorldPoints(ArmorType armor_type);

    void Dl_DistanceMeasurer(ArmorObject&object);

    void Dl_SetWorldPoints();

//    void SolvePNP(Armor& armor);

    void Dl_SolvePNP(ArmorObject &object);

//    void Tv_DisToCenter(Armor& armor);

    void Dl_DisToCenter(ArmorObject &object);

//    void AimTraget(std::vector<Armor>& armors);

    void Dl_AimTraget(std::vector<ArmorObject>&objects);


    cv::Mat CameraMatrix_;      // 相机内参矩阵
    cv::Mat DistCoeffs_;        // 相机畸变矩阵

    std::vector<cv::Point3d> point_in_world_;   // 以装甲板中心为原点的坐标系
    double target_width_;
    double target_height_;

    cv::Mat rotate_mat_;        // 旋转矩阵
    cv::Mat trans_mat_;         // 平移矩阵

    // in use
    std::vector<cv::Point3d> points_in_world;
    double target_width;
    double target_height;
    cv::Mat rotate_mat;
    cv::Mat trans_mat;

    float send_yaw;
    float send_pitch;
    float send_distance;

    float bullet_fight_time;
};


#endif //DESIGN_RESOLVER_H