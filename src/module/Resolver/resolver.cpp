#include "../../../include/module/Resolver/resolver.hpp"

Resolver::Resolver() 
{
    this->CameraMatrix_ = (cv::Mat_<double>(3, 3) << 1150, 0, 660, 0, 1150, 512, 0, 0, 1);
    this->DistCoeffs_ = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
//    this->CameraMatrix_ = (cv::Mat_<double>(3, 3) << 1960.43296438033, 0, 0,
//                                                     -10.2808161880261, 1962.01633815233,0,
//                                                     396.704225631127, 237.488797790347, 1);
//    this->DistCoeffs_ = (cv::Mat_<double>(5, 1) << -0.239755448075257,
//                                                   4.76999358907361,
//                                                   -49.5478129025460,
//                                                   -0.00790568179531096,
//                                                   0.00990227840275078);
}

void Resolver::Dl_DistanceMeasurer(ArmorObject &object)
{
    Dl_SetWorldPoints();
    Dl_SolvePNP(object);
    Dl_DisToCenter(object);

}

void Resolver::Dl_SetWorldPoints()
{
//    // small // outpost
//    target_width=133;
//    target_height=55;

    //big
    target_width=228;
    target_height=55;

    points_in_world.clear();

    points_in_world.push_back(cv::Point3d (-target_width/2.0,target_height/2.0,0));
    points_in_world.push_back(cv::Point3d(-target_width / 2.0, -target_height / 2.0, 0));
    points_in_world.push_back(cv::Point3d(target_width / 2.0, -target_height / 2.0, 0));
    points_in_world.push_back(cv::Point3d(target_width / 2.0, target_height / 2.0, 0));


}

void Resolver::Dl_SolvePNP(ArmorObject&object)
{
    auto armorDetector=new ArmorDetector();
//    std::vector<cv::Point2f> object_points;
    object.object_points.push_back(armorDetector->FourPoints(&object.apex[0]));
    object.object_points.push_back(armorDetector->FourPoints(&object.apex[1]));
    object.object_points.push_back(armorDetector->FourPoints(&object.apex[3]));
    object.object_points.push_back(armorDetector->FourPoints(&object.apex[2]));

    solvePnP(points_in_world,object.object_points,CameraMatrix_,DistCoeffs_,rotate_mat,trans_mat,false,cv::SOLVEPNP_AP3P);

    //std::cout<<"points:"<<object_points[0].x<<","<<object.object_points[0].y<<std::endl;

    object.pose.translation().x()=trans_mat.ptr<double>(0)[0];
    object.pose.translation().y()=trans_mat.ptr<double>(0)[1];
    object.pose.translation().z()=trans_mat.ptr<double>(0)[2];

    // 旋转向量到旋转矩阵的转化    3*1/1*3 -> 3*3
    cv::Mat rotation_matrix;
    cv::Rodrigues(rotate_mat, rotation_matrix);

    Eigen::Matrix3d tf2_rotation_matrix;
    tf2_rotation_matrix << rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1),
            rotation_matrix.at<double>(1, 2), rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2);
    // 设置旋转部分
    object.pose.linear() = tf2_rotation_matrix;
    object.distance= sqrtf(pow(trans_mat.ptr<double>(0)[0],2)+
                             pow(trans_mat.ptr<double>(0)[1],2)+
                             pow(trans_mat.ptr<double>(0)[2],2));


}

void Resolver::Dl_DisToCenter(ArmorObject &object)
{
    auto detector=new ArmorDetector();
     object.center=detector->TargetCenter(object.apex);
   object.distance_to_target_center=cv::norm(object.center-cv::Point2f(image_width/2,image_height/2));
//   std::cout<<"center:"<<object.center.x<<","<<object.center.y<<std::endl;
//   std::cout<<"distance:"<<object.distance_to_target_center<<std::endl;

}

void Resolver::Dl_AimTraget(std::vector<ArmorObject> &objects)
{
//    ArmorObject object;

    ArmorObject temp=objects[0];

//    ArmorObject temp;

    for(auto&object:objects)
    {
        if(object.distance_to_target_center<temp.distance_to_target_center)
        {
            temp=object;
        }

    }

    send_distance=temp.distance;

    init();



    if(send_distance < 1500)
    {
        send_yaw=(-atan(temp.pose.translation().x()/temp.distance)-0.013f)*180/CV_PI;

        send_pitch=Transform(temp.pose.translation().z(),(temp.pose.translation().y() -0.0f)) * 180/CV_PI;

    }
    else if(send_distance >= 1500 && send_distance < 2000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 5.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 2000 && send_distance < 2500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 8.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 2500 && send_distance < 3000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 10.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 3000 && send_distance < 3500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 11.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 3500 && send_distance < 4000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 13.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 4000 && send_distance < 4500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 15.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 4500 && send_distance < 5000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 17.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 5000 && send_distance < 5500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 19.0f)) * 180 / CV_PI;
    }

}
