#include "../../../../include/module/buff/Manager/EkfManager.hpp"

EkfManage::EkfManage()

{
    ekf=ExtendedKalmanFilter();
//    ekf_=ExtendedKalman();
//    ekf_gns=ExtendedKalman();
//    gns=GaussNewtonSolver();



}

void EkfManage::EkfInit()
{
    // 初始状态 (x, y, vx, vy)
    ekf.getState().at<float>(0) = 0;  // x
    ekf.getState().at<float>(1) = 0;  // y
    ekf.getState().at<float>(2) = 0;  // vx
    ekf.getState().at<float>(3) = 0;  // vy
}

void EkfManage::RunKalman(const cv::Point2f& center)
{
    //EKF
    // 转换为测量值
    //|x|
    //|y|
    cv::Mat measurement(2, 1, CV_32F);
    measurement.at<float>(0) = center.x;
    measurement.at<float>(1) = center.y;
    //cv::Mat measurement = (cv::Mat_<float>(2,1) << center.x,center.y);
    // 预测
    ekf.predict();
    // 校正
    ekf.correct(measurement);

    // 获取预测状态
    cv::Mat state = ekf.getState();
    cv::Point2f predictPt(state.at<float>(0), state.at<float>(1));
    predict_point=predictPt;
    //return predictPt;
}

cv::Point2f EkfManage::GetPredicrPoint(){
    return predict_point;
}

//void EkfManage::runPredictor(double t)
//{
//    // abs
//    double true_theta = -0.9/1.94*cos(1.94*t)+(2.09-0.9)*t;
//    double meas_theta=true_theta;
//    predictor.process_measurement(meas_theta);
//    predictor.optimize_params();
//    double future_t = 0.1f;
//    double pred_theta=predictor.predict(future_t);

//}
