#ifndef EKFMANAGER_HPP
#define EKFMANAGER_HPP

//#include "../Detector/Detector.hpp"
#include "../PredictSolver/ExtendedKalman.hpp"
#include "../PredictSolver/GaussNewtonSolver.hpp"
//#include "../PredictSolver/tracker.hpp"

//#define PI 3.1415926
//#define R 700.0
//#define OMEGA 0.0

class EkfManage
{
public:
    EkfManage();
    //--------------------------------------------
    void EkfInit();
    void RunKalman(const cv::Point2f& center);
    cv::Point2f GetPredicrPoint();   
    //--------------------------------------------

    //void runPredictor(double t);

    ExtendedKalmanFilter ekf;

    //AnglePredictor predictor;

private:

    cv::Point2f predict_point;

};



#endif //EKFMANAGER_HPP
