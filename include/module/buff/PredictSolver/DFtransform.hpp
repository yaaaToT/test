/**
 * a * sin(w * (t+phi)) + 2.090 - a
 */

#ifndef DFTRANSFORM_HPP
#define DFTRANSFORM_HPP

#include "iostream"
#include <cmath>
#include <complex>
#include "opencv2/opencv.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>

class AnglePredictor
{
public:
    AnglePredictor()=default;
    AnglePredictor(double dt,int size)
        :deltaT(dt),windowSize(size),currentOmega(2.000),lastTime(0)
    {
        // 初始化汉宁窗
        hanningWindow.resize(windowSize);
        for (int i = 0; i < windowSize; ++i) {
             hanningWindow[i] = 0.5 * (1 - cos(2 * M_PI * i / (windowSize - 1)));
        }
    }

    void addAngle(double angle,double timestamp);

    void computeFrequency();

    double predictNextAngle();

    double getCurrentOmega()const;


private:
    void applyHanningWindow(Eigen::VectorXd& data);
    std::vector<double> angleWindow;  // 滑动窗口存储角度数据
    double deltaT;               // 采样时间间隔
    int windowSize;              // 窗口大小
    double currentOmega;         // 当前估计的角频率ω
    std::vector<double> hanningWindow;// 汉宁窗系数
    double lastTime;

};



#endif // DFTRANSFORM_HPP
