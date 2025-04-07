#include "../../../../include/module/buff/PredictSolver/DFtransform.hpp"

void AnglePredictor::addAngle(double angle,double timestamp)
{
    angleWindow.push_back(angle);
    lastTime = timestamp;
    //std::cout<<"angleWindow.size(): "<<angleWindow.size()<<std::endl;
    if (angleWindow.size() > windowSize) {
        angleWindow.erase(angleWindow.begin());
    }
}

void AnglePredictor::computeFrequency()
{
    if (angleWindow.size() < windowSize) return;

    Eigen::VectorXd data(windowSize);
    for (int i = 0; i < windowSize; ++i) {
         data[i] = angleWindow[i];
    }
    // 应用汉宁窗
    applyHanningWindow(data);

    // 执行FFT
    Eigen::FFT<double> fft;
    //std::vector<std::complex<double>> spectrum(windowSize);
    Eigen::VectorXcd spectrum(windowSize);
    Eigen::VectorXcd data_complex = data.cast<std::complex<double>>();
    fft.fwd(spectrum, data_complex);
    // 寻找在[1.884, 2.000]范围内的主频
            int maxIndex = -1;
            double maxAmp = 0;
            const int numBins = spectrum.size() / 2;

            for (int k = 1; k < numBins; ++k) { // 跳过直流分量
                // 计算实际角频率
                double omega = 2 * M_PI * k / (windowSize * deltaT);

                if (omega >= 1.884 && omega <= 2.000) {
                    double amp = abs(spectrum[k]);
                    if (amp > maxAmp) {
                        maxAmp = amp;
                        maxIndex = k;
                    }
                }
            }
            // 抛物线插值提高精度
                    if (maxIndex != -1 && maxIndex > 0 && maxIndex < numBins-1) {
                        double y0 = abs(spectrum[maxIndex-1]);
                        double y1 = abs(spectrum[maxIndex]);
                        double y2 = abs(spectrum[maxIndex+1]);

                        double delta = 0.5 * (y0 - y2) / (y0 - 2*y1 + y2);
                        double kInterp = maxIndex + delta;
                        currentOmega = 2 * M_PI * kInterp / (windowSize * deltaT);
                    } else if (maxIndex != -1) {
                        currentOmega = 2 * M_PI * maxIndex / (windowSize * deltaT);
                    }

                    // 确保结果在有效范围内
                    currentOmega = std::clamp(currentOmega, 1.884, 2.000);
}

double AnglePredictor::predictNextAngle()
{
    if (angleWindow.size() < windowSize) return NAN;

            // 构建回归矩阵
            Eigen::MatrixXd X(windowSize, 4);
            Eigen::VectorXd y(windowSize);

            for (int i = 0; i < windowSize; ++i) {
                double t = i * deltaT;
                double wt = currentOmega * t;

                X(i, 0) = sin(wt);
                X(i, 1) = cos(wt);
                X(i, 2) = t;        // 线性项 bt
                X(i, 3) = 1.0;      // 常数项 c

                y(i) = angleWindow[i];
            }

            // 最小二乘求解
             Eigen::Vector4d coeffs = X.householderQr().solve(y);
             //Eigen::Vector4d coeffs = X.jacobiSvd(ComputeThinU | ComputeThinV).solve(y);
                    // 预测下一时刻
                    double nextT = /*windowSize*/lastTime + deltaT;
                    double nextWt = currentOmega * nextT;

                    return coeffs[0] * sin(nextWt) +
                           coeffs[1] * cos(nextWt) +
                           coeffs[2] * nextT +
                           coeffs[3];
}

double AnglePredictor::getCurrentOmega()const{
    return currentOmega;
}

void AnglePredictor::applyHanningWindow(Eigen::VectorXd &data)
{
    for (int i = 0; i < windowSize; ++i) {
                data[i] *= hanningWindow[i];
            }
}
