#ifndef EXTENDEDKALMAN_HPP
#define EXTENDEDKALMAN_HPP

#include<Eigen/Dense>
#include<functional>
#include<cmath>

#include "../../../../include/utils/MessageUtils.hpp"

// 参数调整
// 过程噪声Q: 加速度变化越大,Q应设置越大
// 测量噪声R: 检测点噪声越大,R值需增大
// 时间步长dt: 需与帧率匹配(fps=30,dt=1/30=0.033)

//·

typedef std::chrono::steady_clock::time_point TIME;

class ExtendedKalmanFilter{
public:
    ExtendedKalmanFilter(int stateDim=6,int measDim=2,double dt=0.1) //状态维度4&6,观测维度2
        :stateSize(stateDim),measSize(measDim),dt(dt) {

        // 初始化状态向量和协方差矩阵
        // [0,0,0,0,0,0]
        x = cv::Mat::zeros(stateDim, 1, CV_32F); // 状态 [x, y, vx, vy, ax, ay]

        P = cv::Mat::eye(stateDim,stateDim,CV_32F)*1e-1; // 初始协方差矩阵

        // 过程噪声协方差矩阵 (假设加速度噪声为主)

        Q = cv::Mat::eye(stateDim, stateDim, CV_32F) * 1e-2; //1 3

        // 测量噪声协方差矩阵
        // [1 0
        //  0 1]
        R = cv::Mat::eye(measDim, measDim, CV_32F) * 1e-1;
    }

        // 非线性状态转移函数 匀速运动模型 stateDim=4
        cv::Mat stateTransition(const cv::Mat& state) {
            cv::Mat newState = state.clone();
            newState.at<float>(0) += state.at<float>(2) * dt;  // x += vx * dt
            newState.at<float>(1) += state.at<float>(3) * dt;  // y += vy * dt
            return newState;
        }

       // 非线性状态转移函数 匀加速模型 stateDim=6
//        cv::Mat stateTransition(const cv::Mat& state) {
//            cv::Mat newState = state.clone();
//            float ax=state.at<float>(4);
//            float ay=state.at<float>(5);
//            newState.at<float>(0) += state.at<float>(2) * dt + 0.5 * ax * dt * dt;
//            newState.at<float>(1) += state.at<float>(3) * dt + 0.5 * ay * dt * dt;
//            newState.at<float>(2) += ax * dt;  // vx += ax * dt
//            newState.at<float>(3) += ay * dt;  // vy += ay * dt
//            return newState;
//        }

        // 圆周运动模型
        // newState.x = x * cos(w*dt) - y*sin(w*dt);
        // newState.y = x * sin(w*dt) + y*cos(w*dt);

        // 非线性观测函数 (直接观测位置)
        cv::Mat measurementFunction(const cv::Mat& state) {
            cv::Mat meas(measSize, 1, CV_32F);
            meas.at<float>(0) = state.at<float>(0);  // x
            meas.at<float>(1) = state.at<float>(1);  // y
            return meas;
        }

       // 计算状态转移雅可比矩阵F
       // 匀速
       // cv::Mat F=(cv::Mat_<float>(4,4)) <<
       //   1,  0,  dt, 0
       //   0,  1,  0,  dt
       //   0,  0,  1,  0
       //   0,  0,  0,  1   );

       cv::Mat computeJacobianF(const cv::Mat& state) {
           cv::Mat F = cv::Mat::eye(stateSize, stateSize, CV_32F);
           F.at<float>(0, 2) = dt;  // dx/dvx = dt
           F.at<float>(1, 3) = dt;  // dy/dvy = dt
           return F;
       }

       // 匀加速
       // cv::Mat F=(cv::Mat_<float>(6,6) <<
       //   1,         0,         dt,        0,         0.5*dt*dt, 0
       //   0,         1,         0,         dt,        0,         0.5*dt*dt
       //   0,         0,         1,         0,         dt,        0
       //   0,         0,         0,         1,         0,         dt
       //   0,         0,         0,         0,         1,         0
       //   0,         0,         0,         0,         0,         1  );

//       cv::Mat computeJacobianF(const cv::Mat& state) {
//           cv::Mat F = cv::Mat::eye(stateSize, stateSize, CV_32F);
//           F.at<float>(0, 2) = dt;  // dx/dvx = dt
//           F.at<float>(0, 4) = 0.5 * dt * dt;  // dx/dax
//           F.at<float>(1, 3) = dt;  // dy/dvy = dt
//           F.at<float>(1, 5) = 0.5 * dt * dt;  // dy/day
//           F.at<float>(2, 4) = dt;  // dvx/dax
//           F.at<float>(3, 5) = dt;  // dvy/day

//           return F;
//       }

       // 预测步骤
       void predict(){
           // 状态预测
           x = stateTransition(x);
           // 协方差预测: P=F*P*F^T+Q
           cv::Mat F=computeJacobianF(x);
           P = F * P * F.t() + Q;
       }

       // 校正步骤
       void correct(const cv::Mat&z){
           // 计算观测雅可比矩阵 H
           cv::Mat H=cv::Mat::zeros(measSize,stateSize,CV_32F);
           H.at<float>(0, 0) = 1;  // dx/dx = 1
           H.at<float>(1, 1) = 1;  // dy/dy = 1

           // 计算卡尔曼增益: K = P * H^T * (H * P * H^T + R)^(-1)
           cv::Mat S = H * P * H.t() + R;
           cv::Mat K = P * H.t() * S.inv();

           // 更新状态和协方差
           cv::Mat y = z - measurementFunction(x);
           x = x + K * y;
           P = (cv::Mat::eye(stateSize, stateSize, CV_32F) - K * H) * P;
       }     

       // 获取当前状态
       cv::Mat getState() const {return x;}

private:
    int stateSize; // 状态维度(6: 4:(x, y, vx, vy), ax, ay)
    int measSize;  // 观测维度(2: x, y)
    double dt;     // 时间步长

    cv::Mat x;     // 状态向量
    cv::Mat P;     // 协方差矩阵
    cv::Mat Q;     // 过程噪声协方差
    cv::Mat R;     // 测量噪声协方差
};



//class ExtendedKalman
//{
//public:
//    ExtendedKalman()=default;
//    using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
//    using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;
//    using VoidMatFunc = std::function<Eigen::MatrixXd()>;

//    ExtendedKalman(const VecVecFunc& f,const VecVecFunc& h,const VecMatFunc& j_f,
//                   const VecMatFunc& j_h,const VoidMatFunc& u_q,const VecMatFunc& u_r,
//                   const Eigen::MatrixXd& P0):f_(f), h_(h), jacobian_f_(j_f), jacobian_h_(j_h),
//        update_Q_(u_q), update_R_(u_r), P_post_(P0),
//        n_(P0.rows()), I_(Eigen::MatrixXd::Identity(n_, n_)),
//        x_pri_(n_), x_post_(n_) {}

//    void setState(const Eigen::VectorXd& x0); //{x_post_ = x0;}

//    Eigen::VectorXd predict();

//    Eigen::VectorXd update(const Eigen::VectorXd& z);

//private:
//    VecVecFunc f_, h_;
//    VecMatFunc jacobian_f_, jacobian_h_;
//    VoidMatFunc update_Q_;
//    VecMatFunc update_R_;
//    Eigen::MatrixXd F_, H_, Q_, R_, K_, P_pri_, P_post_, I_;
//    Eigen::VectorXd x_pri_, x_post_;
//    int n_;
//};



//class ExtendedKalman
//{
//public:
//    ExtendedKalman() = default;
//    using /*StateFunc*/VedVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
//    //using MeasFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
//    using /*JacFunc*/VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;
//    //using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;
//    using /*NoiseFunc*/VoidMatFunc = std::function<Eigen::MatrixXd()>;
//    using NoiseFunc = std::function<Eigen::MatrixXd()>;

//    explicit ExtendedKalman(VedVecFunc f,VedVecFunc h,VecMatFunc j_f,VecMatFunc j_h,VoidMatFunc q, NoiseFunc r,
//                   const Eigen::MatrixXd& P0, const Eigen::VectorXd& x0)
//        :f_(f), h_(h), jac_f_(j_f), jac_h_(j_h),Q_(q), R_(r), P_post_(P0), x_post_(x0)
//    {
// //        n_ = x0.size();
// //        m_ = h(x0).size();
// //        I_ = Eigen::MatrixXd::Identity(n_,n_);
// //        // 初始化矩阵为动态维度
// //        F_ = Eigen::MatrixXd::Zero(n_, n_);
// //        H_ = Eigen::MatrixXd::Zero(m_, n_);
// //        K_ = Eigen::MatrixXd::Zero(n_, m_);
// //        P_pri_ = Eigen::MatrixXd::Zero(n_, n_);
//    }

//    void predict();
//    void update(const Eigen::VectorXd& z);

//    Eigen::VectorXd state()const;

//private:
//    int n_;
//    //int m_;
//    //Eigen::MatrixXd I_;
//    VedVecFunc f_;
//    VedVecFunc h_;
//    VecMatFunc jac_f_, jac_h_;
//    NoiseFunc Q_, R_;
//    //Eigen::MatrixXd Q,R;
//    //VecMatFunc R_;
//    Eigen::MatrixXd F_, H_, K_, P_pri_, P_post_;
//    Eigen::VectorXd x_pri_, x_post_;
//};




 // Copyright 2022 Chen Jun
class ExtendedKalman
{
public:
    ExtendedKalman()=default;

    using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;//向量
    using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;//矩阵
    using VoidMatFunc = std::function<Eigen::MatrixXd()>;//空矩阵

    // f 状态转移矩阵
    // j_f 状态转移雅可比矩阵
    // p 协方差矩阵
    // h 观测矩阵
    // k 增益矩阵
    // q 过程噪声矩阵
    // r 观测噪声矩阵
    explicit ExtendedKalman(
        const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
        const VoidMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0);

    // 设置修正状态
    void setState(const Eigen::VectorXd & x0);

    // 设置初始状态
    void setInitState(const Eigen::VectorXd & x0);

    // 计算预测状态
    Eigen::MatrixXd predict();

    // 根据测量更新估计状态
    Eigen::MatrixXd update(const Eigen::VectorXd & z);

private:
    // 过程非线性向量函数
    VecVecFunc f;
    // 观测非线性向量函数
    VecVecFunc h;
    // f（）的雅可比行列式
    VecMatFunc jacobian_f;
    Eigen::MatrixXd F;
    // h（）的雅可比行列式
    VecMatFunc jacobian_h;
    Eigen::MatrixXd H;
    // 过程噪声协方差矩阵
    VoidMatFunc update_Q;
    Eigen::MatrixXd Q;
    // 测量噪声协方差矩阵
    VecMatFunc update_R;
    Eigen::MatrixXd R;

    // 先验误差估计协方差矩阵
    Eigen::MatrixXd P_pri;
    // 后验误差估计协方差矩阵
    Eigen::MatrixXd P_post;

    // 卡尔曼增益
    Eigen::MatrixXd K;

    // 系统尺寸
    int n;

    // N 大小标识
    Eigen::MatrixXd I;

    // 先验状态
    Eigen::VectorXd x_pri;
    // 后验状态
    Eigen::VectorXd x_post;

};

//class ExtendedKalman
//{
//public:
//    ExtendedKalman()=default;

//    explicit ExtendedKalman(const Eigen::MatrixXd & P0)
//        :P_post(P0),n((int )P0.rows()),I(Eigen::MatrixXd::Identity(n, n)),x_pri(n),x_post(n){}

//    // 设置初始状态
//    void setState(const Eigen::VectorXd & x0){ x_post = x0; };

//    // 计算预测状态
//    Eigen::MatrixXd predict() {
//    Eigen::MatrixXd Q;
//    F = jacobian_f(), Q = update_Q();
//    //预测
//    //更新误差估计和估计误差协方差
//    x_pri = f();
//    P_pri = F * P_post * F.transpose() + Q;

//    // 处理在下一次预测之前没有测量的情况
//    x_post = x_pri;
//    P_post = P_pri;

//      return x_pri;
//    };

//    // 根据测量更新估计状态
//    Eigen::MatrixXd update(const Eigen::VectorXd & measurement) {
//    Eigen::MatrixXd R;
//    H = jacobian_h(), R = update_R();
//    //更新
//    //计算卡尔曼增益
//    Eigen::MatrixXd S = H * P_pri * H.transpose() + R;
//    K = P_pri * H.transpose() * S.inverse();

//    //更新状态估计矩阵和估计误差协方差
//    x_post = x_pri + K * (measurement - h(x_pri));
//    P_post = (I - K * H) * P_pri;

//    ekf_prediction = x_post;
//    return x_post;
//    };

//    Eigen::VectorXd getState() {
//           return x_post;
//    };

//    // 过程非线性向量函数
//    [[nodiscard]] Eigen::VectorXd f() const {
//          Eigen::VectorXd x = x_post;
//          x(0) += x(1) * t;
//          return x;
//    };

//    // 观测非线性向量函数
//    static Eigen::VectorXd h(const Eigen::VectorXd & x_pri) {
//           Eigen::VectorXd x(1);
//           double x1 = x_pri(0);
//           x(0) = x1;
//           return x;
//    };

//    //系统的动力学模型的雅可比矩阵
//    [[nodiscard]] Eigen::MatrixXd jacobian_f() const {
//    //        auto a = x_post;
//           Eigen::MatrixXd j_f(2, 2);
//           j_f << 1.0, t,
//                  0.0, 1.0;
//           return j_f;
//     };

//     //观测矩阵M*N 雅可比矩阵
//     static Eigen::MatrixXd jacobian_h() {
//     //        auto a = x_post;
//     Eigen::MatrixXd j_h(1, 2);
//     j_h << 1.0, 0.0;
//     return j_h;
//     };

//     // 过程噪声协方差矩阵
//     [[nodiscard]] Eigen::MatrixXd update_Q() const {
//         Eigen::MatrixXd q(2, 2);
//         //      angle                   v_angle
//         q   <<  pow(t, 4) / 4 * 5.5f,     pow(t, 3) / 2 * 3.5f,
//         pow(t, 3) / 2 * 3.5f,     pow(t, 2) * 1.5f;
//         //        cout << q << endl;
//         return q;
//     };

//     // 测量噪声协方差矩阵
//     static Eigen::MatrixXd update_R() {
//     Eigen::DiagonalMatrix<double, 1> r;
//     r.diagonal() << 0.001f;
//     return r;
//     };

//     // 时间差
//     double t=0;
//     Eigen::VectorXd ekf_prediction;


//private:
//    // f()的雅可比矩阵
//    // Eigen::MatrixXd jacobian_f;
//    Eigen::MatrixXd F;
//    // h()的雅可比矩阵
//    // Eigen::MatrixXd jacobian_h;
//    Eigen::MatrixXd H;
//    // 先验误差估计协方差矩阵
//    Eigen::MatrixXd P_pri;
//    // 后验误差估计协方差矩阵
//    Eigen::MatrixXd P_post;
//    // 卡尔曼增益
//    Eigen::MatrixXd K;
//    // 系统尺寸
//    int n{};
//    // N大小恒等式
//    Eigen::MatrixXd I;
//    // 先验状态
//    Eigen::VectorXd x_pri;
//    // 后验状态
//    Eigen::VectorXd x_post;
//    // 公式所用参数
//    // float a,w,angle_zero;

//};



#endif //EXTENDEDKALMAN_HPP
