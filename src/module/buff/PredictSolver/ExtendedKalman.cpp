#include "../../../../include/module/buff/PredictSolver/ExtendedKalman.hpp"

// Copyright 2022 Chen Jun
ExtendedKalman::ExtendedKalman(
    const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
    const VoidMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0)
    : f(f),
    h(h),
    jacobian_f(j_f),
    jacobian_h(j_h),
    update_Q(u_q),
    update_R(u_r),
    P_post(P0),
    n(P0.rows()),
    I(Eigen::MatrixXd::Identity(n, n)),
    x_pri(n),
    x_post(n)
{}

void ExtendedKalman::setState(const Eigen::VectorXd & x0) { x_post = x0; }

void ExtendedKalman::setInitState(const Eigen::VectorXd &x0)
{
    x_post = x0;
    auto n = x0.rows();
    Eigen::DiagonalMatrix<double,Eigen::Dynamic>p0(n);
    p0.setIdentity();
    P_post = p0;
    P_pri = p0;
    x_pri = x0;
}

Eigen::MatrixXd ExtendedKalman::predict()
{
    F = jacobian_f(x_post), Q = update_Q();

    x_pri = f(x_post);
    P_pri = F * P_post * F.transpose() + Q;

    // 处理在下一次预测之前没有测量的情况
    x_post = x_pri;
    P_post = P_pri;

    return x_pri;
}

Eigen::MatrixXd ExtendedKalman::update(const Eigen::VectorXd & z)
{
    H = jacobian_h(x_pri), R = update_R(z);

    K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
    x_post = x_pri + K * (z - h(x_pri));
    P_post = (I - K * H) * P_pri;

    return x_post;
}



//void ExtendedKalman::setState(const Eigen::VectorXd& x0){
//    x_post_ = x0;
//}

//Eigen::VectorXd ExtendedKalman::predict()
//{
//    F_ = jacobian_f_(x_post_);
//    Q_ = update_Q_();
//    x_pri_ = f_(x_post_);
//    P_pri_ = F_ * P_post_ * F_.transpose() + Q_;
//    x_post_ = x_pri_;
//    P_post_ = P_pri_;
//    return x_pri_;
//}

//Eigen::VectorXd ExtendedKalman::update(const Eigen::VectorXd& z)
//{
//    H_ = jacobian_h_(x_pri_);
//    R_ = update_R_(z);
//    K_ = P_pri_ * H_.transpose() * (H_ * P_pri_ * H_.transpose() + R_).inverse();
//    x_post_ = x_pri_ + K_ * (z - h_(x_pri_));
//    P_post_ = (I_ - K_ * H_) * P_pri_;
//    return x_post_;
//}



//void ExtendedKalman::predict()
//{
//    F_ = jac_f_(x_post_);
//    x_pri_ = f_(x_post_);
//    P_pri_ = F_ * P_post_ * F_.transpose() + Q_();
//    x_post_ = x_pri_;
//    P_post_ = P_pri_;
//}

//void ExtendedKalman::update(const Eigen::VectorXd& z)
//{
//    //std::cout<<1<<std::endl;
//    H_ = jac_h_(x_pri_)/*,R=R_(z)*/;
//    //std::cout<<2<<std::endl;
//    K_ = P_pri_ * H_.transpose() * (H_ * P_pri_ * H_.transpose() + R_()).inverse();
//    //std::cout<<3<<std::endl;
//    x_post_ = x_pri_ + K_ * (z - h_(x_pri_));
//    //std::cout<<4<<std::endl;
//    P_post_ = (Eigen::Matrix3d::Identity() - K_ * H_) * P_pri_;
//    //std::cout<<5<<std::endl;
//}

//Eigen::VectorXd ExtendedKalman::state() const{
//    return x_post_;
//}



