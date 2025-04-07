#include "../../../../include/module/buff/PredictSolver/tracker.hpp"

AnglePredictor::AnglePredictor()
    :measurement(Eigen::VectorXd::Zero(3)),
     target_state(7),
     spd_state(Eigen::VectorXd::Zero(3))
{
//    auto f = [this](const Eigen::VectorXd& x){
//        Eigen::VectorXd x_new(3);
//        double dt = 0.1;
//        x_new << x(0) + (x(1)*sin(x(2)*t_) + (2.09-x(1)))*dt, x(1), x(2);
//        t_ += dt;
//        return x_new;
//    };

//    auto h = [this](const Eigen::VectorXd& x) { return Eigen::VectorXd::Constant(1, x(0)); };

//    auto jac_f = [this](const Eigen::VectorXd& x){
//        Eigen::MatrixXd F(3,3);
//        F << 1, sin(x(2)*t_)*0.1, x(1)*t_*cos(x(2)*t_)*0.1,
//             0, 1, 0,
//             0, 0, 1;
//        return F;
//    };

//    auto jac_h = [this](const Eigen::VectorXd& x){
//        Eigen::MatrixXd H(1,3);
//        H << 1, 0, 0;
//        return H;
//    };

//    ekf_ = ExtendedKalman(f,h,jac_f,jac_h,
//                          []{return Eigen::Matrix3d::Identity()*1e-4;},
//                          []{return Eigen::MatrixXd::Identity(1,1)*1e-4;},
//                          Eigen::Matrix3d::Identity()*0.1,Eigen::Vector3d(0,0.9,1.94));

//********************************************************************
//    // EKF
//    // 状态变量: x, y, theta, omega
//    // 测量变量: x, y, theta
//    // f - 过程函数
//    auto f = [this](const Eigen::VectorXd & x) {
//      Eigen::VectorXd x_new = x;
//      x_new(0) = x(0);              // x 保持不变
//      x_new(1) = x(1);              // y 保持不变
//      x_new(2) += x(3) * dt_;       // theta += omega * dt
//      x_new(3) = x(3);              // omega 保持不变
//      return x_new;
//    };

//    auto j_f = [this](const Eigen::VectorXd &) {
//      Eigen::MatrixXd f(4, 4);
//      // clang-format off
//      f << 1, 0, 0,    0,
//            0, 1, 0,    0,
//            0, 0, 1, dt_,
//            0, 0, 0,   1;
//      // clang-format on
//      return f;
//    };

//    // h - 观测函数
//    auto h = [](const Eigen::VectorXd & x) {
//      Eigen::VectorXd z(3);
//      z(0) = x(0); // x
//      z(1) = x(1); // y
//      z(2) = x(2); // theta
//      return z;
//    };

//    // J_h - 观测函数雅可比矩阵
//    auto j_h = [](const Eigen::VectorXd &) {
//      Eigen::MatrixXd h(3, 4);
//      // clang-format off
//      h << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0;
//      // clang-format on
//      return h;
//    };

//    s2qxyz_ = 1e-4;
//    s2qtheta_ = 1e-2;
//    auto u_q = [this]() {
//      Eigen::MatrixXd q(4, 4);
//      double t = dt_;
//      double pos_var = s2qxyz_ * t;         // 位置噪声方差
//      double theta_var = s2qtheta_ * t;     // 角度噪声方差
//      double omega_var = s2qtheta_ * t;     // 角速度噪声方差

//      // clang-format off
//      q << pos_var, 0,       0,         0,
//            0,       pos_var, 0,         0,
//            0,       0,       theta_var, 0,
//            0,       0,       0,         omega_var;
//      // clang-format on
//      return q;
//    };

//    // update_R - 测量噪声协方差矩阵
//    r_center_ =1e-8;
//    auto u_r = [this](const Eigen::VectorXd &) {
//      Eigen::DiagonalMatrix<double, 3> r;
//      r.diagonal() << r_center_, r_center_, r_center_; // 固定测量噪声
//      return r;
//    };


//    Eigen::DiagonalMatrix<double,4>p0;
//    p0.setIdentity();
//    ekf_ = ExtendedKalman(f, h, j_f, j_h, u_q, u_r, p0);

// *****************************************************************************

    auto f = [this](const Eigen::VectorXd& x){
          Eigen::VectorXd x_new = x;
          x_new(0) += x(2) * dt_;
          x_new(1) += x(3) * dt_;
          x_new(5) += x(6) * dt_;
          return x_new;
        };

    auto j_f = [this](const Eigen::VectorXd&){
          Eigen::MatrixXd f(7,7);
          f <<  1,   0,   dt_, 0,   0,   0,   0,
                0,   1,   0,   dt_, 0,   0,   0,
                0,   0,   1,   0,   0,   0,   0,
                0,   0,   0,   1,   0,   0,   0,
                0,   0,   0,   0,   1,   0,   0,
                0,   0,   0,   0,   0,   1,   dt_,
                0,   0,   0,   0,   0,   0,   1;

              // clang-format on
              return f;
        };

    auto h = [](const Eigen::VectorXd& x){
            Eigen::VectorXd z(3);
            double xc = x(0), yc = x(1), r = x(4), theta = x(5);
            double st = sin(theta), ct = cos(theta);
            double dn_1_2 = pow(xc * xc + yc * yc, -0.5);

            z(0) = xc + r * (st * yc * dn_1_2);   // xb
            z(1) = yc + r * (-st * xc * dn_1_2);  // yb
            z(2) = theta;                         // theta
            return z;
        };

    auto j_h = [](const Eigen::VectorXd& x){
          Eigen::MatrixXd h(3,7);
          double xc = x(0), yc = x(1), r = x(4), theta = x(5);
          double st = sin(theta), ct = cos(theta);
          double dn_1_2 = pow(xc * xc + yc * yc, -0.5);
          double dn_3_2 = pow(xc * xc + yc * yc, -1.5);

          // 各偏导数计算
          double dxb_dx = 1 - r * xc * yc * st * dn_3_2;
          double dxb_dy = r * xc * xc * st * dn_3_2;
          double dxb_dr = yc * st * dn_1_2;
          double dxb_dtheta = yc * ct * dn_1_2;

          double dyb_dx = -r * yc * yc * st * dn_3_2;
          double dyb_dy = 1 + r * xc * yc * st * dn_3_2;
          double dyb_dr = -xc * st * dn_1_2;
          double dyb_dtheta = -xc * ct * dn_1_2;

          // clang-format off
          h << dxb_dx, dxb_dy, 0, 0, dxb_dr, dxb_dtheta, 0,
               dyb_dx, dyb_dy, 0, 0, dyb_dr, dyb_dtheta, 0,
               0,      0,      0, 0, 0,      1,          0;
          // clang-format on
          return h;

        };

    s2qxyz_ = 1e-4;
        s2qtheta_ = 1e-2;
        s2qr_ = 80.0;

        auto u_q = [this](){
            Eigen::MatrixXd q(7, 7);
            double t = dt_;
            double pos_noise = s2qxyz_;
            double ang_noise = s2qtheta_;
            double r_noise = s2qr_;
            // 位置噪声项
            double q_x_x = pow(t,4)/4 * pos_noise;
            double q_x_vx = pow(t,3)/2 * pos_noise;
            double q_vx_vx = t*t * pos_noise;

            double q_y_y = pow(t,4)/4 * pos_noise;
            double q_y_vy = pow(t,3)/2 * pos_noise;
            double q_vy_vy = t*t * pos_noise;

            // 角度噪声项
            double q_theta_theta = pow(t,4)/4 * ang_noise;
            double q_theta_omega = pow(t,3)/2 * ang_noise;
            double q_omega_omega = t*t * ang_noise;

            // 半径噪声项
            double q_r_r = pow(t,4)/4 * r_noise;

            //
            q <<  q_x_x,  0,      q_x_vx,   0,      0,      0,            0,
                  0,      q_y_y,  0,        q_y_vy, 0,      0,            0,
                  q_x_vx, 0,      q_vx_vx,  0,      0,      0,            0,
                  0,      q_y_vy, 0,        q_vy_vy,0,      0,            0,
                  0,      0,      0,        0,      q_r_r,  0,            0,
                  0,      0,      0,        0,      0,      q_theta_theta,q_theta_omega,
                  0,      0,      0,        0,      0,      q_theta_omega,q_omega_omega;

                // clang-format on
                return q;

        };


            r_blade_ = 1e-8;
            r_center_ = 1e-8;

            auto u_r = [this](const Eigen::VectorXd& z) {
                Eigen::DiagonalMatrix<double, 3> r;
                r.diagonal() << r_blade_ * abs(z(0)),
                                r_blade_ * abs(z(1)),
                                r_center_ * abs(z(2));
                return r;
            };

            // 初始化协方差矩阵（7x7）
            Eigen::DiagonalMatrix<double, 7> p0;
            p0.setIdentity();
            ekf_ = ExtendedKalman(f, h, j_f, j_h, u_q, u_r, p0);


// ************************************************************************

    // a,w,c
//        Eigen::VectorXd params(3);
//        params << 0.9,1.94,0.0;
        auto u_fx = [](const Eigen::VectorXd& p,const std::vector<double>& ob){
            double a = p(0), w = p(1),c = p(2);
            double t = ob.at(0), theta = ob.at(1);
            //Eigen::VectorXd res(1);
            Eigen::MatrixXd fx(1,1);
            fx << theta-(a*sin(w*t+c)+(2.09-a));/*theta - (-a/w*cos(w*t) + (2.09-a)*t)*/;
            return fx;
        };

        auto u_J = [](const Eigen::VectorXd& p,const std::vector<double>& ob){
            double a = p(0), w = p(1),c = p(2);
            double t = ob.at(0);
            Eigen::MatrixXd J(1, 3);
            J << -sin(w*t+c)+1,-t*a*cos(w*t+c),-a*cos(w*t+c);
            return J;
        };

        min_a_ = 0.4;
        max_a_ = 1.3;
        min_w_ = 1.5;
        max_w_ = 2.3;

        auto constraint = [this](const Eigen::VectorXd& x){
            double a= x(0),w=x(1);
            bool a_con = a>min_a_&&a<max_a_;
            bool w_con = w > min_w_ && w < max_w_;
            return a_con && w_con;
        };

        max_iter_ = 50;
        min_step_ = 1e-10;
        obs_max_size_ = 150;
        gns=GaussNewtonSolver(u_fx, u_J, constraint,max_iter_,min_step_,obs_max_size_);


        // GNS EKF
        // state: a, w, c
        // measurement: a, w, c
        // f - Process function
        auto f_gns = [this](const Eigen::VectorXd & x) { return x; };
        // J_f - Jacobian of process function
        auto j_f_gns = [this](const Eigen::VectorXd &) {
          Eigen::MatrixXd f(3, 3);
          f.setIdentity();
          return f;
        };
        // h - Observation function
        auto h_gns = [](const Eigen::VectorXd & x) { return x; };
        // J_h - Jacobian of observation function
        auto j_h_gns = [this](const Eigen::VectorXd &) {
          Eigen::MatrixXd h(3, 3);
          h.setIdentity();
          return h;
        };

        s2q_a_ = 0.1;
        s2q_w_ = 0.1;
        s2q_c_ = 100.0;
        auto u_q_gns = [this]() {
          Eigen::MatrixXd q(3, 3);
          // clang-format off
          q <<  s2q_a_, 0,       0,
                0,       s2q_w_, 0,
                0,       0,       s2q_c_;
          // clang-format on
          return q;
        };

        r_a_ = 1e-8;
        r_w_ = 5e-4;
        r_c_ = 1e-8;
        auto u_r_gns = [this](const Eigen::VectorXd & z) {
          Eigen::DiagonalMatrix<double, 3> r;
          r.diagonal() << abs(r_a_ * z(0)), abs(r_w_ * z(1)), abs(r_c_ * z(2));
          return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 3> p0_gns;
        p0_gns.setIdentity();
        ekf_gns = ExtendedKalman(f_gns, h_gns, j_f_gns, j_h_gns, u_q_gns, u_r_gns, p0_gns);

}

//void AnglePredictor::update()
//{
//    Eigen::VectorXd ekf_prediction = ekf_.predict();
//    target_state = ekf_prediction;
//    measurement = Eigen::VectorXd::Zero(3);
//    measurement <<
//}

double AnglePredictor::normalize_angle_positive(double angle)
{
    double new_angle = std::fmod(angle,2*M_PI);
    if(new_angle<0){
        new_angle += 2*M_PI;
    }
    return new_angle;
}

double AnglePredictor::normalize_angle(double angle)
{
    double result = fmod(angle, 2*M_PI);
    if (result < 0)
        result += 2*M_PI;
    return result;
}

void AnglePredictor::calculateMeasurementFromPrediction(cv::Point2f& point,cv::Point2d& c_point,
                                                        double theta,const Eigen::VectorXd& state)
{
    double xc = state(0);
    double yc = state(1);
    double r = state(4);
    double theta_ = state(5);
    point.x = xc + r * (sin(theta) * yc * pow(pow(xc, 2) + pow(yc, 2), -0.5));
    point.y = yc + r * (-sin(theta) * xc * pow(pow(xc, 2) + pow(yc, 2), -0.5));
    c_point.x = xc;
    c_point.y = yc;
    theta_ = normalize_angle(theta_);
}

void AnglePredictor::init(cv::Point2f& point,double theta,double omega)
{
    //process_measurement(point,theta,omega);
    tracker_state = DETECTING;

    gns.obs.clear();
    solver_status = WAITING;

}

void AnglePredictor::process_measurement(cv::Point2d& point,double theta,double omega)
{
    //ekf_.predict();

//    Eigen::VectorXd z(1);
//    z << theta_meas;
    double x = point.x;
    double y = point.y;
    double r = 700/1000;
    double theta_ = theta;
    double omega_ = omega;
    target_state = Eigen::VectorXd::Zero(7);
    target_state << x,y,0,0,r,theta_,omega_;

    //std::cout<<233<<std::endl;
    //ekf_.update(target_state);

    ekf_.setInitState(target_state);

    gns.obs.clear();
    // /*filtered_obs_*/gns.obs.push_back({t_,ekf_.state()(0)});
}

void AnglePredictor::update(cv::Point2f& point,cv::Point2d& c_point,double theta,double omega)
{
//    double x = point.x;
//    double y = point.y;
//    double theta_ = theta;
//    double omega_ = omega;
//    target_state = Eigen::VectorXd::Zero(4);
//    target_state << x,y,theta_,omega_;
//    ekf_.setInitState(target_state);

    Eigen::VectorXd ekf_prediction = ekf_.predict();
    target_state = ekf_prediction;
    //calculateMeasurementFromPrediction(point,c_point,theta,ekf_prediction);

    measurement = Eigen::VectorXd::Zero(3);
    measurement << point.x,point.y,theta;

    // 不更新
    target_state = ekf_.update(measurement);

    target_state(4) = 700/1000;
    ekf_.setState(target_state);
}

void AnglePredictor::optimize_params()
{
    // a,w,c
//    Eigen::VectorXd params(3);
//    params << 0.91,1.94,0.0;
//    double a_start = 0.91;
//    double w_start = 1.94;
//    double c_start = 0.0;

    //——————————————————————————————————————————————

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
    x0 << a_, w_, c_;
    gns.setStartValue(x0);
    spd_state = Eigen::VectorXd::Zero(3);
    //auto status = gns.solve(filtered_obs_);

    ekf_gns.setInitState(x0);
//————————————————————————————————————————————

    double dt = 1.0f;
    double spd = abs(target_state(6));
    //std::cout<<122<<std::endl;
    gns.addObservation(dt,spd);

    //auto status = gns.solve();

    if(dt>min_first_solve_time)
    {
        int fail_count = 0;
        //std::cout<<23423522<<std::endl;
        while(gns.solve()!=GaussNewtonSolver::SUCCESS)
        {
            //std::cout<<232222<<std::endl;
            Eigen::VectorXd xn = gns.getState();
            xn(2)+=0.5;
            if(xn(2)>8){
                xn(2)-=8;
            }
            gns.setStartValue(xn);
            fail_count++;
            std::cout<<"fail_count: "<<fail_count<<std::endl;
            if(fail_count>20){               
                std::cout<<"fail to solve"<<std::endl;
                if(solver_status==VALID){
                   //std::cout<<"aaaaa"<<std::endl;
                   spd_state = ekf_gns.predict();
                   gns.obs.erase(gns.obs.end()-1);
                }else{
                    solver_status = INVALID;
                    gns.obs.erase(gns.obs.begin());

                }
                return;
            }
        }
        ekf_gns.predict();
        std::cout<<"\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\"<<std::endl;
        solver_status = VALID;
        auto gns_state = gns.getState();
        spd_state = ekf_gns.update(gns_state);
        spd_state(2) = normalize_angle_positive(spd_state(2));
        //if(status == GaussNewtonSolver::SUCCESS){
            a_ = spd_state(0);
            w_ = spd_state(1);
            c_ = spd_state(2);
        //}
    }
    else{
        solver_status = NOT_ENOUGH_OBS;
    }

//    if(status == GaussNewtonSolver::SUCCESS){
//        a_ = gns.getState()(0);
//        w_ = gns.getState()(0);
//        c_ = gns.getState()(0);
//    }

}


double AnglePredictor::predict(double t) const{
    std::cout<<"a:"<<a_<<std::endl;
    std::cout<<"w_:"<<w_<<std::endl;
    std::cout<<"c_:"<<w_<<std::endl;
    return -a_/w_*cos(w_*t+c_) + (2.09-a_)*t+c_;
}

//void AnglePredictor::solve(double time)
//{
//    if (tracker_state == DETECTING) {
//        solver_status = WAITING;
//        return;
//    }else if(tracker_state == TRACKING && solver_status == WAITING){
//        // first frame
//        solver_status = NOT_ENOUGH_OBS;
//        //obs_start_time = time;
//    }
//}

//void AnglePredictor::process_measurement(double timestamp,double theta_meas)
//{
//    Eigen::VectorXd z(3);
//    z << a_,w_,c_;
//    ekf_.predict();
//    ekf_.update(z);

//    if(last_timestamp_ > 0){
//        double dt = /*timestamp - last_timestamp_*/0.1;
//        filtered_obs_.push_back({dt,theta_meas});
//        if(filtered_obs_.size()>50) filtered_obs_.erase(filtered_obs_.begin());
//    }
//    last_timestamp_ =timestamp;

//    Eigen::VectorXd x0(3);
//    x0 << 0.912, 1.942, 0.0;  // 初始值
//    Eigen::VectorXd params=gns.solve(x0,filtered_obs_);

//    a_ = params[0];
//    w_ = params[1];
//    c_ = params[2];

//}

//void AnglePredictor::optimize_params()
//{

//    Eigen::VectorXd params(2);
//    params << 0.9,1.94;
//    auto residual = [](const Eigen::VectorXd& p,const std::vector<double>& ob){
//        double a = p(0), w = p(1);
//        double t = ob[0], theta = ob[1];
//        Eigen::VectorXd res(1);
//        res << theta - (-a/w*cos(w*t) + (2.09-a)*t);
//        return res;
//    };

//    auto jacobian = [](const Eigen::VectorXd& p,const std::vector<double>& ob){
//        double a = p(0), w = p(1);
//        double t = ob[0];
//        Eigen::MatrixXd J(1, 2);
//        J(0,0) = (cos(w*t)/w - (2.09-a)*t/a);
//        J(0,1) = (a/(w*w)*cos(w*t) + a*t/w*sin(w*t));
//        return J;
//    };

//    GaussNewtonSolver solver(residual, jacobian, 100, 1e-6);
//    auto status = solver.solve(params, filtered_obs_);

//    if(status == GaussNewtonSolver::SUCCESS){
//        a_ = params(0);
//        w_ = params(1);
//    }

//}






//    // GaussNewtonSolver
//      auto u_fx = [](const Eigen::VectorXd & x, const std::vector<double> & ob) {
//        double t = ob.at(0);
//        double y = ob.at(1);
//        double a = x(0), w = x(1), c = x(2);
//        Eigen::MatrixXd fx(1, 1);

//        fx << y - (a * sin(w * t + c) + (2.09 - a));
//        return fx;
//      };

//      auto u_J = [](const Eigen::VectorXd & x, const std::vector<double> & ob) {
//          double t = ob.at(0);
//          double a = x(0), w = x(1), c = x(2);
//          Eigen::MatrixXd J(1, 3);

//          // clang-format off
//          //   a                     w                         c
//          J << -sin(w * t + c) + 1,  -t * a * cos(w * t + c),  -a * cos(w * t + c);
//          // clang-format on
//          return J;
//        };

//      min_a_ = 0.4;
//      max_a_ = 1.3;
//      min_w_ = 1.5;
//      max_w_ = 2.3;
//      auto constraint = [this](const Eigen::VectorXd & x) {
//        double a = x(0), w = x(1);
//        bool a_con = a > min_a_ && a < max_a_;
//        bool w_con = w > min_w_ && w < max_w_;
//        return a_con && w_con;
//      };

//      max_iter_ = 50;
//      min_step_ = 1e-10;
//      obs_max_size_ = 150;
//      tracker_->gns=GaussNewtonSolver(u_fx, u_J, constraint, max_iter_, min_step_, obs_max_size_);

//      tracker_->a_start = 0.9125;
//      tracker_->w_start = 1.942;
//      tracker_->c_start = 0.0;
//      tracker_->min_first_solve_time = 2.0;

//      // GNS EKF
//        // state: a, w, c
//        // measurement: a, w, c
//        // f - Process function
//        auto f_gns = [this](const Eigen::VectorXd & x) { return x; };
//        // J_f - Jacobian of process function
//        auto j_f_gns = [this](const Eigen::VectorXd &) {
//          Eigen::MatrixXd f(3, 3);
//          f.setIdentity();
//          return f;
//        };
//        // h - Observation function
//        auto h_gns =[](const Eigen::VectorXd &x) {return x;};
//        // J_h - Jacobian of observation function
//          auto j_h_gns = [this](const Eigen::VectorXd &) {
//            Eigen::MatrixXd h(3, 3);
//            h.setIdentity();
//            return h;
//          };
//          // update_Q - process noise covariance matrix
//            s2q_a_ = 0.1;
//            s2q_w_ = 0.1;
//            s2q_c_ = 100.0;
//            auto u_q_gns = [this]() {
//              Eigen::MatrixXd q(3, 3);
//              // clang-format off
//              q <<  s2q_a_, 0,       0,
//                    0,       s2q_w_, 0,
//                    0,       0,       s2q_c_;
//              // clang-format on
//              return q;
//            };

//             // update_R - measurement noise covariance matrix
//            r_a_ = 1e-8;
//            r_w_ = 5e-4;
//            r_c_ = 1e-8;
//            auto u_r_gns = [this](const Eigen::VectorXd & z) {
//              Eigen::DiagonalMatrix<double, 3> r;
//              r.diagonal() << abs(r_a_ * z(0)), abs(r_w_ * z(1)), abs(r_c_ * z(2));
//              return r;
//            };

//            // P - error estimate covariance matrix
//            Eigen::DiagonalMatrix<double, 3> p0_gns;
//            p0_gns.setIdentity();
//            tracker_->ekf_gns = ExtendedKalman{f_gns, h_gns, j_f_gns, j_h_gns, u_q_gns, u_r_gns, p0_gns};

//}



//void BuffTracker::RunBuffTracker(){

//    Tp time=det_.GetSatatTime();
//    if(tracker_->tracker_state == Tracker::State::LOST){
//        tracker_->init();
//        tracking=false;
//    }else{
//        dt_ = (time-last_time_).count();
//        tracker_->lost_threshold=static_cast<int>(lost_time_threshold_/dt_);
//        tracker_->update();
//        tracker_->solve(time);
//    }

//    if(tracker_->tracker_state==Tracker::State::DETECTING)
//        tracking=false;
//    else if(tracker_->tracker_state==Tracker::State::TRACKING||
//            tracker_->tracker_state==Tracker::State::TEMP_LOST){
//        tracking = true;
//        const auto& state = tracker_->target_state;

//        Tracker::Target predict_target;
//        tracker_->Getparams(predict_target);
//        tracker_->getTrackerPosition(predict_target);

//        cv::Point2f position = predict_target.circle;
//        double vx = state(2);
//        double vy = state(3);
//        double r = state(4);
//        double theta = predict_target.theta;
//        double speed = state(6);
//        double a = 0.0;
//        double b = 0.0;
//        double c = 0.0;
//        double d = state(6);



//    }

//}





//Tracker::Tracker(double max_match_theta, double max_match_center_xoy)
//: tracker_state(LOST),
//  blade_id(0),
//  measurement(Eigen::VectorXd::Zero(4)),
//  target_state(Eigen::VectorXd::Zero(9)),
//  spd_state(Eigen::VectorXd::Zero(3)),
//  max_match_theta_(max_match_theta),
//  max_match_center_xoy_(max_match_center_xoy)
//{
//}

//double Tracker::normalize_angle(double angle)
//{
//    while(angle<0){
//        angle+=360.0;
//    }
//    angle = std::fmod(angle,360.0);
//    if(angle<0){
//        angle += 360.0;
//    }
//    return angle;
//}

//double Tracker::shortest_angular_distance(double angle1,double angle2)
//{
//    double diff = fmod(angle2 - angle1 +180.0,360.0) - 180.0;
//    return diff;
//}

//double Tracker::normalize_angle_positive(double angle)
//{
//    angle = std::fmod(angle,2*M_PI);
//    if(angle<0){
//        angle+=2*M_PI;
//    }
//    return angle;
//}

//void Tracker::Getparams(Target& deliver)
//{
//    deliver.point=det.GetBuffCenter();
//    deliver.theta=det.GetTheta();
//    deliver.circle=det.GetFitCenter();
//}


//void Tracker::init()
//{
//    last_theta_ = 0.0;

////    current.point=det.GetBuffCenter();
////    current.theta=det.GetTheta();
////    current.circle=det.GetFitCenter();

//    Getparams(current);
//    initEKF(current);
//    tracker_state = DETECTING;
//    detect_count_ = 0;

//    gns.obs.clear();
//    solver_status = WAITING;
//}

//void Tracker::update()
//{
//    Getparams(current);

//    Eigen::VectorXd ekf_prediction = ekf.predict();

//    bool is_detected = false;
//    target_state = ekf_prediction;
//    Target predict;
//    calculateMeasurementFromPrediction(predict,ekf_prediction);
//    double theta_diff = 0.0;
//    center_xoy_diff = 0.0;
//    theta_diff = shortest_angular_distance(predict.theta,current.theta);
////    double predAngle = std::atan2(predict.point.y - predict.circle.y,predict.point.x - predict.circle.x);
////    double currAngle = std::atan2(current.point.y - current.circle.y,current.point.x - current.circle.x);
////    double deltaTheta = abs(predAngle - currAngle);
//    center_xoy_diff = sqrt(
//                pow(current.point.x-predict.point.x,2)+
//                pow(current.point.y-predict.point.y,2));
//    if(center_xoy_diff<max_match_center_xoy_||tracker_state==DETECTING)
//    {
//        is_detected = true;
//        measurement = Eigen::VectorXd::Zero(3);
//        bool need_update = true;
//        if(abs(theta_diff)>max_match_theta_){
//            if(!handleBladeJump(theta_diff)){
//                measurement<<predict.point.x,predict.point.y,predict.theta;
//                is_detected = false;
//                need_update = false;
//            }else{
//                measurement<<current.point.x,current.point.y,current.theta;
//            }
//        }else{
//            measurement<<current.point.x,current.point.y,current.theta;
//        }
//        measurement(3) = last_theta_ + shortest_angular_distance(last_theta_,measurement(3));
//        if(need_update){
//            last_theta_ = measurement(3);
//            target_state = ekf.update(measurement);
//        }
//    }else{
//        detect_count_ = detect_count_?detect_count_ -1:0;
//        lost_count_++;
//    }
//   target_state(4) = BLADE_R_OFFSET/1000;
//   ekf.setState(target_state);

//   if(tracker_state == DETECTING){
//       if(is_detected){
//           detect_count_++;
//           if(detect_count_>tracking_threshold){
//               detect_count_ = 0;
//               tracker_state = TRACKING;
//           }
//       }else{
//           detect_count_=0;
//           tracker_state =LOST;
//       }
//   }else if(tracker_state == TRACKING){
//       if(!is_detected){
//           lost_count_++;
//           if(lost_count_>lost_threshold){
//               lost_count_ = 0;
//               tracker_state=LOST;
//           }
//       }else{
//           lost_count_=0;
//           tracker_state=TRACKING;
//       }
//   }
//}

//void Tracker::solve(const Tp& time)
//{
//  obs_start_time = det.GetSatatTime();
//  if(tracker_state ==DETECTING){
//      solver_status = WAITING;
//      return;
//  }else if(tracker_state==TRACKING&&solver_status==WAITING){
//      solver_status=NOT_ENOUGH_OBS;
//      obs_start_time = time;
//      Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
//      x0<<a_start,w_start,c_start;
//      gns.setStartValue(x0);
//      spd_state = Eigen::VectorXd::Zero(3);

//      ekf_gns.setInitState(x0);
//  }else if(tracker_state ==TRACKING||tracker_state==TEMP_LOST){
//      double dt = (time-obs_start_time).count();
//      double spd = abs(target_state(6));

//      gns.addObservation(dt,spd);

//      if(dt>min_first_solve_time){
//          int fail_count=0;
//          while(gns.solve()!=GaussNewtonSolver::SUCCESS){
//              Eigen::VectorXd xn = gns.getState();
//              xn(2)+=0.5;
//              if(xn(2)>8){
//                  xn(2)-=8;
//              }
//              gns.setStartValue(xn);
//              fail_count++;
//              if(fail_count>20){
//                  if(solver_status==VALID){
//                      spd_state=ekf_gns.predict();
//                      gns.obs.erase(gns.obs.end()-1);
//                  }else{
//                      solver_status = INVALID;
//                      gns.obs.erase(gns.obs.begin());
//                      lost_count_++;
//                      tracker_state = TEMP_LOST;
//                  }
//                  return;
//              }
//          }
//          ekf_gns.predict();
//          solver_status=VALID;
//          auto gns_state=gns.getState();
//          spd_state(2)=normalize_angle_positive(spd_state(2));
//      }else{
//          solver_status = NOT_ENOUGH_OBS;
//      }
//  }
//  if(lost_count_>lost_threshold){
//      lost_count_=0;
//      tracker_state=LOST;
//  }
//}

//void Tracker::initEKF(Target& obtain)
//{
//    Getparams(obtain);
//    double xc = obtain.point.x;
//    double yc = obtain.point.y;
//    double r = BLADE_R_OFFSET/1000;
//    double theta = obtain.theta;
//    double omega = OMEGA;
//    target_state = Eigen::VectorXd::Zero(7);
//    target_state << xc,yc,0,0,r,theta,omega;
//    ekf.setInitState(target_state);
//}


//bool Tracker::handleBladeJump(double theta_diff)
//{
//    if(theta_diff<1.24)
//    {
//        return true;
//    }
//    return false;

//}

//void Tracker::calculateMeasurementFromPrediction(Target &target, const Eigen::VectorXd &state)
//{
//  Getparams(target);
//  double xc = state(0);
//  double yc = state(1);
//  double r = state(4);
//  double theta = state(5);

//  target.point.x = xc + r * (sin(theta) * yc * pow(pow(xc,2)+pow(yc,2),-0.5));
//  target.point.y = yc + r * (-sin(theta) * xc * pow(pow(xc,2)+pow(yc,2),-0.5));

//  target.circle.x = xc;
//  target.circle.y = yc;
//  target.theta = normalize_angle(theta);

//}

//void Tracker::getTrackerPosition(Target& target)
//{
//    //Getparams(target);
//    calculateMeasurementFromPrediction(target,target_state);
//    target.theta = normalize_angle(target.theta);
//}



