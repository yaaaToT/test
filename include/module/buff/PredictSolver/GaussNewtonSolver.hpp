#ifndef GAUSSNEWTONSOLVER_HPP
#define GAUSSNEWTONSOLVER_HPP

#include <iostream>
#include <chrono>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/SVD>
#include <functional>
#include <vector>

//class GaussNewtonSolver
//{
//public:
//  GaussNewtonSolver() = default;
//  using VecMatFunc =
//    std::function<Eigen::MatrixXd(const Eigen::VectorXd &, const std::vector<double> &)>;
//  using BoolMatFunc = std::function<bool(const Eigen::VectorXd &)>;

//  enum SolverStatus { SUCCESS, NO_CONVERGENCE, INVALID_START_VALUE }; // 成功 不收敛 无效值

//  explicit GaussNewtonSolver(
//    const VecMatFunc & u_fx, const VecMatFunc & u_J, const BoolMatFunc & constraint, int max_iter,
//    double min_step, int obs_max_size);

//  void addObservation(double x, double y);

//  void setStartValue(const Eigen::VectorXd & x);

//  SolverStatus solve();

//  Eigen::VectorXd getState();

//  std::vector<std::vector<double>> obs;

//private:
//  Eigen::VectorXd x_;

//  VecMatFunc u_fx_;
//  Eigen::MatrixXd fx_;

//  VecMatFunc u_J_;
//  Eigen::MatrixXd J_;

//  Eigen::MatrixXd H_;
//  Eigen::VectorXd B_;
//  Eigen::VectorXd delta_x_;

//  BoolMatFunc constraint_;

//  int max_iter_;
//  double min_step_;
//  int obs_max_size_;

//};  // class GaussNewtonSolver



class GaussNewtonSolver
{
public:
    GaussNewtonSolver() = default;
    //using JacFunc  = std::function<Eigen::VectorXd(const Eigen::VectorXd&, const std::vector<double>&)>;
    using VecMatFunc= std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const std::vector<double>&)>;
    using BoolMatFunc = std::function<bool(const Eigen::VectorXd &)>;

    enum SolverStatus { SUCCESS, NO_CONVERGENCE, INVALID_START_VALUE };

    GaussNewtonSolver(VecMatFunc u_fx,VecMatFunc u_J, BoolMatFunc constraint,int max_iter,double min_step, int obs_max_size/*double tol*/)
        :u_fx_(u_fx), u_J_(u_J),  constraint_(constraint),max_iter_(max_iter),min_step_(min_step),obs_max_size_(obs_max_size) /*tol_(tol)*/ {}

    void addObservation(double x, double y);

    SolverStatus solve(/*Eigen::VectorXd& params,*/ /*const std::vector<std::vector<double>>& obs*/);

    void setStartValue(const Eigen::VectorXd& x);

    Eigen::VectorXd getState();

    std::vector<std::vector<double>>obs;

private:
    Eigen::VectorXd x_;

    VecMatFunc u_fx_;
    Eigen::MatrixXd fx_;
    VecMatFunc u_J_;
    Eigen::MatrixXd J_;
    BoolMatFunc constraint_;

    Eigen::MatrixXd H_;
    Eigen::VectorXd B_;
    Eigen::VectorXd delta_x_;

    double min_step_;
    int max_iter_;
    int obs_max_size_;
};


//class GaussNewtonSolver
//{
//public:
//    GaussNewtonSolver()=default;
//    using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const std::vector<double>&)>;
//    using BoolMatFunc = std::function<bool(const Eigen::VectorXd&)>;

//    enum SolverStatus { SUCCESS, NO_CONVERGENCE, INVALID_START_VALUE };

//    GaussNewtonSolver(const VecMatFunc& u_fx, const VecMatFunc& u_J,
//                      const BoolMatFunc& constraint, int max_iter, double min_step/*,int obs_max_size*/)
//        :u_fx_(u_fx), u_J_(u_J), constraint_(constraint),max_iter_(max_iter),min_step_(min_step)/*,obs_max_size_(obs_max_size)*/{}

//    void addObservation(double x, double y);

//    void setStartValue(const Eigen::VectorXd & x);

////    SolverStatus solve();

////    Eigen::VectorXd getState();

////    std::vector<std::vector<double>> obs;

//    Eigen::VectorXd solve(const Eigen::VectorXd& x0, const std::vector<std::vector<double>>& obs);

//private:
////    Eigen::VectorXd x_;

////      VecMatFunc u_fx_;
////      Eigen::MatrixXd fx_;

////      VecMatFunc u_J_;
////      Eigen::MatrixXd J_;

////      Eigen::MatrixXd H_;
////      Eigen::VectorXd B_;
////      Eigen::VectorXd delta_x_;

////      BoolMatFunc constraint_;

////      int max_iter_;
////      double min_step_;
////      int obs_max_size_;


//    VecMatFunc u_fx_, u_J_;
//    BoolMatFunc constraint_;
//    int max_iter_;
//    double min_step_;
//    Eigen::VectorXd x_;
//};


#endif //GAUSSNEWTONSOLVER_HPP
