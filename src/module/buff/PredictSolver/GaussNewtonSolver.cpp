#include "../../../../include/module/buff/PredictSolver/GaussNewtonSolver.hpp"

void GaussNewtonSolver::addObservation(double x, double y)
{
  std::vector<double> ob;
  ob.push_back(x);
  ob.push_back(y);
  obs.push_back(ob);
  if (int(obs.size()) > obs_max_size_) {
    obs.erase(obs.begin());
  }
}

void GaussNewtonSolver::setStartValue(const Eigen::VectorXd &x) {x_=x;}

GaussNewtonSolver::SolverStatus GaussNewtonSolver::solve(/*Eigen::VectorXd& params,*/ /*const std::vector<std::vector<double>>& obs*/)
{
    //std::cout<<"232536"<<std::endl;
//    fx_.resize(obs.size(),1);
//    J_.resize(obs.size(),x_.size());
     J_.resize(obs.size(),x_.size());
    //std::cout<<"6586"<<std::endl;
     fx_.resize(obs.size(),1);
    //std::cout<<"23874"<<std::endl;
    for(int iter = 0; iter < max_iter_; iter++)
    {
        std::cout<<"obs.size: "<<obs.size()<<std::endl;
        for(int i = 0;i < int(obs.size()); i++)
        {
            //std::cout<<"2395"<<std::endl;
            fx_(i,0) = u_fx_(x_,obs.at(i))(0,0);
            //哦std::cout<<"asfjs"<<std::endl;
            for(int k = 0; k<int(x_.size());k++)
            J_(i,k)=u_J_(x_,obs.at(i))(0,k);
        }

        /*Eigen::MatrixXd H*/ H_= J_.transpose() * J_;
        /*Eigen::VectorXd g*/B_ = J_.transpose() * fx_;

        /*Eigen::VectorXd dx*/delta_x_ = H_.ldlt().solve(-B_);
        /*params*/x_ += /*dx*/delta_x_;

        if(!constraint_(/*params*/x_))
        {
            std::cout<<"status: invalid_start_value"<<std::endl;
            return SolverStatus::INVALID_START_VALUE;
        }

        if(delta_x_.norm() < min_step_)
        {
            std::cout<<"status: success"<<std::endl;
            return SolverStatus::SUCCESS;
        }
    }
    std::cout<<"no_convergence"<<std::endl;
    return SolverStatus::NO_CONVERGENCE;
}

Eigen::VectorXd GaussNewtonSolver::getState() {return x_;}
