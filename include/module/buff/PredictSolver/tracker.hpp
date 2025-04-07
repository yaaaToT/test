#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <Eigen/Eigen>
#include <memory>
#include <string>

//#include "../Detector/Detector.hpp"
#include "ExtendedKalman.hpp"
#include "GaussNewtonSolver.hpp"

//#define PI 3.1415926
#define BLADE_R_OFFSET 700.0
// #define OMEGA 1.0 / 3 * PI
#define OMEGA 0.0

//typedef std::chrono::steady_clock::time_point TimePoint;

typedef std::chrono::high_resolution_clock::time_point Tp;


class AnglePredictor
{
public:
    AnglePredictor();

    double normalize_angle_positive(double angle);

    double normalize_angle(double angle);

    void calculateMeasurementFromPrediction(cv::Point2f& point,cv::Point2d& c_point,double theta,const Eigen::VectorXd& state);

    void process_measurement(cv::Point2d& point,double theta,double omega);

    //void process_measurement(double timestamp,double theta_meas);

    //void update();
    void init(cv::Point2f& point,double theta,double omega);

    void update(cv::Point2f& point,cv::Point2d& c_point,double theta,double omega);

    void optimize_params();
    double predict(double t)const;
    void solve(double time);
private:
    ExtendedKalman ekf_;
    GaussNewtonSolver gns;
    ExtendedKalman ekf_gns;
    std::vector<std::vector<double>> filtered_obs_;
    //double last_timestamp_ = -1;
    double a_ = 0.91;
    double w_ = 1.94;
    double c_ = 0.0;
    double t_ =0.01;
    double min_first_solve_time=0.0;

    double dt_=0.01;
    double s2qxyz_;
    double s2qtheta_;
    double s2qr_;
    double r_blade_;
    double r_center_;

    int max_iter_;
    double min_step_;
      int obs_max_size_;
      double max_a_;
      double min_a_;
      double max_w_;
      double min_w_;

      double s2q_a_;
      double s2q_w_;
      double s2q_c_;
      double r_a_;
      double r_w_;
      double r_c_;

      Eigen::VectorXd measurement;
      Eigen::VectorXd target_state;
      Eigen::VectorXd spd_state;

      enum State {
          LOST,
          DETECTING,
          TRACKING,
          TIMEOUT,
          TEMP_LOST,
        } tracker_state;

      enum SolverStatus {
        WAITING,
        NOT_ENOUGH_OBS,
        VALID,
        INVALID,
      } solver_status;

};



/*class Tracker
{
public:
  Tracker(double max_match_theta,double max_match_center_xoy);

  double normalize_angle(double angle);

  double shortest_angular_distance(double a1,double a2);

  double normalize_angle_positive(double angle);

  //void Getparams();

  void init();

  void update();

  void solve(const Tp& time);

  ExtendedKalman ekf;

  GaussNewtonSolver gns;

  ExtendedKalman ekf_gns;

  Detector det;

  double a_start,w_start,c_start;
  double min_first_solve_time;

  int tracking_threshold;
  int lost_threshold;
  int timeout_threshold;

  double blade_z_ground;
  double robot_z_ground;
  double distance;
  double max_distance_diff;
  double center_xoy_diff;

  enum State{
      LOST,
      DETECTING,
      TRACKING,
      TIMEOUT,
      TEMP_LOST,
  }tracker_state;

  enum SolverStatus {
    WAITING,
    NOT_ENOUGH_OBS,
    VALID,
    INVALID,
  }solver_status;

  int blade_id;

  struct Target
  {
      cv::Point2f point;
      cv::Point2f circle;
      double theta;
  };
  Target current;
  void getTrackerPosition(Target& target);
  void Getparams(Target& deliver);

  Eigen::VectorXd measurement;

  Eigen::VectorXd target_state;

  Eigen::VectorXd spd_state;

  Tp obs_start_time; // 使用double类型时间戳


private:
//  void initEKF();

//  bool handleBladeJump(double theta_diff);

//  //cv::Point3d rotateBlade(const blade_transform blade, int idx);

//  void calculateMeasurementFromPrediction(const Eigen::VectorXd&state);
  void initEKF(Target& obtain);
  bool handleBladeJump(double theta_diff);
  void calculateMeasurementFromPrediction(Target& target,const Eigen::VectorXd & state);

  double max_match_theta_;
  double max_match_center_xoy_;

  int detect_count_;
  int lost_count_;
  int timeout_count_;  // ms

  double last_theta_;
};



class BuffTracker
{
public:
    explicit BuffTracker();
private:
    void RunBuffTracker();
    bool tracking;
    double dt_;
    Detector det_;
    Tp start_time;
    Tp last_time_;
    double lost_time_threshold_;
    std::unique_ptr<Tracker>tracker_;
    // param
    double s2qxyz_;
    double s2qtheta_;
    double s2qr_;
    double r_blade_;
    double r_center_;

    int max_iter_;
    double min_step_;
    int obs_max_size_;
    double max_a_;
    double min_a_;
    double max_w_;
    double min_w_;

    double s2q_a_;
    double s2q_w_;
    double s2q_c_;
    double r_a_;
    double r_w_;
    double r_c_;

    std::string task_mode_;
    void taskCallback(const std::string& task_msg);

    std::string target_frame_;

};
*/










#endif //TRACKER_HPP
