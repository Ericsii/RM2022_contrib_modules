#include "rm_trajectory/trajectory_tool.hpp"

namespace rm_trajectory
{

bool TrajectoryTool::solve(double target_x, double target_h, double & angle)
{
  double aimed_h, h;
  double dh = 0;
  double tmp_angle = 0;
  double t = 0;
  aimed_h = target_h;
  for (int i = 0; i < max_iter_; i++) {
    tmp_angle = atan2(aimed_h, target_x);
    if (tmp_angle > 80 * M_PI / 180 || tmp_angle < -80 * M_PI / 180) {
      error_message_ = "iterative angle is out of range(-80d,80d)";
      return false;
    }
    forward_motion_func_(tmp_angle, target_x, h, t);
    if (t > 10) {
      error_message_ = "motion time(" + std::to_string(t) + ") is too long";
      return false;
    }
    dh = target_h - h;
    aimed_h = aimed_h + dh;
    if (fabs(dh) < 0.001) {
      break;
    }
  }
  if (fabs(dh) > 0.01) {
    error_message_ = "height error(" + std::to_string(dh) + ") is too large";
    return false;
  }
  angle = tmp_angle;
  return true;
}

}