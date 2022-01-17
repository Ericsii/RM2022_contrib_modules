#include "rm_trajectory/trajectory_gimbal.hpp"

namespace rm_trajectory
{

bool TrajectoryGimbal::solve(double x, double y, double z, double & pitch, double & yaw)
{
  if (!solver_) {
    pitch = -atan2(z, x);
  } else {
    double angle;
    if (solver_->solve(z, x, angle)) {
      pitch = -angle;
    } else {
      error_message_ = solver_->error_message();
      return false;
    }
  }
  yaw = atan2(y, x);
  return true;
}

}