#include "rm_trajectory/trajectory_gravity.hpp"

#include <cmath>
#include <string>
#include <memory>

const double GRAVITY = 9.7913;

namespace rm_trajectory 
{

TrajectoryGravity::TrajectoryGravity(double initial_vel)
: initial_vel_(initial_vel) {
  auto forward_motion = [&](double given_angle, double given_x, double & h, double & t) {
      t = given_x / (initial_vel_ * cos(given_angle));
      h = initial_vel_ * sin(given_angle) * t - GRAVITY * t * t / 2;
    };
  iterative_tool_ = std::make_shared<TrajectoryTool>();
  iterative_tool_->set_forward_motion(forward_motion);
  iterative_tool_->set_max_iter(20);
}

bool TrajectoryGravity::solve(double target_x, double target_h, double &angle) {
    return iterative_tool_->solve(target_x, target_h, angle);
}

std::string TrajectoryGravity::error_message() {
    return iterative_tool_->error_message();
}

}