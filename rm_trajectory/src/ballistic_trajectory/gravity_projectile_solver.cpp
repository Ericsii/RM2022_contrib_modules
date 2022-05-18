#include "rm_trajectory/gravity_projectile_solver.hpp"

#include <cmath>
#include <string>
#include <memory>

const double GRAVITY = 9.7913;

namespace rm_trajectory
{
    GravityProjectileSolver::GravityProjectileSolver(double initial_vel)
        : initial_vel_(initial_vel)
    {
        auto forward_motion = [&](double given_angle, double &h, double &t)
        {
            h = initial_vel_ * sin(given_angle) * t - GRAVITY * t * t / 2;
        };

        auto forward_time = [&](double given_angle, double given_distance, double &t) 
        {
            t = given_distance / (initial_vel * cos(given_angle));
        };

        iterative_tool_ = std::make_shared<IterativeTrajectoryTool>();
        iterative_tool_->set_forward_motion(forward_motion);
        iterative_tool_->set_forward_time(forward_time);
        iterative_tool_->set_is_database(false);
    }

    bool GravityProjectileSolver::solve(double target_x, double target_h, double &angle)
    {
        return iterative_tool_->solve(target_x, target_h, angle);
    }

    std::string GravityProjectileSolver::error_message()
    {
        return iterative_tool_->error_message();
    }
} // namespace rm_trajectory
