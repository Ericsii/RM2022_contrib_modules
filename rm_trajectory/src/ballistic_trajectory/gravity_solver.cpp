#include "rm_trajectory/gravity_solver.hpp"

const double Gravity = 9.7913;

namespace rm_trajectory
{
	GravitySolver::GravitySolver(double initial_vel, double friction_coeff)
		: initial_vel_(initial_vel), friction_coeff_(friction_coeff)
	{

		auto forward_motion = [&](double given_angle, double &h, double &t)
		{
			h = initial_vel_ * sin(given_angle * DEC) * t - 0.5 * Gravity * t * t;
		};

		auto forward_time = [&](double given_angle, double given_distance, double &t)
		{
			double v = initial_vel_ * cos(given_angle * DEC) / (friction_coeff_ * initial_vel_ * cos(given_angle * DEC) * t + 1);
			t = t - (log(friction_coeff_ * initial_vel_ * cos(given_angle * DEC) * t + 1) / friction_coeff_ - given_distance) / v;
		};

		iterative_tool_ = std::make_shared<IterativeTrajectoryTool>();
		iterative_tool_->set_forward_motion(forward_motion);
		iterative_tool_->set_forward_time(forward_time);
		iterative_tool_->set_is_database(false);
	}

	bool GravitySolver::solve(double target_distance, double target_h, double &angle)
	{
		return iterative_tool_->solve(target_distance, target_h, angle);
	}

	std::string GravitySolver::error_message()
	{
		return iterative_tool_->error_message();
	}

}