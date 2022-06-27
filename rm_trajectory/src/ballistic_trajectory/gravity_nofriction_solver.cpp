#include "rm_trajectory/gravity_nofriction_solver.hpp"

const double Gravity = 9.7913;

namespace rm_trajectory
{
	GravityNofrictionSolver::GravityNofrictionSolver(double initial_vel)
		: initial_vel_(initial_vel) {

		//比例补偿迭代求解h，公式为简单抛物线：h = v * t - 0.5 * g * t^2
		auto forward_motion = [&](double given_angle, double & h, double & t) {

			h = initial_vel_ * sin(given_angle * DEC) * t - 0.5 * Gravity * t * t;		

		};
		//无空气阻力的飞行时间：T = distance / v
		auto forward_time = [&](double given_angle, double given_distance, double & t) {
            t = given_distance / (initial_vel * cos(given_angle * DEC));
		};
	
		iterative_tool_ = std::make_shared<IterativeTrajectoryTool>();
		iterative_tool_->set_forward_motion(forward_motion);
		iterative_tool_->set_forward_time(forward_time);
		iterative_tool_->set_is_database(false);
	}

	bool GravityNofrictionSolver::solve(double target_distance, double target_h, double & angle) {
		return iterative_tool_->solve(target_distance, target_h, angle);
	}

	std::string GravityNofrictionSolver::error_message() {
		return iterative_tool_->error_message();
	}

}