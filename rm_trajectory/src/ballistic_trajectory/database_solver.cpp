#include "rm_trajectory/database_solver.hpp"

namespace rm_trajectory
{

	double DatabaseSolver::pitch_1(double target_h, double bullet_v) {

		double pitch = 0;

		if (bullet_v == 30) {
			pitch = 0.05985 * target_h + 2.794;
		}

		if (bullet_v == 18) {
			pitch = 0.04518 * target_h + 3.257;
		}

		return pitch;
	}
	double DatabaseSolver::pitch_3(double target_h, double bullet_v) {

		double pitch = 0;

		if (bullet_v == 30){
			if (target_h > 0) {
				pitch = 0.01736 * target_h + 2.975;
			}
			else {
				pitch = 0.01736 * (target_h - 70) + 2.975;
			}
		}

		if (bullet_v == 18) {
			pitch = 0.01759 * target_h + 4.546;
		}

		return pitch;
	}
	double DatabaseSolver::pitch_5(double target_h, double bullet_v) {

		double pitch = 0;

		if (bullet_v == 30) {
			pitch = 0.01055 * target_h + 4.292;
		}

		if (bullet_v == 18) {
			pitch = 0.01059 * target_h + 6.482;
		}

		return pitch;
	}
	double DatabaseSolver::pitch_7(double target_h, double bullet_v) {

		double pitch = 0;

		if (bullet_v == 30) {
			pitch = 0.007561 * target_h + 5.16;
		}

		if (bullet_v == 18) {
			pitch = 0.007972 * target_h + 7.762;
		}

		return pitch;
	}

	double DatabaseSolver::get_pitch(double target_distance, double target_h, double bullet_v) {

		bullet_v = initial_vel_;

		target_distance = target_distance / 1000;

		double pitch;
		double f_1 = pitch_1(target_h, bullet_v);
		double f_3 = pitch_3(target_h, bullet_v);
		double f_5 = pitch_5(target_h, bullet_v);
		double f_7 = pitch_7(target_h, bullet_v);

		pitch = f_1 + (f_3 - f_1) / 2 * (target_distance - 1)
			+ ((f_5 - f_3) / 2 - (f_3 - f_1) / 2) / 4 * (target_distance - 1) * (target_distance - 3)
			+ (((f_7 - f_5) / 2 - (f_5 - f_3) / 2) / 4 - ((f_5 - f_3) / 2 - (f_3 - f_1) / 2) / 4) / 6 *
			(target_distance - 1) * (target_distance - 3) * (target_distance - 5);

		return pitch;

	}

	DatabaseSolver::DatabaseSolver(double initial_vel)
		: initial_vel_(initial_vel) {

		auto forward_motion = [&](double target_distance, double target_h, double & angle) {

			double bullet_v = initial_vel_;

			double pitch;
			double f_1 = pitch_1(target_h, bullet_v);
			double f_3 = pitch_3(target_h, bullet_v);
			double f_5 = pitch_5(target_h, bullet_v);
			double f_7 = pitch_7(target_h, bullet_v);

			pitch = f_1 + (f_3 - f_1) / 2 * (target_distance - 1)
				+ ((f_5 - f_3) / 2 - (f_3 - f_1) / 2) / 4 * (target_distance - 1) * (target_distance - 3)
				+ (((f_7 - f_5) / 2 - (f_5 - f_3) / 2) / 4 - ((f_5 - f_3) / 2 - (f_3 - f_1) / 2) / 4) / 6 *
				(target_distance - 1) * (target_distance - 3) * (target_distance - 5);
			angle = pitch;
		};

		iterative_tool_ = std::make_shared<IterativeTrajectoryTool>();
		iterative_tool_->set_forward_motion(forward_motion);
		iterative_tool_->set_is_database(true);
	}

	bool DatabaseSolver::solve(double target_distance, double target_h, double & angle) {
		return iterative_tool_->solve(target_distance, target_h, angle);
	}

	std::string DatabaseSolver::error_message() {
		return iterative_tool_->error_message();
	}

}