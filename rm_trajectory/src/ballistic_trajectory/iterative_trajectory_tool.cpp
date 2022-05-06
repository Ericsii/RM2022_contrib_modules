#include "rm_trajectory/iterative_trajectory_tool.hpp"

namespace rm_trajectory
{
	bool IterativeTrajectoryTool::solve(double target_distance, double target_h, double & angle) {

		double pitch = 0;
		target_distance = target_distance / 1000;

		if (database_flag == 1) {
			forward_motion_func_(target_distance, target_h, pitch);
			angle = pitch;
			return true;
		}
		else {
			double h = 0, error_h = 0;
			double t = 0, t_lk = 0;
			double K = 2;

			for (int i = 0; i < max_iter_; i++) {
				for (int j = 0; j < max_iter_; j++) {
					forward_time_func_(pitch, target_distance, t);
					if (abs(t - t_lk) < 0.01) {
						break;
					}
					t_lk = t;
					if (j > 10) {
						error_message_ = "Error when sloving time";
						return false;
					}
				}
				t = abs(t_lk);
				
				forward_motion_func_(pitch, h, t);
				error_h = target_h / 1000 - h;
				if (abs(error_h) < 0.01) {
					break;
				}
				if (i > 19) {
					error_message_ = "Error when sloving pitch";
					return false;
				}
				pitch = pitch + K * error_h;
			}
			angle = pitch + 2.5;
			return true;
		}
	}

}