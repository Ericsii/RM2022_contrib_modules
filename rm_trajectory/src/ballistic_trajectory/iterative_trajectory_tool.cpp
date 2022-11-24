#include "rm_trajectory/iterative_trajectory_tool.hpp"

namespace rm_trajectory
{
	bool IterativeTrajectoryTool::solve(double target_distance, double target_h, double & angle) {

		double pitch = 0;
		target_distance = target_distance / 1000;
		//这里将单位转换为米，因为测试数据距离单位为米

		if (database_flag_) {
			forward_motion_func_(target_distance, target_h, pitch);
			angle = pitch;
			return true;
		}
		else {
			double h = 0, error_h = 0;
			double t = 0, t_lk = 0;
			double K = 2;				//K为比例补偿迭代求解的比例系数，类似于PID的P

			for (int i = 0; i < max_iter_; i++) {
				for (int j = 0; j < max_iter_; j++) {
					forward_time_func_(pitch, target_distance, t);
					if (abs(t - t_lk) < 0.01) {
						break;
					}
					t_lk = t;
					if (j > 10) {		
						//一般求解迭代时间不会超过10次，如果迭代时间过长，请检查是否比例系数过小，或者算法问题
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
				if (i > (max_iter_ - 1)) {
					error_message_ = "Error when sloving pitch";
					return false;
				}
				pitch = pitch + K * error_h;
			}
			angle = pitch;
			return true;
		}
	}

}