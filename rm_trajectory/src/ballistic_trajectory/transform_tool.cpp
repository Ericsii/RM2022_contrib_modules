#include "rm_trajectory/transform_tool.hpp"

namespace rm_trajectory
{

bool TransformTool::solve(double x, double y, double z, double &pitch, double &yaw) {
	
	double target_distance = sqrt((y * y) + (x * x));
	double target_h = z;

	if (!solver_) {
		pitch_ = atan2(sqrt((y * y) + (x * x)), z);
	}
	else {
		double angle = 0;
		if (solver_->solve(target_distance, target_h, angle)) {
			pitch_ = angle;
		}
		else {
			error_message_ = solver_->error_message();
			return false;
		}
	}
	yaw_ = atan2(x, y);
	return true;
}

}