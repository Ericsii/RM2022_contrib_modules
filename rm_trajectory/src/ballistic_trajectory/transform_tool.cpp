#include "rm_trajectory/transform_tool.hpp"

namespace rm_trajectory
{

bool TransformTool::solve(double x, double y, double z, double &pitch, double &yaw) {
	
	x *= 10; y *= 10; z *= 10;
	//识别后的目标单位为cm，这里转换为mm

	double target_distance = sqrt((y * y) + (x * x));
	double target_h = z;

	if (!solver_) {
		pitch = atan2(sqrt((y * y) + (x * x)), z);
	}
	else {
		double angle = 0;
		if (solver_->solve(target_distance, target_h, angle)) {
			pitch = angle;
		}
		else {
			error_message_ = solver_->error_message();
			return false;
		}
	}
	yaw = atan2(x, y);
	yaw = yaw / DEC;  									//弧度制转为角度制
	return true;
}

}