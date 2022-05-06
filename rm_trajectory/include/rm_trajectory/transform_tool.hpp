#ifndef RM_TRAJECTORY_TRANSFORM_TOOL_HPP_
#define RM_TRAJECTORY_TRANSFORM_TOOL_HPP_

#include "rm_trajectory/trajectory_interface.hpp"

namespace rm_trajectory
{

class TransformTool {
public:
	explicit TransformTool(std::shared_ptr<TrajectoryInterface> solver = nullptr)
		: solver_(solver) {}

	void set_trajectory_solver(std::shared_ptr<TrajectoryInterface> solver) { solver_ = solver; }
	std::string error_message() { return error_message_; }
	bool solve(double x, double y, double z, double &pitch, double &yaw);
	bool solve(Eigen::Vector3d position, double &pitch, double &yaw) {
		return solve(position(0), position(1), position(2), pitch, yaw);
	}
	bool solve(geometry_msgs::msg::Point position, double &pitch, double &yaw) {
		return solve(position.x, position.y, position.z, pitch, yaw);
	}

public:
	double pitch_;
	double yaw_;

private:
	std::shared_ptr<TrajectoryInterface> solver_;
	std::string error_message_;
};

}

#endif // RM_TRAJECTORY_GIMBAL_TRANSFORM_TOOL_HPP_