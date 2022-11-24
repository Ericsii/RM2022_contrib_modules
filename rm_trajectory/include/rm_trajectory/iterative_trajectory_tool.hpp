#ifndef RM_PROJECTORY_ITERATIVE_TRAJECTORY_TOOL_HPP_
#define RM_PROJECTORY_ITERATIVE_TRAJECTORY_TOOL_HPP_

#include "rm_trajectory/trajectory_interface.hpp"

#include <functional>

namespace rm_trajectory
{

class IterativeTrajectoryTool {

	typedef std::function<void (double given_angle, double given_distance, double & t)> ForwardTimeFunc;
	typedef std::function<void (double given_angle, double &h, double & t)> ForwardMotionFunc;

public:
	IterativeTrajectoryTool() {}
	void set_max_iter(int max_iter) { max_iter_ = max_iter; }
	void set_forward_motion(ForwardMotionFunc forward_motion) { forward_motion_func_ = forward_motion; }
	void set_forward_time(ForwardTimeFunc forward_time) { forward_time_func_ = forward_time; }
	void set_is_database(bool is_data_base) { database_flag_ = is_data_base; }
	bool solve(double target_distance, double target_h, double & angle);
	std::string error_message() { return error_message_; }

private:
	int max_iter_{ 50 };			//最大迭代次数
	bool database_flag_{false};		//数据查表标志位
	ForwardMotionFunc forward_motion_func_;		//运动轨迹求解器
	ForwardTimeFunc forward_time_func_;			//飞行时间求解器
	std::string error_message_;
};

}

#endif // RM_PROJECTORY_ITERATIVE_TREJECTORY_TOOL_HPP_