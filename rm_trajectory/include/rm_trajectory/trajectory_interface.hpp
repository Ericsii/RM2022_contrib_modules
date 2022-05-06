#ifndef RM_TRAJECTORY_INTERFACE_HPP_
#define RM_TRAJECTORY_INTERFACE_HPP_

#include <string>
#include <math.h>
#include <memory>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>

namespace rm_trajectory
{
	const double DEC = 3.1415926 / 180;

class TrajectoryInterface{
public:
	virtual bool solve(double target_distance, double target_h, double & angle) = 0;
	virtual std::string error_message() = 0;
};
}

#endif // RM_TRAJECTORY_INTERFACE_HPP_