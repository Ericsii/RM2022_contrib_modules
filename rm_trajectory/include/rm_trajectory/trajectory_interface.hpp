#ifndef RM_TRAJECTORY_INTERFACE_HPP_
#define RM_TRAJECTORY_INTERFACE_HPP_

#include <string>
#include <math.h>
#include <memory>
#include <Eigen/Dense>

namespace rm_trajectory
{
	const double DEC = M_PI / 180;

class TrajectoryInterface{
public:
	virtual void set_initial_vel(double vel) = 0;
	virtual bool solve(double target_distance, double target_h, double & angle) = 0;
	virtual std::string error_message() = 0;
};
}

#endif // RM_TRAJECTORY_INTERFACE_HPP_