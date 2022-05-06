#ifndef RM_TRAJECTORY_GRAVITY_SOLVER_HPP_
#define RM_TRAJECTORY_GRAVITY_SOLVER_HPP_

#include "rm_trajectory/trajectory_interface.hpp"
#include "rm_trajectory/iterative_trajectory_tool.hpp"
#include "rm_trajectory/transform_tool.hpp"

namespace rm_trajectory
{

class GravitySolver : public TrajectoryInterface
{
public:
	explicit GravitySolver(double initial_vel, double friction_coeff);

	void set_initial_vel(double vel) { initial_vel_ = vel; }
	void set_friction_coeff(const double & friction_coeff)  { friction_coeff_ = friction_coeff; }
	bool solve(double target_distance, double target_h, double & angle) override;
	std::string error_message() override;

private:
	std::shared_ptr<IterativeTrajectoryTool> iterative_tool_;
	double initial_vel_;
	double friction_coeff_;
};

}

#endif	// RM_TRAJECTORY_GRAVITY_SOLVER_HPP_