#ifndef RM_TRAJECTORY_GRAVITY_NOFRICTION_SOLVER_HPP_
#define RM_TRAJECTORY_GRAVITY_NOFRICTION_SOLVER_HPP_

#include "rm_trajectory/trajectory_interface.hpp"
#include "rm_trajectory/iterative_trajectory_tool.hpp"
#include "rm_trajectory/transform_tool.hpp"

namespace rm_trajectory
{

class GravityNofrictionSolver : public TrajectoryInterface
{
public:
	explicit GravityNofrictionSolver(double initial_vel);

	void set_initial_vel(double vel) { initial_vel_ = vel; }
	bool solve(double target_distance, double target_h, double & angle) override;
	std::string error_message() override;

private:
	std::shared_ptr<IterativeTrajectoryTool> iterative_tool_;
	double initial_vel_;
};

}

#endif	// RM_TRAJECTORY_GRAVITY_Nofriction_SOLVER_HPP_