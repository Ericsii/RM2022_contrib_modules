#ifndef RM_TRAJECTORY_DATABASE_SOLVER_HPP_
#define RM_TRAJECTORY_DATABASE_SOLVER_HPP_

#include "rm_trajectory/trajectory_interface.hpp"
#include "rm_trajectory/iterative_trajectory_tool.hpp"
#include "rm_trajectory/transform_tool.hpp"

namespace rm_trajectory
{

class DatabaseSolver : public TrajectoryInterface
{
public:
	explicit DatabaseSolver(double initial_vel);

	void set_initial_vel(double vel) { initial_vel_ = vel; }
	bool solve(double target_distance, double target_h, double & angle) override;
	std::string error_message() override;

public:
	double get_pitch(double target_distance, double target_h, double bullet_v);
	double pitch_1(double target_h, double bullet_v);
	double pitch_3(double target_h, double bullet_v);
	double pitch_5(double target_h, double bullet_v);
	double pitch_7(double target_h, double bullet_v);

private:
	std::shared_ptr<IterativeTrajectoryTool> iterative_tool_;
	double initial_vel_;
};

}

#endif	// RM_TRAJECTORY_GRAVITY_SOLVER_HPP_