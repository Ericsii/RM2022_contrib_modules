#ifndef RM_TRAJECTORY_GRAVITY_PROJECTIL_SOLVER_HPP_
#define RM_TRAJECTORY_GRAVITY_PROJECTIL_SOLVER_HPP_

#include "rm_trajectory/trajectory_interface.hpp"
#include "rm_trajectory/iterative_trajectory_tool.hpp"

#include <memory>

namespace rm_trajectory
{
    class GravityProjectileSolver : public TrajectoryInterface
    {
    public:
        explicit GravityProjectileSolver(double initial_vel);

        void set_initial_vel(double vel) { initial_vel_ = vel; }
        std::shared_ptr<IterativeTrajectoryTool> get_iterative_tool() { return iterative_tool_; }
        bool solve(double target_x, double target_h, double &angle) override;
        std::string error_message() override;

    private:
        std::shared_ptr<IterativeTrajectoryTool> iterative_tool_;
        double initial_vel_;
    };
} // namespace rm_trajectory

#endif