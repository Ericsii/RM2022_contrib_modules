#ifndef RM_TRAJECTORY_GRAVITY_HPP_
#define RM_TRAJECTORY_GRAVITY_HPP_

#include "rm_trajectory/trajectory_interface.hpp"
#include "trajectory_tool.hpp"

#include <string>
#include <memory>

namespace rm_trajectory 
{

class TrajectoryGravity : public TrajectoryInterface {
public:
    explicit TrajectoryGravity(double initial_vel);

    void set_initial_vel(double vel) {initial_vel_ = vel;}
    std::shared_ptr<TrajectoryTool> get_iterative_tool() {return iterative_tool_;}
    bool solve(double target_x, double target_h, double &angle) override;
    std::string error_message() override;

private:
    std::shared_ptr<TrajectoryTool> iterative_tool_;
    double initial_vel_;
};

}
#endif // RM_TRAJECTORY_GRAVITY_HPP_