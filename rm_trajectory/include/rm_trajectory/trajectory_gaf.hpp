#ifndef RM_TRAJECTORY_GAF_HPP_
#define RM_TRAJECTORY_GAF_HPP_

#include "rm_trajectory/trajectory_tool.hpp"
#include "rm_trajectory/trajectory_interface.hpp"

#include <string>
#include <memory>

namespace rm_trajectory
{

class TrajectoryGaf : public TrajectoryInterface {
public:
    TrajectoryGaf(double initial_vel, double friction_coeff);

    void set_initial_vel(double vel) {initial_vel_ = vel;}
    void set_friction_coeff(const double &friction_coeff) {friction_coeff_ = friction_coeff;}
    std::shared_ptr<TrajectoryTool> get_iterative_tool() {return iterative_tool_;}
    bool solve(double target_x, double target_h, double &angle) override;
    std::string error_message() override;

private:
    std::shared_ptr<TrajectoryTool> iterative_tool_;
    double initial_vel_;
    double friction_coeff_;
};

}
#endif // RM_TRAJECTORY_GAF_HPP_