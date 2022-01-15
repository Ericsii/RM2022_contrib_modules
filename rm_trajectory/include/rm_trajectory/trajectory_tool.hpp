#ifndef RM_TRAJECTORY_TOOL_HPP_
#define RM_TRAJECTORY_TOOL_HPP_

#include "rm_trajectory/trajectory_interface.hpp"
#include "rm_trajectory/trajectory_pitch.hpp"

#include <memory>
#include <Eigen/Dense>

namespace rm_trajectory
{
class TrajectoryTool {
    typedef std::function<void (double given_angle, double given_x, 
    double &h, double &t)> ForwardMotionFunc;

public:
    TrajectoryTool() {}
    void set_max_iter(int max_iter) {max_iter_ = max_iter;}
    void set_forward_motion(ForwardMotionFunc forward_motion) {forward_motion_func_ = forward_motion;}
    bool solve(double target_x, double target_h, double &angle);
    std::string error_message() {return error_message_;}

private:
    int max_iter_{20};
    ForwardMotionFunc forward_motion_func_;
    std::string error_message_;
};

}
#endif // RM_TRAJECTORY_TOOL_HPP_