#ifndef RM_TRAJECTORY_INTERFACE_HPP_
#define RM_TRAJECTORY_INTERFACE_HPP_
namespace rm_trajectory
{

class TrajectoryInterface {
public:
    virtual bool solve(double target_x, double target_h, double &angle) = 0;
    virtual std::string error_message() = 0;
};

}
#endif // RM_TRAJECTORY_INTERFACE_HPP_