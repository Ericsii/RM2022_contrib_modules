#ifndef RM_TRAJECTORY_PITCH_HPP_
#define RM_TRAJECTORY_PITCH_HPP_

#include "rm_trajectory/trajectory_interface.hpp"

namespace rm_trajectory
{

class GetPitch : public TrajectoryInterface {
public:
    explicit GetPitch(double initial_vel)
    : initial_vel_(initial_vel) {}

    double get_pitch(double target_distance, double target_h, double bullet_v);

public:
    double pitch_1(double target_h);
    double pitch_3(double target_h);
    double pitch_5(double target_h);
    double pitch_7(double target_h);

private:
    double initial_vel_;
};

}

#endif //RM_TRAJECTORY_PITCH_HPP_