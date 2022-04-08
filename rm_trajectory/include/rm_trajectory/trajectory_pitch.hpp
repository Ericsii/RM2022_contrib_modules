#ifndef RM_TRAJECTORY_PITCH_HPP_
#define RM_TRAJECTORY_PITCH_HPP_

namespace rm_trajectory
{

class GetPitch {
public:
    explicit GetPitch(double initial_vel);

    double get_pitch(double target_distance, double target_h, double bullet_v);

public:
    double pitch_1(double target_h, double bullet_v);
    double pitch_3(double target_h, double bullet_v);
    double pitch_5(double target_h, double bullet_v);
    double pitch_7(double target_h, double bullet_v);

private:
    double initial_vel_;
};

}

#endif //RM_TRAJECTORY_PITCH_HPP_