#ifndef RM_TRAJECTORY_PITCH_HPP_
#define RM_TRAJECTORY_PITCH_HPP_

namespace rm_trajectory
{

const double pi = 3.14159265;
const double DEC = pi / 180;

class GetPitch {
public:
    explicit GetPitch(double initial_vel, double air_drag, double K, double correction);

    double get_pitch(double target_distance, double target_h, double bullet_v);
    double get_pitch_model(double target_distance, double target_h, double bullet_v);

public:
    double pitch_1(double target_h, double bullet_v);
    double pitch_3(double target_h, double bullet_v);
    double pitch_5(double target_h, double bullet_v);
    double pitch_7(double target_h, double bullet_v);

public:
    double air_drag_;                                    //空气阻力系数
    double K_;                                           //比例系数
    double correction_;                                  //模型偏差修正值pitch

private:
    double initial_vel_;
};

}

#endif //RM_TRAJECTORY_PITCH_HPP_