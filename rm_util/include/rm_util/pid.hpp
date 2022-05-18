#ifndef RM_UTIL__PID_HPP_
#define RM_UTIL__PID_HPP_

namespace rm_util
{
    class PID
    {
    public:
        PID(float kp, float ki, float kd, float dt, float min_output = 0, float max_output = 0);

        void set_kp(float kp);
        void set_ki(float ki);
        void set_kd(float kd);
        void set_dt(float dt);

        float get_kp();
        float get_ki();
        float get_kd();
        float get_dt();

        float calc(float target, float current, float dt = 0.0f);
    
    private:
        float kp_;
        float ki_;
        float kd_;
        float dt_;

        float last_error_;
        float integral_;

        float limit_min_, limit_max_;
    };
} // namespace rm_util


#endif // RM_UTIL__PID_HPP_