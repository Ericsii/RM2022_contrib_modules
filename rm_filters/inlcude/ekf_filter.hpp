#ifndef RM_FILTER_EKF_FILTER_HPP_
#define RM_FILTER_EKF_FILTER_HPP_
#include "rm_filters/filter_interface.hpp"
#include "rm_filters/filter_state.hpp"

namespace rm_filters
{
 
    class ExKalman : public Filters
    {
    public:
        ExKalman(Eigen::MatrixXd Q, Eigen::MatrixXd R,
                 kinestate base_state, kinestate df_state,
                 kinestate se_df_state, sensor base_sensor,
                 sensor df_sensor, sensor se_df_sensor)
        {
            this->Q = Q;
            this->R = R;
            this->base_state = base_state;
            this->df_state = df_state;
            this->se_df_state = se_df_state;
            this->base_sensor = base_sensor;
            this->df_sensor = df_sensor;
            this->se_df_sensor = se_df_sensor;
        }
        void init(Eigen::MatrixXd &z_k) override;
        Eigen::MatrixXd predict(Eigen::MatrixXd &U, double t) override;
        Eigen::MatrixXd update(Eigen::MatrixXd &z_k) override;

    private:
        Eigen::MatrixXd x_p_k;
        Eigen::MatrixXd x_l_k;
        Eigen::MatrixXd x_k;
        Eigen::MatrixXd K;
        Eigen::MatrixXd A;
        Eigen::MatrixXd W;
        Eigen::MatrixXd H;
        Eigen::MatrixXd V;
        Eigen::MatrixXd P;
        Eigen::MatrixXd U;
        Eigen::MatrixXd Q;

    private:
        Eigen::MatrixXd R;
        Eigen::MatrixXd z_k;

    public:
        kinestate base_state, df_state, se_df_state;
        sensor base_sensor, df_sensor, se_df_sensor;

    };
}

#endif //RM_FILTER_EKF_FILTER_HPP_