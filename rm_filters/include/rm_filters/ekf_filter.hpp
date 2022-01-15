#ifndef RM_FILTER_EKF_FILTER_HPP_
#define RM_FILTER_EKF_FILTER_HPP_
#include "rm_filters/filter_interface.hpp"

namespace rm_filters
{
    class ExKalman : public Filters
    {
    public:
        ExKalman(const Eigen::MatrixXd &x_p_k, const Eigen::MatrixXd &x_l_k,
                 const Eigen::MatrixXd &Q, const Eigen::MatrixXd &P, const Eigen::MatrixXd &R,
                 kinestate base_state, kinestate df_state,
                 kinestate se_df_state, sensor base_sensor,
                 sensor df_sensor, sensor se_df_sensor)
        {
            this->x_p_k = x_p_k;
            this->x_l_k = x_l_k;
            this->Q = Q;
            this->P = P;
            this->R = R;
            this->base_state = base_state;
            this->df_state = df_state;
            this->se_df_state = se_df_state;
            this->base_sensor = base_sensor;
            this->df_sensor = df_sensor;
            this->se_df_sensor = se_df_sensor;
        }
        Eigen::MatrixXd predict(Eigen::MatrixXd &U, double t) override;
        Eigen::MatrixXd update(Eigen::MatrixXd &z_k) override;

    public:
        Eigen::MatrixXd x_p_k;
        Eigen::MatrixXd x_l_k;
        Eigen::MatrixXd x_k;
        Eigen::MatrixXd K;
        Eigen::MatrixXd A;
        Eigen::MatrixXd W;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd P;
        Eigen::MatrixXd H;
        Eigen::MatrixXd V;

    public:
        Eigen::MatrixXd R;
        Eigen::MatrixXd z_k;
        Eigen::MatrixXd U;

    public:
        kinestate base_state, df_state, se_df_state;
        sensor base_sensor, df_sensor, se_df_sensor;
    };
}

#endif //RM_FILTER_EKF_FILTER_HPP_