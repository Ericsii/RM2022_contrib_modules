#ifndef RM_FILTERS__KF_FILTER_HPP_
#define RM_FILTERS__KF_FILTER_HPP_

#include "rm_filters/filter_interface.hpp"

namespace rm_filters
{
    class Kalman : public Filters
    {
    public:
        Kalman(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R)
        {
            this->R = R;
            this->Q = Q;
        }
        void init(Eigen::MatrixXd &z_k) override;
        Eigen::MatrixXd predict(Eigen::MatrixXd &U, double t) override;
        Eigen::MatrixXd update(Eigen::MatrixXd &z_k) override;

    private:
        Eigen::MatrixXd x_p_k;
        Eigen::MatrixXd x_l_k;
        Eigen::MatrixXd x_k;
        Eigen::MatrixXd B;
        Eigen::MatrixXd K;
        Eigen::MatrixXd A;
        Eigen::MatrixXd H;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd P;

    private:
        Eigen::MatrixXd R;
        Eigen::MatrixXd z_k;
        Eigen::MatrixXd U;
    };
}

#endif // RM_FILTERS__KF_FILTER_HPP_