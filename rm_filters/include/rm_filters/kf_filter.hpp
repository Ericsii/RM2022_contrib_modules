#ifndef RM_FILTERS__KF_FILTER_HPP_
#define RM_FILTERS__KF_FILTER_HPP_

#include "rm_filters/filter_interface.hpp"

namespace rm_filters
{
    class Kalman : public Filters
    {
    public:
        Kalman(const Eigen::MatrixXd &x_p_k, Eigen::MatrixXd &x_l_k,
               const Eigen::MatrixXd &H,const Eigen::MatrixXd &Q, 
               const Eigen::MatrixXd &P,const Eigen::MatrixXd &R)
        {
            this->H = H;
            this->P = P;
            this->R = R;
            this->Q = Q;
            this->x_p_k = x_p_k;
            this->x_l_k = x_l_k;
        }
        Eigen::MatrixXd predict(Eigen::MatrixXd &U, double t) override;
        Eigen::MatrixXd update(Eigen::MatrixXd &z_k) override;

    public:
        Eigen::MatrixXd x_p_k;
        Eigen::MatrixXd x_l_k;
        Eigen::MatrixXd x_k;
        Eigen::MatrixXd B;
        Eigen::MatrixXd K;
        Eigen::MatrixXd A;
        Eigen::MatrixXd H;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd P;

    public:
        Eigen::MatrixXd R;
        Eigen::MatrixXd z_k;
        Eigen::MatrixXd U;
    };
}

#endif // RM_FILTERS__KF_FILTER_HPP_