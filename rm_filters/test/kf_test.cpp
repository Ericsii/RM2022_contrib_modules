#include "gtest/gtest.h"
#include <memory>
#include <Eigen/Dense>
#include <vector>
#include <string>

#include "rm_filters/rm_filters.hpp"

TEST(Filter, KalmanFilter)
{
    using namespace rm_filters;
    std::shared_ptr<FilterInterface> filter;
    auto kf = std::make_shared<KalmanFilter>(6, 3);

    Eigen::MatrixXd A(6, 6);
    A << 1, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 1,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    Eigen::MatrixXd B(6, 6);
    B << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;

    Eigen::MatrixXd H(3, 6);
    H << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0;

    Eigen::MatrixXd Q(6, 6);
    Q << 0.01, 0, 0, 0, 0, 0,
        0, 0.01, 0, 0, 0, 0,
        0, 0, 0.01, 0, 0, 0,
        0, 0, 0, 0.01, 0, 0,
        0, 0, 0, 0, 0.01, 0,
        0, 0, 0, 0, 0, 0.01;

    Eigen::MatrixXd R(3, 3);
    R << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;

    kf->A = A;
    kf->B = B;
    kf->H = H;
    kf->Q = Q;
    kf->R = R;
    filter = kf;
    Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(6);

    filter->init(x_0);

    for(int i = 0; i < 100; ++i)
    {
        Eigen::VectorXd u(6);
        Eigen::VectorXd z(3);
        u << 0, 0, 0, 0, 0, 0;
        z << i, i, i;
        filter->predict(u);
        auto x = filter->update(z);
    }

    EXPECT_TRUE(true);
}

TEST(Filter, ExKalmanFilter)
{
    using namespace rm_filters;
    std::shared_ptr<FilterInterface> filter;

    Eigen::MatrixXd Q(6, 6);
    Q << 0.01, 0, 0, 0, 0, 0,
        0, 0.01, 0, 0, 0, 0,
        0, 0, 0.01, 0, 0, 0,
        0, 0, 0, 0.01, 0, 0,
        0, 0, 0, 0, 0.01, 0,
        0, 0, 0, 0, 0, 0.01;

    Eigen::MatrixXd R(3, 3);
    R << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;

    auto ekf = std::make_shared<ExKalmanFilter>(6, 3, 0, Q, R);
    ekf->base_state = MState::const_acc;
    ekf->df_state = MState::df_const_acc;
    ekf->se_df_state = MState::se_df_const_acc;
    ekf->base_sensor = MState::const_acc_sensor;
    ekf->df_sensor = MState::df_const_acc_sensor;
    ekf->se_df_sensor = MState::se_df_const_acc_sensor;

    filter = ekf;

    Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(6);
    filter->init(x_0);

    for (int i = 0; i < 100; ++i)
    {
        Eigen::VectorXd u(6);
        Eigen::VectorXd z(3);
        u << 0, 0, 0, 0, 0, 0;
        z << i, i, i;
        filter->predict(u, 0.01);
        auto x = filter->update(z);
    }

    EXPECT_TRUE(true);
}