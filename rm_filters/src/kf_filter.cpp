#include "rm_filters/kf_filter.hpp"

#include <stdexcept>

namespace rm_filters
{
	void KalmanFilter::init(Eigen::VectorXd &x_k)
	{
		this->x_p_k = x_k;
		this->x_l_k = x_k;
		this->P = Eigen::MatrixXd::Zero(dim_x, dim_x);
	}

	Eigen::VectorXd KalmanFilter::predict(Eigen::VectorXd &u, double t)
	{
		(void) t;
		x_p_k = A * x_l_k + B * u;
		P = A * P * A.transpose() + Q;
		return x_p_k;
	}

	Eigen::VectorXd KalmanFilter::predict(Eigen::VectorXd &u)
	{
		x_p_k = A * x_l_k + B * u;
		P = A * P * A.transpose() + Q;
		return x_p_k;
	}

	Eigen::VectorXd KalmanFilter::update(Eigen::VectorXd &z_k)
	{
		K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
		x_k = x_p_k + K * (z_k - H * x_p_k);
		P = P - K * H * P;
		x_l_k = x_k;
		return x_k;
	}
}