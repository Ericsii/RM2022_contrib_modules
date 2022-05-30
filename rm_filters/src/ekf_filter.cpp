#include "rm_filters/ekf_filter.hpp"

namespace rm_filters {

void ExKalmanFilter::init(Eigen::VectorXd &x_k) {
	this->x_p_k = x_k;
	this->x_l_k = x_k;
	this->P = Eigen::MatrixXd::Zero(dim_x, dim_x);
}

Eigen::VectorXd ExKalmanFilter::predict(Eigen::VectorXd &u, double t) {
	A = df_state(x_l_k, u, t);
	W = se_df_state(x_l_k, u, t);
	x_l_k = base_state(x_l_k, u, t);
	P = A * P * A.transpose() + W * Q * W.transpose();
	return x_l_k;
}

Eigen::VectorXd ExKalmanFilter::predict(Eigen::VectorXd &u) {
	double t = 0.01;
	A = df_state(x_l_k, u, t);
	W = se_df_state(x_l_k, u, t);
	x_l_k = base_state(x_l_k, u, t);
	P = A * P * A.transpose() + W * Q * W.transpose();
	return x_l_k;
}

Eigen::VectorXd ExKalmanFilter::update(Eigen::VectorXd &z_k) {

	H = df_sensor(x_l_k);
	V = se_df_sensor(x_l_k);
	K = P * H.transpose() * (H * P * H.transpose() + V * R * V.transpose()).inverse();
	x_k = x_l_k + K * (z_k - base_sensor(x_l_k));
	P = P - K * H * P;
	x_l_k = x_k;
	return x_k;
}
}