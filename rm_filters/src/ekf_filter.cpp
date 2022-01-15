#include "rm_filters/ekf_filter.hpp"

namespace rm_filters {

Eigen::MatrixXd ExKalman::predict(Eigen::MatrixXd &U, double t) {
	A = df_state(x_l_k, U, t);
	W = se_df_state(x_l_k, U, t);
	x_p_k = base_state(x_l_k, U, t);
	P = A * P * A.transpose() + W * Q * W.transpose();
	return x_p_k;
}
Eigen::MatrixXd ExKalman::update(Eigen::MatrixXd &z_k) {
	H = df_sensor(x_p_k);
	V = se_df_sensor(x_p_k);
	K = P * H.transpose() * (H * P * H.transpose() + V * R * V.transpose()).inverse();
	x_k = x_p_k + K * (z_k - base_sensor(x_p_k));
	P = P - K * H * P;
	x_l_k = x_k;
	return x_k;
}
}