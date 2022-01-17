#include "rm_filters/ekf_filter.hpp"

namespace rm_filters {

void ExKalman::init(Eigen::MatrixXd &z_k) {
	
	Eigen::MatrixXd x_p_k(Matrix_x, 1);
	x_p_k(0, 0) = z_k(0, 0); x_p_k(1, 0) = z_k(1, 0), x_p_k(2, 0) = z_k(2, 0);
	x_p_k(3, 0) = 0; x_p_k(4, 0) = 0; x_p_k(5, 0) = 0;

	this->x_p_k = x_p_k;
	this->x_l_k = x_p_k;

	Eigen::MatrixXd P(Matrix_x, Matrix_x);
    P = Eigen::MatrixXd::Zero(Matrix_x, Matrix_x);

	this->P = P;
}

Eigen::MatrixXd ExKalman::predict(Eigen::MatrixXd &U, double t) {

	Eigen::MatrixXd Q(Matrix_x, Matrix_x);
    Q(0, 0) = 1; Q(0, 1) = 0;  Q(0, 2) = 0; 
	Q(0, 3) = 0; Q(0, 4) = 0;  Q(0, 5) = 0;
	Q(1, 0) = 0; Q(1, 1) = 1;  Q(1, 2) = 0; 
	Q(1, 3) = 0; Q(1, 4) = 0;  Q(1, 5) = 0;
	Q(2, 0) = 0; Q(2, 1) = 0;  Q(2, 2) = 1; 
	Q(2, 3) = 0; Q(2, 4) = 0;  Q(2, 5) = 0;
	Q(3, 0) = 0; Q(3, 1) = 0;  Q(3, 2) = 0; 
	Q(3, 3) = 1; Q(3, 4) = 0;  Q(3, 5) = 0;
    Q(4, 0) = 0; Q(4, 1) = 0;  Q(4, 2) = 0; 
	Q(4, 3) = 0; Q(4, 4) = 1;  Q(4, 5) = 0;
	Q(5, 0) = 0; Q(5, 1) = 0;  Q(5, 2) = 0;
	Q(5, 3) = 0; Q(5, 4) = 0;  Q(5, 5) = 1;

	A = df_state(x_l_k, U, t);
	W = se_df_state(x_l_k, U, t);
	x_p_k = base_state(x_l_k, U, t);
	P = A * P * A.transpose() + W * Q * W.transpose();
	return x_p_k;
}
Eigen::MatrixXd ExKalman::update(Eigen::MatrixXd &z_k) {

	Eigen::MatrixXd R(Matrix_y, Matrix_y);
    R(0, 0) = 1, R(0, 1) = 0, R(0, 2) = 0;
    R(1, 0) = 0, R(1, 1) = 1, R(1, 2) = 0;
    R(2, 0) = 0, R(2, 1) = 0, R(2, 2) = 1;

	H = df_sensor(x_p_k);
	V = se_df_sensor(x_p_k);
	K = P * H.transpose() * (H * P * H.transpose() + V * R * V.transpose()).inverse();
	x_k = x_p_k + K * (z_k - base_sensor(x_p_k));
	P = P - K * H * P;
	x_l_k = x_k;
	return x_k;
}

}