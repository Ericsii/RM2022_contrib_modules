#include "rm_filters/kf_filter.hpp"

namespace rm_filters
{

	Eigen::MatrixXd Kalman::predict(Eigen::MatrixXd &U, double t)
	{
        double delta_t = t;

        Eigen::MatrixXd A(Matrix_x, Matrix_x);
		A(0, 0) = 1, A(0, 1) = 0, A(0, 2) = 0, A(0, 3) = delta_t, A(0, 4) = 0;
		A(1, 0) = 0, A(1, 1) = 1, A(1, 2) = 0, A(1, 3) = 0, A(1, 4) = delta_t;
		A(2, 0) = 0, A(2, 1) = 0, A(2, 2) = 1, A(2, 3) = 0, A(2, 4) = 0;
		A(3, 0) = 0, A(3, 1) = 0, A(3, 2) = 0, A(3, 3) = 1, A(3, 4) = 0;
		A(4, 0) = 0, A(4, 1) = 0, A(4, 2) = 0, A(4, 3) = 0, A(4, 4) = 1;

	    Eigen::MatrixXd B(Matrix_x, 1);
		B(0, 0) = pow(delta_t, 2) / 2;
		B(1, 0) = pow(delta_t, 2) / 2;
		B(2, 0) = pow(delta_t, 2) / 2;
		B(3, 0) = delta_t;
		B(4, 0) = delta_t;

		x_p_k = A * x_l_k + B * U;
		P = A * P * A.transpose() + Q;
		return P;
	}
	Eigen::MatrixXd Kalman::update(Eigen::MatrixXd &z_k)
	{
		K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
		x_k = x_p_k + K * (z_k - H * x_p_k);
		P = P - K * H * P;
		x_l_k = x_k;
		return x_k;
	}
}