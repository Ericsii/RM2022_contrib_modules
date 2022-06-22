#include "rm_filters/filter_state.hpp"

namespace rm_filters
{
	Eigen::MatrixXd MState::const_acc(Eigen::VectorXd &x_l_k, Eigen::VectorXd &u, double t)
	{
		double delta_t = t;
		Eigen::MatrixXd A(6, 6);
		A << 1., 0., 0., delta_t, 0., 0.,
			0., 1., 0., 0., delta_t, 0.,
			0., 0., 1., 0., 0., delta_t,
			0., 0., 0., 1., 0., 0.,
			0., 0., 0., 0., 1., 0.,
			0., 0., 0., 0., 0., 1.;

		Eigen::MatrixXd B(6, 6);
		B << 0., 0., 0., delta_t * delta_t / 2., 0., 0.,
			0., 0., 0., 0., delta_t * delta_t / 2., 0.,
			0., 0., 0., 0., 0., delta_t * delta_t / 2.,
			0., 0., 0., delta_t, 0., 0.,
			0., 0., 0., 0., delta_t, 0.,
			0., 0., 0., 0., 0., delta_t;

		Eigen::MatrixXd x_p = A * x_l_k + B * u;
		return x_p;
	}

	Eigen::MatrixXd MState::df_const_acc(Eigen::VectorXd &x_l_k, Eigen::VectorXd &U, double t)
	{
		(void) x_l_k;
		(void) U;
		double delta_t = t;
		Eigen::MatrixXd jacobian(6, 6);
		jacobian << 1., 0., 0., delta_t, 0., 0.,
			0., 1., 0., 0., delta_t, 0.,
			0., 0., 1., 0., 0., delta_t,
			0., 0., 0., 1., 0., 0.,
			0., 0., 0., 0., 1., 0.,
			0., 0., 0., 0., 0., 1.;
		return jacobian;
	}

	Eigen::MatrixXd MState::se_df_const_acc(Eigen::VectorXd &x_l_k, Eigen::VectorXd &U, double t)
	{
		(void) x_l_k;
		(void) U;
		(void) t;
		Eigen::MatrixXd hessian(6, 6);
		hessian << 1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0,
			0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1;
		return hessian;
	}

	Eigen::MatrixXd MState::const_acc_sensor(Eigen::VectorXd &x_p_k)
	{
		Eigen::MatrixXd H(3, 6);
		H << 1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0;
		return H * x_p_k;
	}

	Eigen::MatrixXd MState::df_const_acc_sensor(Eigen::VectorXd &x_p_k)
	{
		(void) x_p_k;
		Eigen::MatrixXd dH(3, 6);
		dH << 1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0;
		return dH;
	}

	Eigen::MatrixXd MState::se_df_const_acc_sensor(Eigen::VectorXd &x_p_k)
	{
		(void) x_p_k;
		Eigen::MatrixXd ddH(3, 3);
		ddH << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;
		return ddH;
	}

	// Eigen::MatrixXd MState::ctrv(Eigen::MatrixXd &x_l_k, Eigen::MatrixXd &U, double t)
	// {
	// 	double ctrv_delta_t = t;
	// 	Eigen::MatrixXd x_p_k(Matrix_x, 1);
	// 	Eigen::MatrixXd F_Matrix(Matrix_x, Matrix_x);
	// 	Eigen::MatrixXd B(Matrix_x, Matrix_y);

	// 	B(0, 0) = cos(x_l_k(3, 0) * DEC) *
	// 			  pow(ctrv_delta_t, 2) / 2;
	// 	B(1, 0) = sin(x_l_k(3, 0) * DEC) *
	// 			  pow(ctrv_delta_t, 2) / 2;
	// 	B(2, 0) = ctrv_delta_t;
	// 	B(3, 1) = pow(ctrv_delta_t, 2) / 2;
	// 	B(4, 1) = ctrv_delta_t;
	// 	B(0, 1) = 0;
	// 	B(1, 1) = 0;
	// 	B(2, 1) = 0;
	// 	B(3, 0) = 0;
	// 	B(4, 0) = 0;

	// 	if (x_l_k(4, 0) != 0)
	// 	{
	// 		F_Matrix(0, 0) = (1 + x_l_k(2, 0) * (sin(x_l_k(3, 0) * DEC + ctrv_delta_t * x_l_k(4, 0) * DEC) - sin(x_l_k(3, 0) * DEC)) /
	// 								  (x_l_k(4, 0) * x_l_k(0, 0)));
	// 		F_Matrix(1, 1) = (1 + x_l_k(2, 0) * (-cos(x_l_k(3, 0) * DEC + ctrv_delta_t * x_l_k(4, 0) * DEC) + cos(x_l_k(3, 0) * DEC)) /
	// 								  (x_l_k(4, 0) * x_l_k(1, 0)));
	// 	}
	// 	else if (x_l_k(4, 0) == 0)
	// 	{
	// 		F_Matrix(0, 0) = (1 + x_l_k(2, 0) * ctrv_delta_t * cos(x_l_k(3, 0) * DEC) / x_l_k(0, 0));
	// 		F_Matrix(1, 1) = (1 + x_l_k(2, 0) * ctrv_delta_t * sin(x_l_k(3, 0) * DEC) / x_l_k(1, 0));
	// 	}
	// 	F_Matrix(2, 2) = 1;
	// 	F_Matrix(3, 3) = 1;
	// 	F_Matrix(4, 4) = 1;
	// 	F_Matrix(3, 4) = ctrv_delta_t;
	// 	F_Matrix(0, 1) = 0;
	// 	F_Matrix(0, 2) = 0;
	// 	F_Matrix(0, 3) = 0;
	// 	F_Matrix(0, 4) = 0;
	// 	F_Matrix(1, 0) = 0;
	// 	F_Matrix(1, 2) = 0;
	// 	F_Matrix(1, 3) = 0;
	// 	F_Matrix(1, 4) = 0;
	// 	F_Matrix(2, 0) = 0;
	// 	F_Matrix(2, 1) = 0;
	// 	F_Matrix(2, 3) = 0;
	// 	F_Matrix(2, 4) = 0;
	// 	F_Matrix(3, 0) = 0;
	// 	F_Matrix(3, 1) = 0;
	// 	F_Matrix(3, 2) = 0;
	// 	F_Matrix(4, 0) = 0;
	// 	F_Matrix(4, 1) = 0;
	// 	F_Matrix(4, 2) = 0;
	// 	F_Matrix(4, 3) = 0;

	// 	x_p_k = F_Matrix * x_l_k + B * U;
	// 	return x_p_k;
	// }

	// Eigen::MatrixXd MState::df_ctrv(Eigen::MatrixXd &x_l_k, Eigen::MatrixXd &U, double t)
	// {
	// 	(void) U;
	// 	double ctrv_delta_t = t;
	// 	Eigen::MatrixXd F_Matrix(Matrix_x, Matrix_x);

	// 	/*泰勒级数展开后的雅克比矩阵*/
	// 	if (x_l_k(4, 0) != 0)
	// 	{
	// 		F_Matrix(0, 0) = 1;
	// 		F_Matrix(0, 1) = 0;
	// 		F_Matrix(0, 2) = (-sin(x_l_k(3, 0) * DEC) +
	// 						  sin(ctrv_delta_t * x_l_k(4, 0) * DEC +
	// 							  x_l_k(3, 0) * DEC)) /
	// 						 x_l_k(4, 0);
	// 		F_Matrix(0, 3) = x_l_k(2, 0) * (-cos(x_l_k(3, 0) * DEC) + cos(ctrv_delta_t * x_l_k(4, 0) * DEC + x_l_k(3, 0) * DEC)) / x_l_k(4, 0);
	// 		F_Matrix(0, 4) = ctrv_delta_t * x_l_k(2, 0) / x_l_k(4, 0) * cos(ctrv_delta_t * x_l_k(4, 0) * DEC + x_l_k(3, 0) * DEC) -
	// 						 x_l_k(2, 0) / pow(x_l_k(4, 0), 2) *
	// 							 (-sin(x_l_k(3, 0) * DEC) +
	// 							  sin(ctrv_delta_t * x_l_k(4, 0) * DEC +
	// 								  x_l_k(3, 0) * DEC));
	// 		F_Matrix(1, 0) = 0;
	// 		F_Matrix(1, 1) = 1;
	// 		F_Matrix(1, 3) = x_l_k(2, 0) * (-sin(x_l_k(3, 0) * DEC) + sin(ctrv_delta_t * x_l_k(4, 0) * DEC + x_l_k(3, 0) * DEC)) / x_l_k(4, 0);
	// 		F_Matrix(1, 2) = (cos(x_l_k(3, 0) * DEC) -
	// 						  cos(ctrv_delta_t * x_l_k(4, 0) * DEC +
	// 							  x_l_k(3, 0) * DEC)) /
	// 						 x_l_k(4, 0);
	// 		F_Matrix(1, 4) = ctrv_delta_t * x_l_k(2, 0) / x_l_k(4, 0) * sin(ctrv_delta_t * x_l_k(4, 0) * DEC + x_l_k(3, 0) * DEC) - x_l_k(2, 0) /
	// 																																	pow(x_l_k(4, 0), 2) *
	// 																																	(cos(x_l_k(3, 0) * DEC) -
	// 																																	 cos(ctrv_delta_t * x_l_k(4, 0) * DEC +
	// 																																		 x_l_k(3, 0) * DEC));
	// 	}
	// 	else if (x_l_k(4, 0) == 0)
	// 	{
	// 		F_Matrix(0, 0) = 1;
	// 		F_Matrix(0, 1) = 0;
	// 		F_Matrix(0, 4) = 0;
	// 		F_Matrix(0, 2) = ctrv_delta_t * cos(x_l_k(3, 0) * DEC);
	// 		F_Matrix(0, 3) = -ctrv_delta_t * x_l_k(2, 0) *
	// 						 sin(x_l_k(3, 0) * DEC);
	// 		F_Matrix(1, 0) = 0;
	// 		F_Matrix(1, 1) = 1;
	// 		F_Matrix(1, 4) = 0;
	// 		F_Matrix(1, 2) = ctrv_delta_t * sin(x_l_k(3, 0) * DEC);
	// 		F_Matrix(1, 3) = ctrv_delta_t * x_l_k(2, 0) *
	// 						 cos(x_l_k(3, 0) * DEC);
	// 	}
	// 	F_Matrix(2, 0) = 0;
	// 	F_Matrix(2, 1) = 0;
	// 	F_Matrix(2, 2) = 1;
	// 	F_Matrix(2, 3) = 0;
	// 	F_Matrix(2, 4) = 0;
	// 	F_Matrix(3, 0) = 0;
	// 	F_Matrix(3, 1) = 0;
	// 	F_Matrix(3, 2) = 0;
	// 	F_Matrix(3, 3) = 1;
	// 	F_Matrix(3, 4) = ctrv_delta_t;
	// 	F_Matrix(4, 0) = 0;
	// 	F_Matrix(4, 1) = 0;
	// 	F_Matrix(4, 2) = 0;
	// 	F_Matrix(4, 3) = 0;
	// 	F_Matrix(4, 4) = 1;

	// 	return F_Matrix;
	// }

	// Eigen::MatrixXd MState::se_df_ctrv(Eigen::MatrixXd &x_l_k, Eigen::MatrixXd &U, double t)
	// {
	// 	(void) x_l_k;
	// 	(void) U;
	// 	double ctrv_delta_t = t;
	// 	(void) ctrv_delta_t;
	// 	Eigen::MatrixXd Matrix(Matrix_x, Matrix_x);
	// 	Matrix(0, 0) = 1;
	// 	Matrix(0, 1) = 0;
	// 	Matrix(0, 2) = 0;
	// 	Matrix(0, 3) = 0;
	// 	Matrix(0, 4) = 0;
	// 	Matrix(1, 0) = 0;
	// 	Matrix(1, 1) = 1;
	// 	Matrix(1, 2) = 0;
	// 	Matrix(1, 3) = 0;
	// 	Matrix(1, 4) = 0;
	// 	Matrix(2, 0) = 0;
	// 	Matrix(2, 1) = 0;
	// 	Matrix(2, 2) = 1;
	// 	Matrix(2, 3) = 0;
	// 	Matrix(2, 4) = 0;
	// 	Matrix(3, 0) = 0;
	// 	Matrix(3, 1) = 0;
	// 	Matrix(3, 2) = 0;
	// 	Matrix(3, 3) = 1;
	// 	Matrix(3, 4) = 0;
	// 	Matrix(4, 0) = 0;
	// 	Matrix(4, 1) = 0;
	// 	Matrix(4, 2) = 0;
	// 	Matrix(4, 3) = 0;
	// 	Matrix(4, 4) = 1;

	// 	return Matrix;
	// }

	// Eigen::MatrixXd MState::ctrv_sensor(Eigen::MatrixXd &x_p_k)
	// {
	// 	Eigen::MatrixXd Matrix(Matrix_y, Matrix_x);
	// 	Matrix(0, 0) = 1;
	// 	Matrix(0, 1) = 0;
	// 	Matrix(0, 2) = 0;
	// 	Matrix(0, 3) = 0;
	// 	Matrix(0, 4) = 0;
	// 	Matrix(1, 0) = 0;
	// 	Matrix(1, 1) = 1;
	// 	Matrix(1, 2) = 0;
	// 	Matrix(1, 3) = 0;
	// 	Matrix(1, 4) = 0;
	// 	Matrix = Matrix * x_p_k;

	// 	return Matrix;
	// }

	// Eigen::MatrixXd MState::df_ctrv_sensor(Eigen::MatrixXd &x_p_k)
	// {
	// 	(void) x_p_k;
	// 	Eigen::MatrixXd Matrix(Matrix_y, Matrix_x);
	// 	Matrix(0, 0) = 1;
	// 	Matrix(0, 1) = 0;
	// 	Matrix(0, 2) = 0;
	// 	Matrix(0, 3) = 0;
	// 	Matrix(0, 4) = 0;
	// 	Matrix(1, 0) = 0;
	// 	Matrix(1, 1) = 1;
	// 	Matrix(1, 2) = 0;
	// 	Matrix(1, 3) = 0;
	// 	Matrix(1, 4) = 0;

	// 	return Matrix;
	// }

	// Eigen::MatrixXd MState::se_df_ctrv_sensor(Eigen::MatrixXd &x_p_k)
	// {
	// 	(void) x_p_k;
	// 	Eigen::MatrixXd Matrix(Matrix_y, Matrix_y);
	// 	Matrix(0, 0) = 1;
	// 	Matrix(0, 1) = 0;
	// 	Matrix(1, 0) = 0;
	// 	Matrix(1, 1) = 1;
	// 	return Matrix;
	// }

}
