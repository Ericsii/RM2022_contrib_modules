#include "include/rm_filters/ekf_filter.hpp"
#include "include/rm_filters/filter_state.hpp"

using namespace rm_filters;

int main() {

    Eigen::MatrixXd Q(Matrix_x, Matrix_x); // 先验估计的协方差矩阵，Q，主要调节对角线元素
	Q(0, 0) = 1; Q(0, 1) = 0;	
	Q(0, 2) = 0; Q(0, 3) = 0;  Q(0, 4) = 0;
	Q(1, 0) = 0; Q(1, 1) = 1;	
	Q(1, 2) = 0; Q(1, 3) = 0;  Q(1, 4) = 0;
	Q(2, 0) = 0; Q(2, 1) = 0;	
	Q(2, 2) = 1; Q(2, 3) = 0;  Q(2, 4) = 0;
	Q(3, 0) = 0; Q(3, 1) = 0;	
	Q(3, 2) = 0; Q(3, 3) = 1;  Q(3, 4) = 0;
	Q(4, 0) = 0; Q(4, 1) = 0;   
	Q(4, 2) = 0; Q(4, 3) = 0;  Q(4, 4) = 1;

    Eigen::MatrixXd R(Matrix_y, Matrix_y); // 传感器协方差矩阵，R，根据测试到数据的误差量决定
	R(0, 0) = 1, R(0, 1) = 0, R(0, 2) = 0;
    R(1, 0) = 0, R(1, 1) = 1, R(1, 2) = 0;
    R(2, 0) = 0, R(2, 1) = 0, R(2, 2) = 1;

    Eigen::MatrixXd U(Matrix_x, Matrix_x);
    U = Eigen::MatrixXd::Zero(Matrix_x, Matrix_x); // 控制量，U，只改变对角线元素，对于匀加速模型，U即为加速度

    Eigen::MatrixXd P(Matrix_x, Matrix_x);
    P = Eigen::MatrixXd::Zero(Matrix_x, Matrix_x);  // 协方差矩阵，初始化为0

    Eigen::MatrixXd x_l_k(Matrix_x, 1);
    Eigen::MatrixXd x_p_k(Matrix_x, 1);
    Eigen::MatrixXd z_k(Matrix_y, 1);

    x_l_k(0, 0) = 1, x_l_k(1, 0) = 1, x_l_k(2, 0) = 1,
    x_l_k(3, 0) = 0.5, x_l_k(4, 0) = 0.5;             // 初始状态量

    x_p_k = x_l_k; //初始化时，先验估计即为当前状态量

    z_k(0, 0) = 2, z_k(1, 0) = 2, z_k (2, 0) = 2; //传感器数据，x，y，z的值。

    Filters * ekf_filter = new ExKalman(x_p_k, x_l_k, Q, P, R,
    MState::const_acc, MState::df_const_acc, MState::se_df_const_acc,
    MState::const_acc_sensor, MState::df_const_acc_sensor, MState::se_df_const_acc_sensor);
    
    double t = 1; //参考帧率的时间，即每进行一次滤波的时间
    ekf_filter->predict(U, t);
    x_l_k = ekf_filter->update(z_k);

    return 0;
}