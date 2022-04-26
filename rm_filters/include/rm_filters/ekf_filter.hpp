#ifndef RM_FILTER_EKF_FILTER_HPP_
#define RM_FILTER_EKF_FILTER_HPP_
#include "rm_filters/filter_interface.hpp"
#include "rm_filters/filter_state.hpp"

namespace rm_filters
{
 
    class ExKalmanFilter : public FilterInterface
    {
    public:
        /**
         * @brief 扩展卡尔曼滤波器
         * 
         * @param dim_x 状态维度
         * @param dim_z 测量维度
         * @param dim_u 控制维度 default: 0
         */
        ExKalmanFilter(int dim_x, int dim_z, int dim_u = 0)
            : FilterInterface(dim_x, dim_z, dim_u)
        {
            A = Eigen::MatrixXd::Identity(dim_x, dim_x);
        }

        /**
         * @brief 扩展卡尔曼滤波器
         * 
         * @param dim_x 状态维度
         * @param dim_z 测量维度
         * @param dim_u 控制维度
         * @param Q     过程误差协方差矩阵 (dim_x, dim_x)
         * @param R     观测误差协方差矩阵 (dim_z, dim_z)
         */
        ExKalmanFilter(int dim_x, int dim_z, int dim_u, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R)
            : ExKalmanFilter(dim_x, dim_z, dim_u)
        {
            this->Q = Q;
            this->R = R;
        }

        /**
         * @brief 扩展卡尔曼滤波器
         * 
         * @param dim_x             状态维度
         * @param dim_z             测量维度
         * @param dim_u             控制维度
         * @param Q                 过程误差协方差矩阵 (dim_x, dim_x)
         * @param R                 观测误差协方差矩阵 (dim_z, dim_z)
         * @param state_func        状态转移函数
         * @param state_df_func     状态转移一阶微分函数
         * @param state_ddf_func    状态转移二阶微分函数
         * @param sensor_func       观测函数
         * @param sensor_df_func    观测一阶微分函数
         * @param sensor_ddf_func   观测二阶微分函数
         */
        ExKalmanFilter(int dim_x, int dim_z, int dim_u, 
                       const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                       kinestate_function state_func,
                       kinestate_function state_df_func,
                       kinestate_function state_ddf_func,
                       sensor_function sensor_func,
                       sensor_function sensor_df_func,
                       sensor_function sensor_ddf_func)
            : ExKalmanFilter(dim_x, dim_z, dim_u, Q, R)
        {
            this->base_state = state_func;
            this->df_state = state_df_func;
            this->se_df_state = state_ddf_func;
            this->base_sensor = sensor_func;
            this->df_sensor = sensor_df_func;
            this->se_df_sensor = sensor_ddf_func;
        }

        void init(Eigen::VectorXd &x_k) override;
        Eigen::VectorXd predict(Eigen::VectorXd &u) override;
        Eigen::VectorXd predict(Eigen::VectorXd &u, double t) override;
        Eigen::VectorXd update(Eigen::VectorXd &z_k) override;

    public:
        Eigen::MatrixXd W;
        Eigen::MatrixXd H;
        Eigen::MatrixXd V;

    public:
        kinestate_function base_state;      // 状态转移函数
        kinestate_function df_state;        // 状态转移一阶导数 jacobian
        kinestate_function se_df_state;     // 状态转移二阶导数 hessian
        sensor_function base_sensor;        // 观测函数
        sensor_function df_sensor;          // 观测一阶导数 jacobian
        sensor_function se_df_sensor;       // 观测二阶导数 hessian

    };
}

#endif //RM_FILTER_EKF_FILTER_HPP_