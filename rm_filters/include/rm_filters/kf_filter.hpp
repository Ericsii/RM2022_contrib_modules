#ifndef RM_FILTERS__KF_FILTER_HPP_
#define RM_FILTERS__KF_FILTER_HPP_

#include "rm_filters/filter_interface.hpp"

namespace rm_filters
{
    class KalmanFilter : public FilterInterface
    {
    public:
        /**
         * @brief 线性卡尔曼滤波器
         * 需要初始化的参数:
         *      A: 转移矩阵
         *      B: 控制矩阵
         *      H: 测量矩阵
         *      x_l_k: 初始状态
         *      Q: 过程模型协方差矩阵
         *      R: 测量模型协方差矩阵
         * @param dim_x 状态维度
         * @param dim_z 测量维度
         * @param dim_u 控制维度 default: 0
         */
        KalmanFilter(int dim_x, int dim_z, int dim_u = 0)
            : FilterInterface(dim_x, dim_z, dim_u)
        {
            A = Eigen::MatrixXd::Identity(dim_x, dim_x);
            if (dim_u > 0)
            {
                B = Eigen::MatrixXd::Zero(dim_x, dim_u);
            }
            else
            {
                B = Eigen::MatrixXd::Zero(dim_x, dim_x);
            }
        }

        /**
         * @brief 线性卡尔曼滤波器
         *
         * @param dim_x 状态维度
         * @param dim_z 测量维度
         * @param dim_u 控制维度 default: 0
         * @param Q 过程误差协方差矩阵 (dim_x, dim_x)
         * @param R 观测误差协方差矩阵 (dim_z, dim_z)
         */
        KalmanFilter(int dim_x, int dim_z, int dim_u, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R)
            : KalmanFilter(dim_x, dim_z, dim_u)
        {
            this->Q = Q;
            this->R = R;
        }

        void init(Eigen::VectorXd &x_k) override;
        Eigen::VectorXd predict(Eigen::VectorXd &u) override;
        
        // 线性卡尔曼滤波器中等价于 predict(Eigen::VectorXd &u)
        Eigen::VectorXd predict(Eigen::VectorXd &u, double t) override;
        Eigen::VectorXd update(Eigen::VectorXd &z_k) override;

    public:
        Eigen::MatrixXd B;
        Eigen::MatrixXd H;
    };
}

#endif // RM_FILTERS__KF_FILTER_HPP_