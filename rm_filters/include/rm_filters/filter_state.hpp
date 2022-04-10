#ifndef RM_FILTER_MOTION_STATE_HPP_
#define RM_FILTER_MOTION_STATE_HPP_
#include "rm_filters/filter_interface.hpp"

namespace rm_filters
{
    const double pi = M_PI; 
    const double DEC = pi / 180;

	class MState 
	{
	public:
		/**
		* @brief 匀加速运动模型
		*
		* @param x_l_k 上一时刻状态值
		* @param U 控制量
		* @return Mat
		*/
		static Eigen::MatrixXd const_acc(Eigen::MatrixXd &x_l_k, Eigen::MatrixXd &U, double t);
		/**
		* @brief 匀加速运动模型的状态矩阵
		*
		* @param x_l_k 上一时刻状态值
		* @param U 控制量
		* @return Mat
		*/
		static Eigen::MatrixXd df_const_acc(Eigen::MatrixXd &x_l_k, Eigen::MatrixXd &U, double t);
		/**
		* @brief 匀加速运动模型的协方差变化矩阵
		*
		* @param x_l_k 上一时刻状态值
		* @param U 控制量
		* @return Mat
		*/
		static Eigen::MatrixXd se_df_const_acc(Eigen::MatrixXd &x_l_k, Eigen::MatrixXd &U, double t);
		/**
		* @brief 传感器的运动模型
		*
		* @param x_p_k 预测的状态
		* @return Mat
		*/
		static Eigen::MatrixXd const_acc_sensor(Eigen::MatrixXd &x_p_k);
		/**
		* @brief 传感器的运动模型
		*
		* @param x_p_k 预测的状态
		* @return Mat
		*/
		static Eigen::MatrixXd df_const_acc_sensor(Eigen::MatrixXd &x_p_k);
		/**
		* @brief 传感器模型的协方差变化矩阵
		*
		* @param x_p_k 预测的状态
		* @return Mat
		*/
		static Eigen::MatrixXd se_df_const_acc_sensor(Eigen::MatrixXd &x_p_k);

		/**
		* @brief ctrv运动模型
		*
		* @param x_l_k 上一时刻状态值
		* @param U 控制量
		* @return Mat
		*/
		static Eigen::MatrixXd ctrv(Eigen::MatrixXd &x_l_k, Eigen::MatrixXd &U, double t);
		/**
		* @brief ctrv运动模型的状态矩阵
		*
		* @param x_l_k 上一时刻状态值
		* @param U 控制量
		* @return Mat
		*/
		static Eigen::MatrixXd df_ctrv(Eigen::MatrixXd &x_l_k, Eigen::MatrixXd &U, double t);
		/**
		* @brief ctrv运动模型的协方差变化矩阵
		*
		* @param x_l_k 上一时刻状态值
		* @param U 控制量
		* @return Mat
		*/
		static Eigen::MatrixXd se_df_ctrv(Eigen::MatrixXd &x_l_k, Eigen::MatrixXd &U, double t);
		/**
		* @brief 传感器的ctrv运动模型
		*
		* @param x_p_k 预测的状态
		* @return Mat
		*/
		static Eigen::MatrixXd ctrv_sensor(Eigen::MatrixXd &x_p_k);
		/**
		* @brief 传感器模型的运动状态矩阵
		*
		* @param x_p_k 预测的状态
		* @return Mat
		*/
		static Eigen::MatrixXd df_ctrv_sensor(Eigen::MatrixXd &x_p_k);
		/**
		* @brief 传感器模型的协方差变化矩阵
		*
		* @param x_p_k 预测的状态
		* @return Mat+
		*/
		static Eigen::MatrixXd se_df_ctrv_sensor(Eigen::MatrixXd &x_p_k);
	};
}

#endif