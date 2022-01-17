/*
 * @Author: your name
 * @Date: 2022-01-16 16:13:29
 * @LastEditTime: 2022-01-17 15:36:17
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /scu_rm_ros/contrib_modules/rm_filters/include/rm_filters/filter_interface.hpp
 */
#ifndef RM_FILTERS__FILTER_INTERFACE_HPP_
#define RM_FILTERS__FILTER_INTERFACE_HPP_

#include <cmath>
#include <string>
#include <iostream>
#include <limits>
#include <Eigen/Dense>

namespace rm_filters
{
	const u_int32_t Matrix_x = 6;
	const u_int32_t Matrix_y = 3;
	
	class Filters
	{
	public:
		using kinestate =
			std::function<Eigen::MatrixXd(Eigen::MatrixXd &x_l_k, Eigen::MatrixXd &U, double t)>;
		using sensor =
			std::function<Eigen::MatrixXd(Eigen::MatrixXd &x_p_k)>;

	public:
		/**
		* @brief 滤波的更新部分
		*
		* @param z_k 传感器的值
		* @return Mat
		*/
		virtual void init(Eigen::MatrixXd &z_k) = 0;
		/**
		 * @brief 滤波的预测部分
		 *
		 * @param U 控制量
		 * @return Mat
		 */
		virtual Eigen::MatrixXd predict(Eigen::MatrixXd &U, double t) = 0;

		/**
		* @brief 滤波的更新部分
		*
		* @param z_k 传感器的值
		* @return Mat
		*/
		virtual Eigen::MatrixXd update(Eigen::MatrixXd &z_k) = 0;

	};
}

#endif // RM_FILTERS__FILTER_INTERFACE_HPP_