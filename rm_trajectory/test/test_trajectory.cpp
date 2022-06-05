#include "rm_trajectory/gravity_solver.hpp"
#include "rm_trajectory/database_solver.hpp"
#include "rm_trajectory/gravity_nofriction_solver.hpp"
#include "gtest/gtest.h"

#include <iostream>

// TEST(trajectory, solver)
// {
// 	double pitch = 0, yaw = 0;
// 	double correction = 2.5, init_pitch = -3.2;
// 	Eigen::Vector3d position;
// 	//position为以云台为坐标原点的目标坐标点xyz，单位为cm
// 	position(0) = 0; position(1) = 300; position(2) = 100;
// 	//创建基于模型的求解器,第一个参数为射速，第二个参数为空气阻力系数
// 	auto gravity_solver = std::make_shared<rm_trajectory::GravitySolver>(30, 0.03); 
// 	//创建基于模型的求解器,参数为射速
// 	auto gravity_nofriction_solver = std::make_shared<rm_trajectory::GravityNofrictionSolver>(30);
// 	//创建基于数据的求解器,参数为射速
// 	auto database_solver = std::make_shared<rm_trajectory::DatabaseSolver>(30);
// 	//载入求解器
// 	auto trajectory_transform_tool =
// 		std::make_shared<rm_trajectory::TransformTool>(database_solver);
// 	//求解
// 	trajectory_transform_tool->solve(position, pitch, yaw);
// 	//获取求解后pitch补偿角度，绝对角度，
// 	//correction为不同兵种的弹道补偿值，需要根据实际情况调整，使用数据database求解器不需要加
// 	//init_pitch为零飘角度，需实际测试
// 	pitch = pitch + correction + init_pitch;
// 	std::cout << pitch << std::endl;

// 	EXPECT_TRUE(true);
// }
