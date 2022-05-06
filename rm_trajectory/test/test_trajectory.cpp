#include "rm_trajectory/gravity_solver.hpp"
#include "rm_trajectory/database_solver.hpp"
#include <iostream>

int main()
{
	double pitch = 0, yaw = 0;
	Eigen::Vector3d position;
	position(0) = 0; position(1) = 3000; position(2) = 1000;
	//创建基于模型的求解器
	auto gravity_solver = std::make_shared<rm_trajectory::GravitySolver>(30, 0.03); 
	//创建基于数据的求解器
	auto database_solver = std::make_shared<rm_trajectory::DatabaseSolver>(30);
	//载入求解器
	auto trajectory_transform_tool =
		std::make_shared<rm_trajectory::TransformTool>(gravity_solver);
	//求解
	trajectory_transform_tool->solve(position, pitch, yaw);
	//获取求解后pitch补偿角度
	pitch = trajectory_transform_tool->pitch_;
	yaw = trajectory_transform_tool->yaw_;
	
	std::cout << pitch << std::endl;

	return 0;
}