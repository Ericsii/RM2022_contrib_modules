#include "nahsor_modules/nahsor.hpp"

namespace nahsor
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    NahsorAlgo::NahsorAlgo(rclcpp::Node::SharedPtr node,
                           std::shared_ptr<rm_util::MonoMeasureTool> mono_location_tool)
        : node_(node), nahsor_detector(node), mono_location_tool_(mono_location_tool) {}
    int NahsorAlgo::smallPred(cv::Mat &img)
    {
        nahsor_detector.detect(img);
        auto armors = nahsor_detector.get_armors();
        auto circle = nahsor_detector.get_circle();
        if (armors.size() == 1)
        {
            auto armor = armors[0];
            //暂时假设顺时针转动
            target_angle = armor.angle - 10 * delta_t;
            target_x = circle.R_center.x + circle.radius * cos(target_angle);
            target_y = circle.R_center.y + circle.radius * sin(target_angle);
            //RCLCPP_INFO(node_->get_logger(), "x: %f  y: %f", circle.R_center.x, circle.R_center.y);
            //debug
            auto origin = img.clone();
            cv::circle(origin, cv::Point(target_x, target_y), 2, cv::Scalar(0, 0, 255));
            cv::circle(origin, cv::Point(circle.R_center.x, circle.R_center.y), 2, cv::Scalar(255, 0, 0));
            cv::imshow("result", origin);
            return 0;
        }
        return -1;
    }
    int NahsorAlgo::bigPred(cv::Mat &img, double t)
    {
        //RCLCPP_INFO(node_->get_logger(),"big");
        nahsor_detector.detect(img);
        //RCLCPP_INFO(node_->get_logger(), "%d", count);
        auto armors = nahsor_detector.get_armors();
        auto circle = nahsor_detector.get_circle();
        if (armors.size() == 1)
        {
            t -= temp_t;
            //RCLCPP_INFO(node_->get_logger(),"t:%f",t);
            auto armor = armors[0];
            if (count < 100)
            {
                if (!build_graph)
                {
                    this->graph();
                    last_angle = armor.angle;
                    temp_t = t;
                    last_time = 0;
                    build_graph = true;
                }
                else
                {

                    auto diff_angle = armor.angle - last_angle;
                    last_angle = armor.angle;
                    double temp = last_time;
                    last_time = t;
                    //角度变化过大，说明出现跳变
                    //舍弃一些不合理的值
                    if (abs(diff_angle) > 0.20)
                        return 0;
                    is_clockwise = diff_angle < 0;
                    auto velocity = abs(diff_angle) / (t - temp);
                    RCLCPP_INFO(node_->get_logger(), "%f  %f ", velocity, (t + temp) / 2);
                    this->addEdge(velocity, (t + temp) / 2);
                    target_x = circle.R_center.x;
                    target_y = circle.R_center.y;
                }
                count++;
            }
            else if (count == 100)
            {
                RCLCPP_INFO(node_->get_logger(), "开始优化");
                this->optimize();
                RCLCPP_INFO(node_->get_logger(), "%f,%f,%f,%f", a, w, t0, b);
                optimized = true;
                count++;
            }
            if (optimized)
            {

                //根据转动方向
                auto change_angle = -(a / w) * (cos(w * (t + t0 + delta_t))) +
                                    (a / w) * (cos(w * (t + t0))) + b * delta_t;
                RCLCPP_INFO(node_->get_logger(), "change: %f", change_angle);
                if (is_clockwise)
                    change_angle = -change_angle;

                target_angle = armor.angle + change_angle;
                //角度为0~2pi
                if (target_angle < 0)
                    target_angle += 2 * CV_PI;
                if (target_angle > 2 * CV_PI)
                    target_angle -= 2 * CV_PI;
                RCLCPP_INFO(node_->get_logger(), "x: %f, y: %f, R_x: %f, R_y: %f ", armor.center.x, armor.center.y, circle.R_center.x, circle.R_center.y);
                RCLCPP_INFO(node_->get_logger(), "观测值：angle: %f,  %f", armor.angle, t);
                RCLCPP_INFO(node_->get_logger(), "预测值：x: %f, y: %f, target_angle: %f, t: %f", target_x, target_y, target_angle, t + delta_t);
                target_x = circle.R_center.x + circle.radius * cos(target_angle);
                target_y = circle.R_center.y + circle.radius * sin(target_angle);
                count++;
                if (count % 1000 == 0)
                {
                    count = 0;
                    optimizer.removeVertex(v);
                    RCLCPP_INFO(node_->get_logger(), "开始新一轮优化");
                    build_graph = false;
                }
                //debug
                auto origin = img.clone();
                cv::circle(origin, cv::Point(target_x, target_y), 10, cv::Scalar(0, 0, 255));
                cv::circle(origin, cv::Point(armor.center.x, armor.center.y), 10, cv::Scalar(255, 0, 0));
                cv::circle(origin, cv::Point(circle.R_center.x, circle.R_center.y), 10, cv::Scalar(0, 255, 0));
                cv::imshow("result", origin);
                cv::waitKey(1);
            }
            return 0;
        }
        return -1;
    }
    int NahsorAlgo::noPred(cv::Mat &img)
    {
        nahsor_detector.detect(img);
        auto armors = nahsor_detector.get_armors();
        if (armors.size() == 1)
        {
            auto armor = armors[0];
            target_x = armor.center.x;
            target_y = armor.center.y;
            target_angle = armor.angle;
            return 0;
        }
        return -1;
    }
    void NahsorAlgo::graph()
    {

        auto solver = new g2o::OptimizationAlgorithmGaussNewton(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(true);
        v = new CurveFittingVertex();
        v->setEstimate(Eigen::Vector3d(1.045, 1.884, 0));
        v->setId(0);
        optimizer.addVertex(v);
    }
    void NahsorAlgo::addEdge(double velocity, double time)
    {
        double w_sigma = 1.0;
        CurveFittingEdge *edge = new CurveFittingEdge(time);
        edge->setId(1);
        edge->setVertex(0, v);
        edge->setMeasurement(velocity);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));
        optimizer.addEdge(edge);
    }
    void NahsorAlgo::optimize()
    {
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        auto res = v->estimate();
        a = res[0];
        w = res[1];
        t0 = res[2];
        b = 1.4 - a;
    }

    cv::Point2f NahsorAlgo::get_target()
    {
        return {target_x, target_y}; //z轴为固定值
    }

    cv::Point2f NahsorAlgo::get_R_center()
    {
        return nahsor_detector.get_circle().R_center;
    }
    double NahsorAlgo::get_angle()
    {
        return target_angle;
    }
}