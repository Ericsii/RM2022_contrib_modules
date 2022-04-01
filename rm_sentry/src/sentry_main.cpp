#include "rclcpp/rclcpp.hpp"
#include "rm_sentry/sentry_node.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_sentry::SentryNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
