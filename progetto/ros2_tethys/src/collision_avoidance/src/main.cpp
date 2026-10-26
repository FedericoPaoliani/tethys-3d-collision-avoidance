#include "collision_avoidance/collision_avoidance_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}
