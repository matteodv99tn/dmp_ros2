#include "dmp_ros2/trajectory_listener.hpp"

#include <rclcpp/rclcpp.hpp>

int
main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dmp_ros2::TrajectoryListener>());
    rclcpp::shutdown();
    return 0;
}
