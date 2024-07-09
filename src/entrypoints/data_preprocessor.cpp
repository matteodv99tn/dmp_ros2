#include "dmp_ros2/data_preprocessor.hpp"
#include <memory>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

int
main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin_some(std::make_shared<dmp_ros2::DataPreprocessor>());
    return 0;
}
