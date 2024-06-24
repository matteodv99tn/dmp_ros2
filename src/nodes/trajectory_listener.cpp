#include "dmp_ros2/trajectory_listener.hpp"

#include <filesystem>
#include <string>

#include <rclcpp/node.hpp>

#include "dmp_ros2/defines.hpp"
#include "dmp_ros2/defines.hpp.in"
#include "dmplib/data_handler/conversions.hpp"


using dmp_ros2::TrajectoryListener;

TrajectoryListener::TrajectoryListener() :
        rclcpp::Node("trajectory_listener"),
        _traj({}),
        _first_timestamp(0.0),
        _pose_sub(nullptr) {
    declare_parameter("input_topic", "/demonstrated_pose");
    declare_parameter("output_folder", "");

    const std::string topic = get_parameter("input_topic").as_string();

    auto on_pose_received = [this](const PoseMsg_t::ConstSharedPtr& msg) {
        dmp::TimeStamp_t ts = msg->header.stamp.nanosec + msg->header.stamp.sec * 1e9;
        if (_first_timestamp == 0) _first_timestamp = ts;

        SE3 pose;
        pose.pos.x() = msg->pose.position.x;
        pose.pos.y() = msg->pose.position.y;
        pose.pos.z() = msg->pose.position.z;
        pose.ori.x() = msg->pose.orientation.x;
        pose.ori.y() = msg->pose.orientation.y;
        pose.ori.z() = msg->pose.orientation.z;
        pose.ori.w() = msg->pose.orientation.w;

        _traj.emplace_back(ts - _first_timestamp, pose);
    };
    _pose_sub =
            create_subscription<PoseMsg_t>(topic, rclcpp::QoS(10), on_pose_received);
    RCLCPP_INFO(get_logger(), "Listening trajectory from topic %s", topic.c_str());
};

TrajectoryListener::~TrajectoryListener() {
    _pose_sub.reset();
    std::string out_folder = get_parameter("output_folder").as_string();
    if (out_folder.empty()) out_folder = dmp_ros2::constants::tests_data_dir;

    std::filesystem::path folder_path(out_folder);
    std::filesystem::path file_path = folder_path / "demonstration.txt";

    if (!std::filesystem::exists(folder_path))
        std::filesystem::create_directory(folder_path);

    RCLCPP_INFO(get_logger(), "Writing demonstration into %s", file_path.c_str());

    dmp::to::file(file_path, _traj);
}
