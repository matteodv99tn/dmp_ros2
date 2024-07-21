#include "dmp_ros2/trajectory_listener.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <tf2/buffer_core.h>
#include <tf2/time.h>
#include <vector>

#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include "dmp_ros2/defines.hpp"
#include "dmp_ros2/demonstration_handler.hpp"
#include "dmp_ros2/dmp_filesystem.hpp"
#include "dmp_ros2/transformations.hpp"

using dmp_ros2::TrajectoryListener;

using DoubleVec_t = std::vector<double>;

TrajectoryListener::TrajectoryListener() :
        rclcpp::Node("trajectory_listener"),
        _traj({}),
        _first_timestamp(0.0),
        _pose_sub(nullptr),
        _tf_buff(get_clock()),
        _tf_listener(_tf_buff) {
    declare_parameter("input_topic", "/demonstrated_pose");
    declare_parameter("demonstration_folder", "");
    declare_parameter("translation", DoubleVec_t({0.0, 0.0, 0.0}));
    declare_parameter("rpy", DoubleVec_t({0.0, 0.0, 0.0}));

    const std::string topic = get_parameter("input_topic").as_string();

    auto on_pose_received = [this](const PoseMsg_t::ConstSharedPtr& msg) {
        dmp::TimeStamp_t ts = msg->header.stamp.nanosec + msg->header.stamp.sec * 1e9;
        if (_first_timestamp == 0) {
            _first_timestamp      = ts;
            _reference_frame_name = msg->header.frame_id;
        }

        if (_reference_frame_name != msg->header.frame_id) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Ref. frame of data changed from %s to %s",
                    _reference_frame_name.c_str(),
                    msg->header.frame_id.c_str()
            );
        }

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
    using namespace dmp_ros2::fs;
    using geometry_msgs::msg::TransformStamped;

    _pose_sub.reset();
    std::string out_folder = get_parameter("demonstration_folder").as_string();
    if (out_folder.empty()) out_folder = dmp_ros2::constants::tests_data_dir;

    const DoubleVec_t trans_data = get_parameter("translation").as_double_array();
    const DoubleVec_t rpy_data   = get_parameter("rpy").as_double_array();

    const Eigen::Vector3d    tool_transl{trans_data[0], trans_data[1], trans_data[2]};
    const Eigen::Quaterniond tool_rot =
            Eigen::AngleAxis(rpy_data[0], Eigen::Vector3d::UnitX())
            * Eigen::AngleAxis(rpy_data[1], Eigen::Vector3d::UnitY())
            * Eigen::AngleAxis(rpy_data[2], Eigen::Vector3d::UnitZ());
    RCLCPP_INFO(get_logger(), "Additional tool transform:");
    RCLCPP_INFO_STREAM(get_logger(), "  - translation: " << tool_transl.transpose());
    RCLCPP_INFO(get_logger(), "  - rotation: ");
    RCLCPP_INFO(get_logger(), "       roll: %.2lf deg", rpy_data[0] * 180 / M_PI);
    RCLCPP_INFO(get_logger(), "       pitch: %.2lf deg", rpy_data[1] * 180 / M_PI);
    RCLCPP_INFO(get_logger(), "       yaw: %.2lf deg", rpy_data[2] * 180 / M_PI);
    RCLCPP_INFO_STREAM(get_logger(), "       as quaterion: " << tool_rot);

    for (auto& sample : _traj) {
        const Eigen::Affine3d sample_transf = affine_from_se3(std::get<SE3>(sample));
        std::get<SE3>(sample).pos           = sample_transf * tool_transl;
        std::get<SE3>(sample).ori *= tool_rot;
    }

    Path_t               demonstration_path = future_demonstration_dir(out_folder);
    DemonstrationHandler handler(demonstration_path);

    RCLCPP_INFO(
            get_logger(),
            "Writing demonstration data into %s",
            handler.raw_data_file().c_str()
    );

    handler.write_raw_trajectory(_traj, _reference_frame_name);
    RCLCPP_INFO(get_logger(), "Exported %zu points", _traj.size());

    const std::vector<std::string>& frame_names = _tf_buff.getAllFrameNames();
    std::vector<DemonstrationHandler::RefFrameData_t> frames_data;

    for (const auto& frame_name : frame_names) {
        TransformStamped tf_transf =
                _tf_buff.lookupTransform("world", frame_name, tf2::TimePointZero);
        dmp::riemannmanifold::SE3 se3_transf;
        se3_transf.pos(0)  = tf_transf.transform.translation.x;
        se3_transf.pos(1)  = tf_transf.transform.translation.y;
        se3_transf.pos(2)  = tf_transf.transform.translation.z;
        se3_transf.ori.x() = tf_transf.transform.rotation.x;
        se3_transf.ori.y() = tf_transf.transform.rotation.y;
        se3_transf.ori.z() = tf_transf.transform.rotation.z;
        se3_transf.ori.w() = tf_transf.transform.rotation.w;
        frames_data.emplace_back(frame_name, se3_transf);
        RCLCPP_INFO(
                get_logger(),
                "Exporting transform relationship of rf '%s'",
                frame_name.c_str()
        );
    }
    handler.write_frames_transforms(frames_data);
}
