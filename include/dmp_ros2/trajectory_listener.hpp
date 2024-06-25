#ifndef DMPROS2_TRAJ_LISTENER_HPP
#define DMPROS2_TRAJ_LISTENER_HPP

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

#include "dmplib/manifolds/se3_manifold.hpp"

// clang-format off
// Disabled to avoid undesired include reordering
#include "dmplib/manifolds/aliases.hpp"

// clang-format on

namespace dmp_ros2 {

class TrajectoryListener : public rclcpp::Node {
public:
    using SE3                = dmp::riemannmanifold::SE3;
    using PoseTrajectory_t   = dmp::StampedPosTrajectory_t<SE3>;
    using TrajectorySample_t = dmp::StampedPosSample_t<SE3>;
    using PoseMsg_t          = geometry_msgs::msg::PoseStamped;

    TrajectoryListener();

    ~TrajectoryListener();

private:
    PoseTrajectory_t _traj;
    dmp::TimeStamp_t _first_timestamp;
    std::string      _reference_frame_name;

    rclcpp::Subscription<PoseMsg_t>::SharedPtr _pose_sub;
    tf2_ros::Buffer                            _tf_buff;
    tf2_ros::TransformListener                 _tf_listener;
};


}  // namespace dmp_ros2


#endif  // DMPROS2_TRAJ_LISTENER_HPP
