#ifndef DMPROS2_PREPROCESSING_TOOLS_HPP
#define DMPROS2_PREPROCESSING_TOOLS_HPP

#include <cstddef>
#include <vector>

#include "dmp_ros2/aliases.hpp"
#include "dmplib/manifolds/aliases.hpp"
#include "dmplib/manifolds/se3_manifold.hpp"
#include "range/v3/view/transform.hpp"

namespace dmp_ros2 {

struct SegmentationProperties {
    double      vel_th{0.2};
    std::size_t min_demonstration_length{100};
    std::size_t window_size{20};
    std::size_t stop_len{40};
};

PosTraj_t rolling_mean(const PosTraj_t& traj, const std::vector<double>& weights);

Traj_t differentiate(const PosTraj_t& traj);


std::vector<bool> filter_traj(
        const Traj_t&      traj,
        const double&      vel_th,
        const std::size_t& min_demonstration_length,
        const std::size_t& window_size,
        const std::size_t& stop_len
);

std::vector<Traj_t> segment_trajectory(
        const Traj_t& traj, const SegmentationProperties& prop
);

namespace internal {

    namespace rv = ::ranges::views;
    using SE3    = ::dmp::riemannmanifold::SE3;

    template <typename Traj>
    [[nodiscard]] inline auto
    construct_pos_view(const Traj& traj, const int id) {
        return traj | rv::transform([id](const auto& sample) -> double {
                   return std::get<SE3>(sample).pos(id);
               });
    };

    template <typename Traj>
    [[nodiscard]] inline auto
    construct_ori_view(const Traj& traj, const int id) {
        return traj | rv::transform([id](const auto& sample) -> double {
                   return std::get<SE3>(sample).ori.coeffs()(id);
               });
    };

    template <typename Traj>
    [[nodiscard]] inline auto
    construct_vel_view(const Traj& traj, const int id) {
        return traj | rv::transform([id](const auto& sample) -> double {
                   return std::get<2>(sample)(id);
               });
    };

    template <typename Traj>
    [[nodiscard]] inline auto
    construct_acc_view(const Traj& traj, const int id) {
        return traj | rv::transform([id](const auto& sample) -> double {
                   return std::get<3>(sample)(id);
               });
    };
}  // namespace internal

template <typename Traj>
[[nodiscard]] inline auto
get_time_view(const Traj& traj) {
    return traj | internal::rv::transform([](const auto& sample) -> double {
               return std::get<dmp::TimeStamp_t>(sample);
           });
}

template <typename Traj>
[[nodiscard]] inline auto
get_x_pos_view(const Traj& traj) {
    return internal::construct_pos_view(traj, 0);
}

template <typename Traj>
[[nodiscard]] inline auto
get_y_pos_view(const Traj& traj) {
    return internal::construct_pos_view(traj, 1);
}

template <typename Traj>
[[nodiscard]] inline auto
get_z_pos_view(const Traj& traj) {
    return internal::construct_pos_view(traj, 2);
}

template <typename Traj>
[[nodiscard]] inline auto
get_x_quat_view(const Traj& traj) {
    return internal::construct_ori_view(traj, 0);
}

template <typename Traj>
[[nodiscard]] inline auto
get_y_quat_view(const Traj& traj) {
    return internal::construct_ori_view(traj, 1);
}

template <typename Traj>
[[nodiscard]] inline auto
get_z_quat_view(const Traj& traj) {
    return internal::construct_ori_view(traj, 2);
}

template <typename Traj>
[[nodiscard]] inline auto
get_w_quat_view(const Traj& traj) {
    return internal::construct_ori_view(traj, 3);
}

template <typename Traj>
[[nodiscard]] inline auto
get_x_vel_view(const Traj& traj) {
    return internal::construct_vel_view(traj, 0);
}

template <typename Traj>
[[nodiscard]] inline auto
get_y_vel_view(const Traj& traj) {
    return internal::construct_vel_view(traj, 1);
}

template <typename Traj>
[[nodiscard]] inline auto
get_z_vel_view(const Traj& traj) {
    return internal::construct_vel_view(traj, 2);
}

template <typename Traj>
[[nodiscard]] inline auto
get_x_omega_view(const Traj& traj) {
    return internal::construct_vel_view(traj, 3);
}

template <typename Traj>
[[nodiscard]] inline auto
get_y_omega_view(const Traj& traj) {
    return internal::construct_vel_view(traj, 4);
}

template <typename Traj>
[[nodiscard]] inline auto
get_z_omega_view(const Traj& traj) {
    return internal::construct_vel_view(traj, 5);
}

template <typename Traj>
[[nodiscard]] inline auto
get_x_acc_view(const Traj& traj) {
    return internal::construct_acc_view(traj, 0);
}

template <typename Traj>
[[nodiscard]] inline auto
get_y_acc_view(const Traj& traj) {
    return internal::construct_acc_view(traj, 1);
}

template <typename Traj>
[[nodiscard]] inline auto
get_z_acc_view(const Traj& traj) {
    return internal::construct_acc_view(traj, 2);
}

template <typename Traj>
[[nodiscard]] inline auto
get_x_angular_acc_view(const Traj& traj) {
    return internal::construct_acc_view(traj, 3);
}

template <typename Traj>
[[nodiscard]] inline auto
get_y_angular_acc_view(const Traj& traj) {
    return internal::construct_acc_view(traj, 4);
}

template <typename Traj>
[[nodiscard]] inline auto
get_z_angular_acc_view(const Traj& traj) {
    return internal::construct_acc_view(traj, 5);
}

}  // namespace dmp_ros2


#endif  // DMPROS2_PREPROCESSING_TOOLS_HPP
