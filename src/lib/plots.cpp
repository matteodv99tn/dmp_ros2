#include "dmp_ros2/plots.hpp"

#include <string>
#include <vector>

#include "dmp_ros2/aliases.hpp"
#include "dmp_ros2/preprocessing.hpp"
#include "matplotlibcpp.h"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/transform.hpp"

namespace rs  = ranges;
namespace rv  = ranges::views;
namespace plt = matplotlibcpp;

void
dmp_ros2::plot_trajectory(const Traj_t& traj, const std::string& title) {
    using RealVec_t = std::vector<double>;

    const auto      time_view = get_time_view(traj);
    // const double    t0        = time_view[0];
    const double    t0        = 0;
    const RealVec_t time =
            time_view | rv::transform([t0](const auto& t) { return (t - t0) * 1e-9; })
            | rs::to_vector;


    const auto px_view = get_x_pos_view(traj);
    const auto py_view = get_y_pos_view(traj);
    const auto pz_view = get_z_pos_view(traj);

    const double px0 = px_view[0];
    const double py0 = py_view[0];
    const double pz0 = pz_view[0];

    const RealVec_t px = px_view
                         | rv::transform([px0](const auto& pt) { return pt - px0; })
                         | rs::to_vector;
    const RealVec_t py = py_view
                         | rv::transform([py0](const auto& pt) { return pt - py0; })
                         | rs::to_vector;
    const RealVec_t pz = pz_view
                         | rv::transform([pz0](const auto& pt) { return pt - pz0; })
                         | rs::to_vector;
    const RealVec_t qx = get_x_quat_view(traj) | rs::to_vector;
    const RealVec_t qy = get_y_quat_view(traj) | rs::to_vector;
    const RealVec_t qz = get_z_quat_view(traj) | rs::to_vector;
    const RealVec_t qw = get_w_quat_view(traj) | rs::to_vector;
    const RealVec_t vx = get_x_vel_view(traj) | rs::to_vector;
    const RealVec_t vy = get_y_vel_view(traj) | rs::to_vector;
    const RealVec_t vz = get_z_vel_view(traj) | rs::to_vector;

    plt::figure();

    plt::subplot(3, 1, 1);
    if (!title.empty()) plt::title(title);
    plt::named_plot("pos x", time, px);
    plt::named_plot("pos y", time, py);
    plt::named_plot("pos z", time, pz);
    plt::legend();

    plt::subplot(3, 1, 2);
    plt::named_plot("quat x", time, qx);
    plt::named_plot("quat y", time, qy);
    plt::named_plot("quat z", time, qz);
    plt::named_plot("quat w", time, qw);
    plt::legend();

    plt::subplot(3, 1, 3);
    plt::named_plot("vel x", time, vx);
    plt::named_plot("vel y", time, vy);
    plt::named_plot("vel z", time, vz);
    plt::legend();
}
