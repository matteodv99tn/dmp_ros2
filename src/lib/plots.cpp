#include "dmp_ros2/plots.hpp"

#include <string>
#include <vector>

#include "dmp_ros2/aliases.hpp"
#include "dmp_ros2/preprocessing.hpp"
#include "dmp_ros2/se3_dmp.hpp"
#include "matplotlibcpp.h"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/transform.hpp"

namespace rs  = ranges;
namespace rv  = ranges::views;
namespace plt = matplotlibcpp;

using dmp_ros2::Se3Dmp_t;

void
dmp_ros2::plot_trajectory(const Traj_t& traj, const std::string& title) {
    using RealVec_t = std::vector<double>;

    const auto time_view = get_time_view(traj);
    // const double    t0        = time_view[0];
    const double    t0 = 0;
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
    const RealVec_t ox = get_x_omega_view(traj) | rs::to_vector;
    const RealVec_t oy = get_y_omega_view(traj) | rs::to_vector;
    const RealVec_t oz = get_z_omega_view(traj) | rs::to_vector;

    plt::figure();

    plt::subplot(4, 1, 1);
    plt::title("Position");
    if (!title.empty()) plt::title(title);
    plt::named_plot("pos x", time, px);
    plt::named_plot("pos y", time, py);
    plt::named_plot("pos z", time, pz);
    plt::legend();

    plt::subplot(4, 1, 2);
    plt::title("Orientation");
    plt::named_plot("quat x", time, qx);
    plt::named_plot("quat y", time, qy);
    plt::named_plot("quat z", time, qz);
    plt::named_plot("quat w", time, qw);
    plt::legend();

    plt::subplot(4, 1, 3);
    plt::title("Linear velocity");
    plt::named_plot("vel x", time, vx);
    plt::named_plot("vel y", time, vy);
    plt::named_plot("vel z", time, vz);
    plt::legend();

    plt::subplot(4, 1, 4);
    plt::title("Angular velocity");
    plt::named_plot("omega x", time, ox);
    plt::named_plot("omega y", time, oy);
    plt::named_plot("omega z", time, oz);
    plt::legend();
}

void
dmp_ros2::plot_trajectory_comparison(const Traj_t& demonstration, Se3Dmp_t& dmp) {
    using RealVec_t = std::vector<double>;

    const auto      time_view = get_time_view(demonstration);
    const double    t0        = time_view[0];
    const RealVec_t dem_time =
            time_view | rv::transform([t0](const auto& t) { return (t - t0) * 1e-9; })
            | rs::to_vector;
    const double tf = dem_time.back();

    const Traj_t reconstructed = dmp.integrate_trajectory(
            demonstration.front(), demonstration.back(), tf, 0.002, 1.1 * tf
    );

    const RealVec_t rec_time = get_time_view(reconstructed)
                               | rv::transform([](const auto& t) { return t * 1e-9; })
                               | rs::to_vector;


    const RealVec_t dem_px = get_x_pos_view(demonstration) | rs::to_vector;
    const RealVec_t dem_py = get_y_pos_view(demonstration) | rs::to_vector;
    const RealVec_t dem_pz = get_z_pos_view(demonstration) | rs::to_vector;
    const RealVec_t dem_qx = get_x_quat_view(demonstration) | rs::to_vector;
    const RealVec_t dem_qy = get_y_quat_view(demonstration) | rs::to_vector;
    const RealVec_t dem_qz = get_z_quat_view(demonstration) | rs::to_vector;
    const RealVec_t dem_qw = get_w_quat_view(demonstration) | rs::to_vector;

    const RealVec_t rec_px = get_x_pos_view(reconstructed) | rs::to_vector;
    const RealVec_t rec_py = get_y_pos_view(reconstructed) | rs::to_vector;
    const RealVec_t rec_pz = get_z_pos_view(reconstructed) | rs::to_vector;
    const RealVec_t rec_qx = get_x_quat_view(reconstructed) | rs::to_vector;
    const RealVec_t rec_qy = get_y_quat_view(reconstructed) | rs::to_vector;
    const RealVec_t rec_qz = get_z_quat_view(reconstructed) | rs::to_vector;
    const RealVec_t rec_qw = get_w_quat_view(reconstructed) | rs::to_vector;

    plt::figure();

    plt::subplot(4, 1, 1);
    plt::title("Position x");
    plt::plot(dem_time, dem_px, "r.");
    plt::plot(rec_time, rec_px, "r--");

    plt::subplot(4, 1, 2);
    plt::title("Position y");
    plt::plot(dem_time, dem_py, "g.");
    plt::plot(rec_time, rec_py, "g--");

    plt::subplot(4, 1, 3);
    plt::title("Position z");
    plt::plot(dem_time, dem_pz, "b.");
    plt::plot(rec_time, rec_pz, "b--");

    plt::subplot(4, 1, 4);
    plt::title("Orientation");
    plt::plot(dem_time, dem_qx, "r.");
    plt::plot(rec_time, rec_qx, "r--");
    plt::plot(dem_time, dem_qy, "g.");
    plt::plot(rec_time, rec_qy, "g--");
    plt::plot(dem_time, dem_qz, "b.");
    plt::plot(rec_time, rec_qz, "b--");
    plt::plot(dem_time, dem_qw, "k.");
    plt::plot(rec_time, rec_qw, "k--");
}
