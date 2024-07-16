#include "dmp_ros2/preprocessing.hpp"

#include <iostream>
#include <iterator>

#include <range/v3/iterator/operations.hpp>

#include "dmplib/manifolds/aliases.hpp"
#include "dmplib/manifolds/se3_manifold.hpp"
#include "dmplib/utils/filter.hpp"
#include "dmplib/utils/numeric_differentiate.hpp"
#include "range/v3/all.hpp"
#include "range/v3/view/sliding.hpp"
#include "range/v3/view/transform.hpp"

namespace rs = ranges;
namespace rv = ranges::views;

using dmp_ros2::PosTraj_t;
using dmp_ros2::SE3;
using dmp_ros2::SegmentationProperties;
using dmp_ros2::Traj_t;

using TrajSample_t = dmp::StampedPosVelAccSample_t<SE3>;

double
position_velocity_norm(const TrajSample_t& sample) {
    return std::get<2>(sample).head<3>().norm();
}

PosTraj_t
dmp_ros2::rolling_mean(const PosTraj_t& traj, const std::vector<double>& weights) {
    auto posx_view = get_x_pos_view(traj);
    auto posy_view = get_y_pos_view(traj);
    auto posz_view = get_z_pos_view(traj);
    auto orix_view = get_x_quat_view(traj);
    auto oriy_view = get_y_quat_view(traj);
    auto oriz_view = get_z_quat_view(traj);
    auto oriw_view = get_w_quat_view(traj);

    auto posx_filt = dmp::utils::rolling_mean(posx_view, weights);
    auto posy_filt = dmp::utils::rolling_mean(posy_view, weights);
    auto posz_filt = dmp::utils::rolling_mean(posz_view, weights);
    auto orix_filt = dmp::utils::rolling_mean(orix_view, weights);
    auto oriy_filt = dmp::utils::rolling_mean(oriy_view, weights);
    auto oriz_filt = dmp::utils::rolling_mean(oriz_view, weights);
    auto oriw_filt = dmp::utils::rolling_mean(oriw_view, weights);

    PosTraj_t res(traj.size());
    for (std::size_t i = 0; i < traj.size(); i++) {
        std::get<0>(res[i])                 = std::get<0>(traj[i]);
        std::get<1>(res[i]).pos(0)          = posx_filt[i];
        std::get<1>(res[i]).pos(1)          = posy_filt[i];
        std::get<1>(res[i]).pos(2)          = posz_filt[i];
        std::get<1>(res[i]).ori.coeffs()(0) = orix_filt[i];
        std::get<1>(res[i]).ori.coeffs()(1) = oriy_filt[i];
        std::get<1>(res[i]).ori.coeffs()(2) = oriz_filt[i];
        std::get<1>(res[i]).ori.coeffs()(3) = oriw_filt[i];
        std::get<1>(res[i]).ori.normalize();
    }

    return res;
}

Traj_t
dmp_ros2::differentiate(const PosTraj_t& traj) {
    return dmp::utils::differentiate_twice(traj);
}

std::vector<bool>
dmp_ros2::filter_traj(
        const Traj_t&      traj,
        const double&      vel_th,
        const std::size_t& min_demonstration_length,
        const std::size_t& window_size,
        const std::size_t& stop_len
) {
    auto sliding_view =
            traj | rv::sliding(window_size)
            | rv::transform([vel_th](const auto& rng) -> bool {
                  return rs::max(rng | rv::transform(position_velocity_norm)) > vel_th;
              });

    std::size_t       to_add = 0;
    std::vector<bool> res;
    for (const bool& val : sliding_view) {
        res.push_back(val || to_add > 0);
        if (to_add > 0) to_add--;
        if (val) to_add = stop_len;
    }

    return res;
}

std::vector<Traj_t>
dmp_ros2::segment_trajectory(const Traj_t& traj, const SegmentationProperties& prop) {
    auto moving_parts = filter_traj(
            traj,
            prop.vel_th,
            prop.min_demonstration_length,
            prop.window_size,
            prop.stop_len
    );
    auto subtrajectories =
            rv::zip(moving_parts, traj)
            | rv::split_when([](const auto& pair) -> bool { return !rs::get<0>(pair); })
            | rv::filter(
                    [min = prop.min_demonstration_length](const auto& rng) -> bool {
                        return (rs::distance(rng) > min);
                    }
            );

    std::vector<Traj_t> res;
    for (const auto& subtraj : subtrajectories) {
        Traj_t tr = subtraj | rv::transform([](const auto& sample) -> TrajSample_t {
                        return rs::get<1>(sample);
                    })
                    | rs::to<Traj_t>;
        //if (rs::max(tr | rv::transform(position_velocity_norm)) < 5) res.push_back(tr);
        res.push_back(tr);
    }

    return res;
}
