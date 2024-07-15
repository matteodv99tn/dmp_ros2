#include "dmp_ros2/data_preprocessor.hpp"

#include <cstddef>
#include <iostream>
#include <stdexcept>

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logging.hpp>

#include "dmp_ros2/defines.hpp"
#include "dmp_ros2/demonstration_handler.hpp"
#include "dmp_ros2/dmp_filesystem.hpp"
#include "dmp_ros2/plots.hpp"
#include "dmp_ros2/preprocessing.hpp"
#include "dmp_ros2/se3_dmp.hpp"
#include "dmplib/utils/filter.hpp"
#include "fmt/format.h"
#include "matplotlibcpp.h"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/transform.hpp"
namespace plt = matplotlibcpp;

using dmp_ros2::DataPreprocessor;
using namespace dmp_ros2::fs;

namespace params {
constexpr char dem_dir[]     = "demonstrations_directory";
constexpr char filt_w[]      = "filter_weights";
constexpr char vel_th[]      = "segmentation.velocity_th";
constexpr char min_dem_len[] = "segmentation.minimum_window_length";
constexpr char win_sz[]      = "segmentation.window_size";
constexpr char tail_sz[]     = "segmentation.tail_size";
constexpr char dmp_basis[]   = "n_basis";
}  // namespace params

void
DataPreprocessor::initialise_parameters() {
    declare_parameter(params::dem_dir, dmp_ros2::constants::tests_data_dir);
    declare_parameter(params::filt_w, std::vector<double>({2.0, 1.0, 1.0, 0.5}));
    declare_parameter(params::vel_th, 0.2);
    declare_parameter(params::min_dem_len, 200);
    declare_parameter(params::win_sz, 40);
    declare_parameter(params::tail_sz, 60);
    declare_parameter(params::dmp_basis, int(15));
}

DataPreprocessor::DataPreprocessor() : rclcpp::Node("dmpros2_data_preprocessor") {
    initialise_parameters();
    process_data();
}

void
DataPreprocessor::process_data() const {
    const std::string dem_dir = get_parameter(params::dem_dir).as_string();

    auto dir = fs::latest_demonstration_dir(dem_dir);
    if (!dir.has_value()) {
        const std::string msg = fmt::format(
                "Unable to find demonstrations in the folder '{}'", dem_dir
        );
        RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
        throw std::runtime_error(msg.c_str());
    }
    fs::DemonstrationHandler handler(dir.value());

    RCLCPP_INFO(
            get_logger(),
            "Loading trajectory from file %s",
            handler.raw_data_file().c_str()
    );
    auto traj = handler.load_raw_trajectory();

    RCLCPP_INFO(
            get_logger(),
            "Applying non-causal moving average filters on loaded data with "
            "coefficients"
    );
    const std::vector<double> filter = get_parameter(params::filt_w).as_double_array();
    for (const double& v : filter) RCLCPP_INFO(get_logger(), "  - %f", v);
    const auto filtered_traj =
            dmp_ros2::rolling_mean(dmp::utils::remove_duplicates(traj), filter);

    RCLCPP_INFO(get_logger(), "Differentiating numerically the trajectory");
    const auto full_traj = dmp_ros2::differentiate(filtered_traj);

    RCLCPP_INFO(get_logger(), "Performing trajectory segmentation");
    const double      vel_th      = get_parameter(params::vel_th).as_double();
    const std::size_t min_dem_len = get_parameter(params::min_dem_len).as_int();
    const std::size_t win_sz      = get_parameter(params::win_sz).as_int();
    const std::size_t tail_sz     = get_parameter(params::tail_sz).as_int();
    RCLCPP_INFO(get_logger(), "  - velocity threshold: %f", vel_th);
    RCLCPP_INFO(get_logger(), "  - minimum demonstration length: %zu", min_dem_len);
    RCLCPP_INFO(get_logger(), "  - window size: %zu", win_sz);
    RCLCPP_INFO(get_logger(), "  - tail size: %zu", tail_sz);

    const auto subtrajs = dmp_ros2::segment_trajectory(
            full_traj, vel_th, min_dem_len, win_sz, tail_sz
    );
    RCLCPP_INFO(get_logger(), "Found %zu trajectories", subtrajs.size());

    RCLCPP_INFO(
            get_logger(),
            "Writing segmented trajectories into directory %s",
            handler.demonstration_data_dir_path().c_str()
    );
    handler.write_demonstrated_trajectories(subtrajs);

    const std::size_t nb = get_parameter(params::dmp_basis).as_int();
    RCLCPP_INFO(get_logger(), "Computing dmp...");
    RCLCPP_INFO(get_logger(), "Using %zu basis functions", nb);

    auto traj_original = subtrajs.front();
    auto tr            = traj_original;

#if 1
    auto vlx_view = get_x_vel_view(traj_original);
    auto vly_view = get_y_vel_view(traj_original);
    auto vlz_view = get_z_vel_view(traj_original);
    auto vax_view = get_x_omega_view(traj_original);
    auto vay_view = get_y_omega_view(traj_original);
    auto vaz_view = get_z_omega_view(traj_original);

    auto alx_view = get_x_acc_view(traj_original);
    auto aly_view = get_y_acc_view(traj_original);
    auto alz_view = get_z_acc_view(traj_original);
    auto aax_view = get_x_angular_acc_view(traj_original);
    auto aay_view = get_y_angular_acc_view(traj_original);
    auto aaz_view = get_z_angular_acc_view(traj_original);

    auto vlx_filt = dmp::utils::rolling_mean(vlx_view, filter);
    auto vly_filt = dmp::utils::rolling_mean(vly_view, filter);
    auto vlz_filt = dmp::utils::rolling_mean(vlz_view, filter);
    auto vax_filt = dmp::utils::rolling_mean(vax_view, filter);
    auto vay_filt = dmp::utils::rolling_mean(vay_view, filter);
    auto vaz_filt = dmp::utils::rolling_mean(vaz_view, filter);

    auto aly_filt = dmp::utils::rolling_mean(aly_view, filter);
    auto alx_filt = dmp::utils::rolling_mean(alx_view, filter);
    auto alz_filt = dmp::utils::rolling_mean(alz_view, filter);
    auto aax_filt = dmp::utils::rolling_mean(aax_view, filter);
    auto aay_filt = dmp::utils::rolling_mean(aay_view, filter);
    auto aaz_filt = dmp::utils::rolling_mean(aaz_view, filter);

    for (std::size_t i = 0; i < tr.size(); ++i) {
        std::get<2>(tr[i])(0) = vlx_filt[i];
        std::get<2>(tr[i])(1) = vly_filt[i];
        std::get<2>(tr[i])(2) = vlz_filt[i];
        std::get<2>(tr[i])(3) = vax_filt[i];
        std::get<2>(tr[i])(4) = vay_filt[i];
        std::get<2>(tr[i])(5) = vaz_filt[i];

        std::get<3>(tr[i])(0) = alx_filt[i];
        std::get<3>(tr[i])(1) = aly_filt[i];
        std::get<3>(tr[i])(2) = alz_filt[i];
        std::get<3>(tr[i])(3) = aax_filt[i];
        std::get<3>(tr[i])(4) = aay_filt[i];
        std::get<3>(tr[i])(5) = aaz_filt[i];
    }
#endif


    Se3Dmp_t dmp = dmp_ros2::dmp_from_demonstration(tr, nb);
    RCLCPP_INFO(get_logger(), "Plotting dmp");
    auto rec_traj = dmp.integrate_trajectory(tr.front(), tr.back(), 4.0, 0.002, 5.0);

    dmp_ros2::plot_trajectory(tr);
    // dmp_ros2::plot_trajectory(traj_original, "ORIGINAL");
    dmp_ros2::plot_trajectory(rec_traj);


#if 0
    const std::vector<double> t =
            dmp_ros2::get_time_view(rec_traj)
            | ranges::views::transform([](const auto& t) { return t * 1e-9; })
            | ranges::to_vector;
    const std::vector<double> s = dmp.coord_sys().compute_coordinate(t);

    Eigen::MatrixXd funcs(t.size(), nb);
    for (long i = 0; i < t.size(); ++i) {
        funcs.row(i) = dmp.learnable_func().basis().evaluate(s[i]);
    }

    plt::figure();
    plt::title("s - basis");
    for (long i = 0; i < nb; ++i) {
        std::vector<double> fv(t.size());
        for (long j = 0; j < fv.size(); ++j) fv[j] = funcs(j, i);
        plt::plot(s, fv);
    }

    plt::figure();
    plt::title("t - basis");
    for (long i = 0; i < nb; ++i) {
        std::vector<double> fv(t.size());
        for (long j = 0; j < fv.size(); ++j) fv[j] = funcs(j, i);
        plt::plot(t, fv);
    }

    plt::figure();
    plt::title("t - s");
    plt::plot(t, s);
#endif

    plt::show();
}
