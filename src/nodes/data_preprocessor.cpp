#include "dmp_ros2/data_preprocessor.hpp"

#include <cstddef>
#include <iostream>
#include <stdexcept>

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logging.hpp>

#include "dmp_ros2/defines.hpp"
#include "dmp_ros2/demonstration_handler.hpp"
#include "dmp_ros2/dmp_filesystem.hpp"
#include "dmp_ros2/preprocessing.hpp"
#include "dmp_ros2/se3_dmp.hpp"
#include "dmplib/utils/filter.hpp"
#include "fmt/format.h"


using dmp_ros2::DataPreprocessor;
using namespace dmp_ros2::fs;

namespace params {
constexpr char dem_dir[]     = "demonstrations_directory";
constexpr char filt_w[]      = "filter_weights";
constexpr char vel_th[]      = "segmentation.velocity_th";
constexpr char min_dem_len[] = "segmentation.minimum_window_length";
constexpr char win_sz[]      = "segmentation.window_size";
constexpr char tail_sz[]     = "segmentation.tail_size";
}  // namespace params

void
DataPreprocessor::initialise_parameters() {
    declare_parameter(params::dem_dir, dmp_ros2::constants::tests_data_dir);
    declare_parameter(params::filt_w, std::vector<double>({2.0, 1.0, 1.0, 0.5}));
    declare_parameter(params::vel_th, 0.2);
    declare_parameter(params::min_dem_len, 200);
    declare_parameter(params::win_sz, 40);
    declare_parameter(params::tail_sz, 60);
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

    RCLCPP_INFO(get_logger(), "Applying non-causal moving average filters on loaded data with coefficients");
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

    for (const auto& tr: subtrajs){
        [[maybe_unused]] dmp_ros2::Se3Dmp_t d= dmp_ros2::dmp_from_demonstration(tr, 25);
    }
}
