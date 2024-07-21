#include "dmp_ros2/data_preprocessor.hpp"

#include <cstddef>
#include <stdexcept>

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logging.hpp>

#include "dmp_ros2/defines.hpp"
#include "dmp_ros2/demonstration_handler.hpp"
#include "dmp_ros2/dmp_filesystem.hpp"
#include "dmp_ros2/preprocessing.hpp"
#include "dmplib/utils/filter.hpp"
#include "range/v3/view/intersperse.hpp"
#include "range/v3/view/join.hpp"
#include "range/v3/view/transform.hpp"

using dmp_ros2::DataPreprocessor;
using dmp_ros2::SegmentationProperties;
using namespace dmp_ros2::fs;

namespace rs = ranges;
namespace rv = ranges::views;

using std::size_t;
using std::string;
using std::vector;

namespace params {
static constexpr char dem_dir[]        = "demonstrations_directory";
static constexpr char dem_frame_name[] = "car_chassis";
static constexpr char filt_w[]         = "filter_weights";
static constexpr char vel_th[]         = "segmentation.velocity_th";
static constexpr char min_dem_len[]    = "segmentation.minimum_window_length";
static constexpr char win_sz[]         = "segmentation.window_size";
static constexpr char tail_sz[]        = "segmentation.tail_size";
static constexpr char dmp_basis[]      = "n_basis";
}  // namespace params

void
DataPreprocessor::initialise_parameters() {
    declare_parameter(params::dem_dir, dmp_ros2::constants::tests_data_dir);
    declare_parameter(params::dem_frame_name, "car_chassis");
    declare_parameter(params::filt_w, vector<double>({2.0, 1.0, 1.0, 0.5}));
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
    const string dem_dir        = get_parameter(params::dem_dir).as_string();
    const string dem_frame_name = get_parameter(params::dem_frame_name).as_string();
    const vector<double> filter = get_parameter(params::filt_w).as_double_array();
    const double         vel_th = get_parameter(params::vel_th).as_double();
    const size_t         min_dem_len = get_parameter(params::min_dem_len).as_int();
    const size_t         win_sz      = get_parameter(params::win_sz).as_int();
    const size_t         tail_sz     = get_parameter(params::tail_sz).as_int();

    const std::string filt_desc =
            filter | rv::transform([](const double& d) { return std::to_string(d); })
            | rv::intersperse(", ") | rv::join | rs::to<string>;

    RCLCPP_INFO(
            get_logger(),
            "Applying non-causal moving average filters on loaded data with "
            "coefficients: %s",
            filt_desc.c_str()
    );
    RCLCPP_INFO(get_logger(), "Trajectory segmentation properties");
    RCLCPP_INFO(get_logger(), "  - velocity threshold: %f", vel_th);
    RCLCPP_INFO(get_logger(), "  - minimum demonstration length: %zu", min_dem_len);
    RCLCPP_INFO(get_logger(), "  - window size: %zu", win_sz);
    RCLCPP_INFO(get_logger(), "  - tail size: %zu", tail_sz);
    RCLCPP_INFO(
            get_logger(), "Projecting data into ref. frame '%s'", dem_frame_name.c_str()
    );


    RCLCPP_INFO(
            get_logger(), "Scanning for demonstrations in folder %s", dem_dir.c_str()
    );
    const auto base_directories = fs::demonstration_directories(dem_dir);
    RCLCPP_INFO(get_logger(), "%zu directories found", base_directories.size());

    SegmentationProperties seg_props;
    seg_props.min_demonstration_length = min_dem_len;
    seg_props.vel_th                   = vel_th;
    seg_props.window_size              = win_sz;
    seg_props.stop_len                 = tail_sz;

    for (const Path_t& dir : base_directories) {
        process_single_directory(dir, dem_frame_name, filter, seg_props);
    }
}

void
DataPreprocessor::process_single_directory(
        const fs::Path_t&             dir,
        const std::string&            data_domain,
        const vector<double>&         filter,
        const SegmentationProperties& seg_props
) const {
    RCLCPP_INFO(get_logger(), "Processing folder %s", dir.c_str());
    fs::DemonstrationHandler handler(dir);

    auto raw_traj = handler.load_raw_trajectory(data_domain);
    RCLCPP_INFO(get_logger(), " - n. of data points: %zu", raw_traj.size());

    const auto traj = dmp_ros2::differentiate(
            dmp_ros2::rolling_mean(dmp::utils::remove_duplicates(raw_traj), filter)
    );

    const auto subtrajs = dmp_ros2::segment_trajectory(traj, seg_props);
    RCLCPP_INFO(get_logger(), " - computed %zu demonstrations", subtrajs.size());
    handler.write_demonstrated_trajectories(subtrajs);

    for (const Path_t& path : handler.list_demonstrated_trajectories()) {
        RCLCPP_INFO(get_logger(), "      Written file %s", path.c_str());
    }
}
