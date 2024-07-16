#include <cmath>
#include <iostream>

#include "dmp_ros2/aliases.hpp"
#include "dmp_ros2/defines.hpp"
#include "dmp_ros2/demonstration_handler.hpp"
#include "dmp_ros2/dmp_filesystem.hpp"
#include "dmp_ros2/plots.hpp"
#include "dmp_ros2/se3_dmp.hpp"
#include "dmplib/manifolds/aliases.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int
main() {
    auto dir =
            dmp_ros2::fs::latest_demonstration_dir(dmp_ros2::constants::tests_data_dir);
    dmp_ros2::fs::DemonstrationHandler handler(dir.value());

    const auto dem_path = handler.list_demonstrated_trajectories()[0];
    std::cout << "Loading demonstration " << dem_path << std::endl;
    const auto traj =
            dmp::from::file<dmp::StampedPosVelAccSample_t<dmp_ros2::SE3>>(dem_path);
    auto dmp = dmp_ros2::dmp_from_demonstration(traj, 10);

    dmp_ros2::plot_trajectory_comparison(traj, dmp);

    plt::show();
    return 0;
}
