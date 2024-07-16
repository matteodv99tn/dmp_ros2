#ifndef DMPROS2_PLOTS_HPP
#define DMPROS2_PLOTS_HPP

#include <string>

#include "dmp_ros2/aliases.hpp"
#include "dmp_ros2/se3_dmp.hpp"

namespace dmp_ros2 {

void plot_trajectory(const Traj_t& traj, const std::string& title = "");
void plot_trajectory_comparison(const Traj_t& demonstration, Se3Dmp_t& reconstruction);

}  // namespace dmp_ros2

#endif  // DMPROS2_PLOTS_HPP
