#ifndef DMPROS2_PLOTS_HPP
#define DMPROS2_PLOTS_HPP

#include "dmp_ros2/aliases.hpp"
#include <string>

namespace dmp_ros2 {

void plot_trajectory(const Traj_t& traj, const std::string& title = "");


}  // namespace dmp_ros2


#endif  // DMPROS2_PLOTS_HPP
