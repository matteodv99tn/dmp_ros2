#ifndef DMPROS2_TYPE_ALIASES_HPP
#define DMPROS2_TYPE_ALIASES_HPP

#include "dmplib/manifolds/aliases.hpp"
#include "dmplib/manifolds/se3_manifold.hpp"

namespace dmp_ros2 {

using dmp::riemannmanifold::SE3;
using Traj_t    = dmp::StampedPosVelAccTrajectory_t<SE3>;
using PosTraj_t = dmp::StampedPosTrajectory_t<SE3>;


}  // namespace dmp_ros2


#endif  // DMPROS2_TYPE_ALIASES_HPP
