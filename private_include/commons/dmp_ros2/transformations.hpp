#ifndef DMPROS2_TRANSFORMATIONS_HPP
#define DMPROS2_TRANSFORMATIONS_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dmp_ros2/aliases.hpp"

namespace dmp_ros2 {

Eigen::Affine3d affine_from_se3(const SE3& frame);

/**
 * @brief Builds an affine transformation that projects points defined in the reference
 * frame "from" into the reference frame "to"
 */
Eigen::Affine3d compute_transform(const SE3& from, const SE3& to);


SE3 se3_from_affine(const Eigen::Affine3d& transform);


}  // namespace dmp_ros2


#endif  // DMPROS2_TRANSFORMATIONS_HPP
