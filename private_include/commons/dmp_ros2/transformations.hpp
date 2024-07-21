#ifndef DMPROS2_TRANSFORMATIONS_HPP
#define DMPROS2_TRANSFORMATIONS_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dmp_ros2/aliases.hpp"

namespace dmp_ros2 {

/**
 * @brief Builds an affine transformation that projects points defined in the reference
 * frame "from" into the reference frame "to"
 */
Eigen::Affine3d compute_transform(const SE3& from, const SE3& to);


}  // namespace dmp_ros2


#endif  // DMPROS2_TRANSFORMATIONS_HPP
