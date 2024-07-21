#include "dmp_ros2/transformations.hpp"

#include <Eigen/Geometry>
#include <Eigen/src/Geometry/Transform.h>

#include "dmp_ros2/aliases.hpp"

using dmp_ros2::SE3;
using Eigen::Affine3d;
using Eigen::Quaterniond;

Affine3d
dmp_ros2::affine_from_se3(const SE3& frame) {
    Affine3d transform(Affine3d::Identity());
    transform.translate(frame.pos).rotate(frame.ori);
    return transform;
}

Affine3d
dmp_ros2::compute_transform(const SE3& from, const SE3& to) {
    const Affine3d M_from_world = affine_from_se3(from);
    const Affine3d M_to_world   = affine_from_se3(to);
    return M_to_world.inverse() * M_from_world;
}

SE3
dmp_ros2::se3_from_affine(const Eigen::Affine3d& transform) {
    Quaterniond q(transform.rotation());
    if (q.w() < 0.0) q.coeffs().array() *= -1;
    return {transform.translation(), q};
}
