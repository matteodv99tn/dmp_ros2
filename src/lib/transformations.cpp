#include "dmp_ros2/transformations.hpp"

#include <Eigen/Geometry>

#include "dmp_ros2/aliases.hpp"

using Eigen::Affine3d;

Affine3d
dmp_ros2::compute_transform(const SE3& from, const SE3& to) {
    Affine3d M_from_world, M_to_world;

    M_from_world.setIdentity();
    M_to_world.setIdentity();

    M_from_world.translate(from.pos).rotate(from.ori);
    M_to_world.translate(to.pos).rotate(to.ori);

    return M_to_world.inverse() * M_from_world;
}
