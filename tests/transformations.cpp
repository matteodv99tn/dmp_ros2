#include "dmp_ros2/transformations.hpp"

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "dmp_ros2/aliases.hpp"

using namespace Eigen;

TEST_CASE("Test simple transformations", "[transformations]") {
    const dmp_ros2::SE3 f1({1.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0});
    const dmp_ros2::SE3 f2({0.0, 1.0, 0.0}, {1.0, 0.0, 0.0, 0.0});

    const Affine3d transf = dmp_ros2::compute_transform(f1, f2);

    REQUIRE(transf.translation()(0) == 1.0);
    REQUIRE(transf.translation()(1) == -1.0);
    REQUIRE(transf.translation()(2) == 0.0);

    const Vector3d p_vec1{1.0, 1.0, 0.0};
    const Vector3d p_vec1_t = transf * p_vec1;

    REQUIRE(p_vec1_t(0) == 2.0);
    REQUIRE(p_vec1_t(1) == 0.0);
    REQUIRE(p_vec1_t(2) == 0.0);
}

TEST_CASE("Test rotation transformation", "[transformations]") {
    const Quaterniond   q2(AngleAxis(0.5 * M_PI, Vector3d::UnitZ()));
    const dmp_ros2::SE3 f1({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0});
    const dmp_ros2::SE3 f2({0.0, 0.0, 0.0}, q2);

    const Affine3d transf = dmp_ros2::compute_transform(f1, f2);

    REQUIRE(transf.translation().norm() <= 1e-6);

    const Vector3d p_vec1{1.0, 1.0, 0.0};
    const Vector3d p_vec1_t = transf * p_vec1;

    REQUIRE_THAT(p_vec1_t(0), Catch::Matchers::WithinRel(1.0, 0.001));
    REQUIRE_THAT(p_vec1_t(1), Catch::Matchers::WithinRel(-1.0, 0.001));
    REQUIRE_THAT(p_vec1_t(2), Catch::Matchers::WithinRel(0.0, 0.001));
}
