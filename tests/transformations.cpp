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

    REQUIRE_THAT(p_vec1_t(0), Catch::Matchers::WithinAbs(1.0, 0.001));
    REQUIRE_THAT(p_vec1_t(1), Catch::Matchers::WithinAbs(-1.0, 0.001));
    REQUIRE_THAT(p_vec1_t(2), Catch::Matchers::WithinAbs(0.0, 0.001));
}

TEST_CASE("Affine transformation on quaternion", "[transformations]") {
    Quaterniond q1(AngleAxis(0.5 * M_PI, Vector3d::UnitZ()));
    if (q1.w() < 0) q1.coeffs().array() *= -1;
    const dmp_ros2::SE3 f1({0.0, 0.0, 0.0}, q1);
    const dmp_ros2::SE3 f2({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0});

    const Quaterniond   q_init      = Quaterniond::Identity();
    const Affine3d      transf      = dmp_ros2::compute_transform(f1, f2);
    const dmp_ros2::SE3 se3_from_to = dmp_ros2::se3_from_affine(transf);
    Quaterniond         q_transf    = se3_from_to.ori * q_init;
    if (q_transf.w() < 0) q_transf.coeffs().array() *= -1;

    REQUIRE_THAT(q_transf.x(), Catch::Matchers::WithinAbs(q1.x(), 0.00001));
    REQUIRE_THAT(q_transf.y(), Catch::Matchers::WithinAbs(q1.y(), 0.00001));
    REQUIRE_THAT(q_transf.z(), Catch::Matchers::WithinAbs(q1.z(), 0.00001));
    REQUIRE_THAT(q_transf.w(), Catch::Matchers::WithinAbs(q1.w(), 0.00001));
}

TEST_CASE("SE3 to affine conversion", "[transformations]") {
    const Vector3d      t = Vector3d::Random();
    const Quaterniond   r = Quaterniond::UnitRandom();
    const dmp_ros2::SE3 frame(t, r);
    const Affine3d      transf = dmp_ros2::affine_from_se3(frame);


    REQUIRE_THAT(
            (transf.translation() - t).norm(), Catch::Matchers::WithinAbs(0.0, 0.00001)
    );
    REQUIRE_THAT(
            (transf.rotation() - r.toRotationMatrix()).norm(),
            Catch::Matchers::WithinAbs(0.0, 0.00001)
    );
}

TEST_CASE("Affine to SE3 conversion", "[transformations]") {
    const Vector3d    t = Vector3d::Random();
    const Quaterniond r = Quaterniond::UnitRandom();
    Affine3d          transf(Affine3d::Identity());
    transf.translate(t).rotate(r);
    const dmp_ros2::SE3 frame = dmp_ros2::se3_from_affine(transf);


    REQUIRE_THAT((frame.pos - t).norm(), Catch::Matchers::WithinAbs(0.0, 0.00001));
    REQUIRE_THAT(
            (frame.ori.coeffs() - r.coeffs()).norm(),
            Catch::Matchers::WithinAbs(0.0, 0.00001)
    );
}
