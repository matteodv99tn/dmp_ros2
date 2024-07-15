#ifndef DMPROS2_SE3_DMP_HPP
#define DMPROS2_SE3_DMP_HPP

#include "dmplib/coordinate_systems/exponential_decay_cs.hpp"
#include "dmplib/dmp.hpp"
#include "dmplib/learnable_functions/basis_functions/gaussian_bf.hpp"
#include "dmplib/learnable_functions/weighted_basis_function.hpp"
#include "dmplib/manifolds/aliases.hpp"
#include "dmplib/manifolds/se3_manifold.hpp"
#include "dmplib/transformation_systems/modified_second_order_tf.hpp"
#include "dmplib/transformation_systems/second_order_tf.hpp"

#define USE_MODIFIED_SYSTEM
namespace dmp_ros2 {

#ifdef USE_MODIFIED_SYSTEM
using Se3Dmp_t = dmp::Dmp<
        dmp::riemannmanifold::SE3,
        dmp::ExponentialDecayCs,
        dmp::transformationsystem::ModifiedSecondOrderTs<dmp::riemannmanifold::SE3>,
        dmp::learnablefunction::WeightedBasisFunction<
                dmp::learnablefunction::GaussianBf,
                dmp::riemannmanifold::SE3>>;
#else
using Se3Dmp_t = dmp::Dmp<
        dmp::riemannmanifold::SE3,
        dmp::ExponentialDecayCs,
        dmp::transformationsystem::SecondOrderTs<dmp::riemannmanifold::SE3>,
        dmp::learnablefunction::WeightedBasisFunction<
                dmp::learnablefunction::GaussianBf,
                dmp::riemannmanifold::SE3>>;
#endif

using Se3Traj_t = dmp::StampedPosVelAccTrajectory_t<dmp::riemannmanifold::SE3>;

Se3Dmp_t dmp_from_demonstration(const Se3Traj_t& traj, const std::size_t& n_basis);


}  // namespace dmp_ros2

#endif  // DMPROS2_SE3_DMP_HPP
