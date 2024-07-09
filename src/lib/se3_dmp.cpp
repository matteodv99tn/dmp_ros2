#include "dmp_ros2/se3_dmp.hpp"

#include <cstddef>

#include "dmplib/learnable_functions/basis_functions/gaussian_bf.hpp"

using dmp::learnablefunction::GaussianBf;
using dmp_ros2::Se3Dmp_t;
using dmp_ros2::Se3Traj_t;

Se3Dmp_t
dmp_ros2::dmp_from_demonstration(const Se3Traj_t& traj, const std::size_t& n_basis) {
    Se3Dmp_t dmp;

    dmp.initialise_coordinate_system(dmp.get_period());
    dmp.initialise_transformation_system(dmp.get_period());

    const std::vector<double> c = dmp.coord_sys().distribution_on_support(n_basis);
    dmp.initialise_learnable_function(GaussianBf(n_basis, c));

    dmp.batch_learn(traj, true);
    return dmp;
}
