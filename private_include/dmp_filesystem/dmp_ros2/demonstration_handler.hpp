#ifndef DMPROS2_DEMONSTRATION_HANDLER_HPP
#define DMPROS2_DEMONSTRATION_HANDLER_HPP

#include <Eigen/Geometry>
#include <filesystem>
#include <string>
#include <vector>

#include "dmp_ros2/aliases.hpp"
#include "dmp_ros2/se3_dmp.hpp"
#include "dmplib/manifolds/se3_manifold.hpp"

namespace dmp_ros2::fs {

class DemonstrationHandler {
public:
    using Path_t         = std::filesystem::path;
    using SE3            = dmp::riemannmanifold::SE3;
    using RefFrameData_t = std::pair<std::string, SE3>;  // pair of reference
                                                         // frame name and pose
                                                         // (w.r.t. world)
    using RawTrajectory_t           = dmp::StampedPosTrajectory_t<SE3>;
    using DemonstrationTrajectory_t = dmp::StampedPosVelAccTrajectory_t<SE3>;

    DemonstrationHandler(const Path_t& demonstration_folder_path);

    void write_raw_trajectory(
            const RawTrajectory_t& traj, const std::string& reference_frame
    ) const;
    RawTrajectory_t load_raw_trajectory(const std::string& diplay_in = "") const;

    void write_frames_transforms(const std::vector<RefFrameData_t>& frames_data) const;
    const std::vector<RefFrameData_t>& load_frame_transforms();

    void write_demonstrated_trajectories(
            const std::vector<DemonstrationTrajectory_t>& trajs
    );

    std::vector<Path_t>                    list_demonstrated_trajectories() const;
    std::vector<DemonstrationTrajectory_t> load_demonstrated_trajectories() const;
    std::vector<Se3Dmp_t> load_demonstrated_dmps(const std::size_t& n_basis) const;


    std::optional<Eigen::Affine3d> get_transformation(
            const std::string& from, const std::string& to
    ) const;


    bool has_all_raw_entries() const;

    Path_t raw_data_dir_path() const;
    Path_t raw_data_file() const;
    Path_t raw_transforms_file() const;
    Path_t raw_data_refframename_file() const;

    Path_t demonstration_data_dir_path() const;

private:
    Path_t                                     _folder_path;
    std::optional<std::vector<RefFrameData_t>> _transforms;

    void write_rawtraj_refframe(const std::string& frame_name) const;   
    std::string raw_data_frame() const;
};

std::vector<DemonstrationHandler> all_demonstration_handlers(
        const std::filesystem::path& base_path
);

std::vector<Se3Dmp_t> all_demonstrations_dmps(
        const std::filesystem::path& base_path, std::size_t& n_basis
);

}  // namespace dmp_ros2::fs


#endif  // DMPROS2_DEMONSTRATION_HANDLER_HPP
