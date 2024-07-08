#ifndef DMPROS2_DEMONSTRATION_HANDLER_HPP
#define DMPROS2_DEMONSTRATION_HANDLER_HPP

#include <filesystem>
#include <string>
#include <vector>

#include "dmplib/data_handler/conversions.hpp"
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

    void write_frames_transforms(const std::vector<RefFrameData_t>& frames_data) const;

    RawTrajectory_t load_raw_trajectory() const;

    void write_demonstrated_trajectories(
            const std::vector<DemonstrationTrajectory_t>& trajs
    );

    std::vector<Path_t> list_demonstrated_trajectories() const;
    std::vector<DemonstrationTrajectory_t> load_demonstrated_trajectories() const;


    bool has_all_raw_entries() const;

    Path_t raw_data_dir_path() const;
    Path_t raw_data_file() const;
    Path_t raw_transforms_file() const;
    Path_t raw_data_refframename_file() const;

    Path_t demonstration_data_dir_path() const;

private:
    Path_t _folder_path;

    void write_rawtraj_refframe(const std::string& frame_name) const;
};

}  // namespace dmp_ros2::fs


#endif  // DMPROS2_DEMONSTRATION_HANDLER_HPP
