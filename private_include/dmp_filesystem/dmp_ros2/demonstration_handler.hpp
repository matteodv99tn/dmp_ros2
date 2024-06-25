#ifndef DMPROS2_DEMONSTRATION_HANDLER_HPP
#define DMPROS2_DEMONSTRATION_HANDLER_HPP

#include <filesystem>
#include <string>

#include "dmplib/data_handler/conversions.hpp"
#include "dmplib/manifolds/se3_manifold.hpp"

namespace dmp_ros2::fs {

class DemonstrationHandler {
public:
    using Path_t = std::filesystem::path;
    using RefFrameData_t =
            std::pair<std::string, dmp::riemannmanifold::SE3>;  // pair of reference
                                                                // frame name and pose
                                                                // (w.r.t. world)


    DemonstrationHandler(const Path_t& demonstration_folder_path);

    Path_t raw_data_dir_path() const;
    Path_t raw_data_file() const;
    Path_t raw_transforms_file() const;
    Path_t raw_data_refframename_file() const;

    void
    write_raw_trajectory(const auto& traj, const std::string& reference_frame) const {
        dmp::to::file(raw_data_file(), traj);
        write_rawtraj_refframe(reference_frame);
    }

    void write_frames_transforms(const std::vector<RefFrameData_t>& frames_data) const;


private:
    Path_t _folder_path;

    void write_rawtraj_refframe(const std::string& frame_name) const;
};

}  // namespace dmp_ros2::fs


#endif  // DMPROS2_DEMONSTRATION_HANDLER_HPP
