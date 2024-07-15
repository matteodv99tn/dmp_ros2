#include "dmp_ros2/demonstration_handler.hpp"

#include <stdexcept>

#include "dmp_ros2/dmp_filesystem.hpp"
#include "dmp_ros2/se3_dmp.hpp"
#include "dmplib/data_handler/conversions.hpp"
#include "dmplib/manifolds/aliases.hpp"
#include "dmplib/manifolds/se3_manifold.hpp"
#include "range/v3/all.hpp"
#include "range/v3/view/join.hpp"
#include "range/v3/view/transform.hpp"

using dmp_ros2::Se3Dmp_t;
using dmp_ros2::fs::DemonstrationHandler;
using Path_t    = DemonstrationHandler::Path_t;
using DemTraj_t = DemonstrationHandler::DemonstrationTrajectory_t;

namespace rs = ranges;
namespace rv = ranges::views;

void
close_file(std::FILE* f) {
    std::fclose(f);
}

using FilePtr_t = std::unique_ptr<std::FILE, decltype(&close_file)>;

FilePtr_t
open_file(const DemonstrationHandler::Path_t& file_path) {
    return {std::fopen(file_path.c_str(), "w"), &close_file};
}

DemonstrationHandler::DemonstrationHandler(const Path_t& demonstration_folder_path) :
        _folder_path(demonstration_folder_path) {
    if (!std::filesystem::exists(_folder_path))
        std::filesystem::create_directory(_folder_path);
}

void
DemonstrationHandler::write_raw_trajectory(
        const RawTrajectory_t& traj, const std::string& reference_frame
) const {
    dmp::to::file(raw_data_file(), traj);
    write_rawtraj_refframe(reference_frame);
}

void
DemonstrationHandler::write_frames_transforms(
        const std::vector<RefFrameData_t>& frames_data
) const {
    const FilePtr_t output_file = open_file(raw_transforms_file());
    for (const auto& frame : frames_data) {
        fmt::println(
                output_file.get(), "{}:{}", frame.first, dmp::to::string(frame.second)
        );
    }
}

DemonstrationHandler::RawTrajectory_t
DemonstrationHandler::load_raw_trajectory() const {
    if (!has_all_raw_entries())
        throw std::runtime_error("Not all files are present in the raw data folder");
    return dmp::from::file<std::tuple<dmp::TimeStamp_t, SE3>>(raw_data_file().string());
}

void
DemonstrationHandler::write_demonstrated_trajectories(
        const std::vector<DemonstrationTrajectory_t>& trajs
) {
    for (std::size_t i = 0; i < trajs.size(); ++i) {
        const Path_t filepath = demonstration_data_dir_path()
                                / fmt::format("demonstration_{:03}.csv", i);
        dmp::to::file(filepath.string(), trajs[i]);
    }
}

std::vector<Path_t>
DemonstrationHandler::list_demonstrated_trajectories() const {
    auto path_entries     = fs::directory_content(demonstration_data_dir_path());
    auto is_demonstration = [](const Path_t& p) -> bool {
        return p.filename().string().find("demonstration") != std::string::npos;
    };

    return path_entries | rv::filter(is_demonstration) | rs::to_vector;
}

std::vector<DemTraj_t>
DemonstrationHandler::load_demonstrated_trajectories() const {
    std::vector<Path_t> paths = list_demonstrated_trajectories();
    return paths | rv::transform([](const auto& p) {
               return dmp::from::file<dmp::StampedPosVelAccSample_t<SE3>>(p.string());
           })
           | rs::to_vector;
}

std::vector<Se3Dmp_t>
DemonstrationHandler::load_demonstrated_dmps(const std::size_t& n_basis) const {
    std::vector<DemTraj_t> trajs = load_demonstrated_trajectories();
    return trajs | rv::transform([n = n_basis](const auto& tr) {
               return dmp_from_demonstration(tr, n);
           })
           | rs::to_vector;
}

void
DemonstrationHandler::write_rawtraj_refframe(const std::string& frame_name) const {
    const FilePtr_t output_file = open_file(raw_data_refframename_file());
    fmt::print(output_file.get(), "{}", frame_name);
}

bool
DemonstrationHandler::has_all_raw_entries() const {
    using std::filesystem::exists;
    return exists(raw_data_file()) && exists(raw_transforms_file())
           && exists(raw_data_refframename_file());
}

Path_t
DemonstrationHandler::raw_data_dir_path() const {
    Path_t raw_dir = _folder_path / "raw_data";
    if (!std::filesystem::exists(raw_dir)) std::filesystem::create_directory(raw_dir);
    return raw_dir;
}

Path_t
DemonstrationHandler::raw_data_file() const {
    return raw_data_dir_path() / "data.csv";
}

Path_t
DemonstrationHandler::raw_transforms_file() const {
    return raw_data_dir_path() / "transforms.txt";
}

Path_t
DemonstrationHandler::raw_data_refframename_file() const {
    return raw_data_dir_path() / "data_domain.txt";
}

Path_t
DemonstrationHandler::demonstration_data_dir_path() const {
    Path_t raw_dir = _folder_path / "processed_data";
    if (!std::filesystem::exists(raw_dir)) std::filesystem::create_directory(raw_dir);
    return raw_dir;
}

std::vector<DemonstrationHandler>
dmp_ros2::fs::all_demonstration_handlers(const Path_t& base_path) {
    std::vector<Path_t> directories =
            dmp_ros2::fs::demonstration_directories(base_path);
    return directories
           | rv::transform([](const auto& dir) { return DemonstrationHandler(dir); })
           | rs::to_vector;
}

std::vector<Se3Dmp_t>
dmp_ros2::fs::all_demonstrations_dmps(
        const std::filesystem::path& base_path, std::size_t& n_basis
) {
    const std::vector<DemonstrationHandler> handlers =
            all_demonstration_handlers(base_path);
    std::vector<Se3Dmp_t> res;
    for (const auto& h : handlers) {
        std::vector<Se3Dmp_t> dmps = h.load_demonstrated_dmps(n_basis);
        for (Se3Dmp_t& d : dmps) res.push_back(std::move(d));
    }
    return res;
}
