#include "dmp_ros2/demonstration_handler.hpp"

#include "dmplib/data_handler/conversions.hpp"
#include "dmplib/manifolds/se3_manifold.hpp"

using dmp_ros2::fs::DemonstrationHandler;

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

void
DemonstrationHandler::write_rawtraj_refframe(const std::string& frame_name) const {
    const FilePtr_t output_file = open_file(raw_data_refframename_file());
    fmt::print(output_file.get(), "{}", frame_name);
}

DemonstrationHandler::Path_t
DemonstrationHandler::raw_data_dir_path() const {
    Path_t raw_dir = _folder_path / "raw_data";
    if (!std::filesystem::exists(raw_dir)) std::filesystem::create_directory(raw_dir);

    return raw_dir;
}

DemonstrationHandler::Path_t
DemonstrationHandler::raw_data_file() const {
    return raw_data_dir_path() / "data.csv";
}

DemonstrationHandler::Path_t
DemonstrationHandler::raw_transforms_file() const {
    return raw_data_dir_path() / "transforms.txt";
}

DemonstrationHandler::Path_t
DemonstrationHandler::raw_data_refframename_file() const {
    return raw_data_dir_path() / "data_domain.txt";
}
