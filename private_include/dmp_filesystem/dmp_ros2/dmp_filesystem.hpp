#ifndef DMPROS2_FILESYSTE_HPP
#define DMPROS2_FILESYSTE_HPP

#include <filesystem>
#include <optional>
#include <vector>

namespace dmp_ros2::fs {

using Path_t = std::filesystem::path;

std::vector<Path_t> directory_content(const std::filesystem::path& base_path);

std::vector<unsigned int> demonstration_idxs(const Path_t& base_path);

std::vector<Path_t> demonstration_directories(const std::filesystem::path& base_path);

std::optional<Path_t> latest_demonstration_dir(const Path_t& base_path);

Path_t future_demonstration_dir(
        const Path_t& base_path, const bool& initialise = false
);
}  // namespace dmp_ros2::fs


#endif  // DMPROS2_FILESYSTE_HPP
