#ifndef DMPROS2_FILESYSTE_HPP
#define DMPROS2_FILESYSTE_HPP

#include <filesystem>
#include <vector>
#include <optional>

namespace dmp_ros2::fs {

std::vector<std::filesystem::path> directory_content(
        const std::filesystem::path& base_path
);

std::vector<unsigned int> demonstration_idxs(const std::filesystem::path& base_path);

std::optional<std::filesystem::path> latest_demonstration_dir(const std::filesystem::path& base_path);

std::filesystem::path future_demonstration_dir(
        const std::filesystem::path& base_path, const bool& initialise = false
);
}  // namespace dmp_ros2::fs


#endif  // DMPROS2_FILESYSTE_HPP
