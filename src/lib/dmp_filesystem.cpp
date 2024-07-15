#include "dmp_ros2/dmp_filesystem.hpp"

#include <filesystem>
#include <fmt/format.h>
#include <string_view>
#include <vector>

#include "range/v3/all.hpp"

namespace fs = dmp_ros2::fs;

namespace rs = ranges;
namespace rv = ranges::views;

using std::filesystem::path;

constexpr char demonstration_prefix[] = "demonstration_";
constexpr int  prefix_length          = std::string_view(demonstration_prefix).size();

std::vector<path>
fs::directory_content(const path& base_path) {
    return std::filesystem::directory_iterator(base_path) | rs::to<std::vector<path>>;
}

std::vector<unsigned int>
fs::demonstration_idxs(const path& base_path) {
    std::vector<path> path_entries = directory_content(base_path);
    auto is_directory = [](const path& p) { return std::filesystem::is_directory(p); };
    auto get_filename = [](const path& p) { return p.filename(); };
    auto is_demonstration_dir = [](const std::string& fn) {
        return fn.find(demonstration_prefix) != std::string::npos;
    };
    return path_entries | rv::filter(is_directory) | rv::transform(get_filename)
           | rv::filter(is_demonstration_dir)
           | rv::transform([](const std::string& fn) -> unsigned int {
                 return std::stoi(fn.substr(prefix_length));
             })
           | rs::to_vector;
}

std::vector<path>
fs::demonstration_directories(const std::filesystem::path& base_path) {
    std::vector<path> path_entries = directory_content(base_path);
    auto is_directory = [](const path& p) { return std::filesystem::is_directory(p); };
    auto get_filename = [](const path& p) { return p.filename(); };
    auto is_demonstration_dir = [](const std::string& fn) {
        return fn.find(demonstration_prefix) != std::string::npos;
    };
    return path_entries | rv::filter(is_directory) | rv::filter(is_demonstration_dir)
           | rs::to_vector;
}

std::optional<path>
fs::latest_demonstration_dir(const path& base_path) {
    const std::vector<unsigned int> idxs = demonstration_idxs(base_path);
    if (idxs.empty()) return std::nullopt;
    unsigned int      biggest_id = rs::max(idxs);
    const std::string filename =
            fmt::format("{}{:03}", demonstration_prefix, biggest_id);
    return base_path / filename;
}

path
fs::future_demonstration_dir(const path& base_path, const bool& initialise) {
    const std::vector<unsigned int> idxs      = demonstration_idxs(base_path);
    const unsigned int              folder_id = idxs.size() > 0 ? rs::max(idxs) + 1 : 0;
    const std::string               filename =
            fmt::format("{}{:03}", demonstration_prefix, folder_id);
    const path newpath = base_path / filename;

    if (initialise) std::filesystem::create_directory(newpath);
    return newpath;
}
