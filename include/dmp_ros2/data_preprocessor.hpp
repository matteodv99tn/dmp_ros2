#ifndef DMPROS2_DATA_PREPROCESSOR_HPP
#define DMPROS2_DATA_PREPROCESSOR_HPP

#include <rclcpp/node.hpp>

#include "dmp_ros2/dmp_filesystem.hpp"
#include "dmp_ros2/preprocessing.hpp"

namespace dmp_ros2 {

class DataPreprocessor : public rclcpp::Node {
public:
    DataPreprocessor();

    void process_data() const;

private:
    void initialise_parameters();

    void process_single_directory(
            const fs::Path_t&             dir,
            const std::string&            data_domain,
            const std::vector<double>&    filter,
            const SegmentationProperties& seg_props
    ) const;
};

}  // namespace dmp_ros2


#endif  // DMPROS2_DATA_PREPROCESSOR_HPP
