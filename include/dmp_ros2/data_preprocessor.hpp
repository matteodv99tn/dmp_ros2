#ifndef DMPROS2_DATA_PREPROCESSOR_HPP
#define DMPROS2_DATA_PREPROCESSOR_HPP

#include <rclcpp/node.hpp>

namespace dmp_ros2 {

class DataPreprocessor: public rclcpp::Node {

public: 
    DataPreprocessor();

    void process_data() const;

private:
    void initialise_parameters();


};

} // namespace dmp_ros2


#endif  // DMPROS2_DATA_PREPROCESSOR_HPP
