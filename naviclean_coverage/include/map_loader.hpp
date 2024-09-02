#ifndef MAP_LOADER_HPP
#define MAP_LOADER_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

class MapLoader {
public:
    MapLoader(rclcpp::Node::SharedPtr node);

    cv::Mat get_map_image() const;

    YAML::Node get_map_data() const;

private:
    rclcpp::Node::SharedPtr node_;

    std::string map_file_;
    std::string yaml_file_;
    
    cv::Mat map_image_;
    YAML::Node map_data_;
    
    void load_map();

    void display_map(const cv::Mat& map_image);
};

#endif // MAP_LOADER_HPP
