#include "map_loader.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <filesystem>
#include <thread>

MapLoader::MapLoader(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // Declare parameters
    node_->declare_parameter<std::string>("yaml_file", "");

    // Get parameters
    yaml_file_ = node_->get_parameter("yaml_file").as_string();
    

    // Load YAML and extract PGM file path if not provided
    if (!yaml_file_.empty()) {
        RCLCPP_INFO(node_->get_logger(), "YAML File: %s", yaml_file_.c_str());
        map_data_ = YAML::LoadFile(yaml_file_);
        map_file_ = map_data_["image"].as<std::string>();
        
        std::filesystem::path yaml_path(yaml_file_);
        map_file_ = (yaml_path.parent_path() / map_file_).string();
        RCLCPP_INFO(node_->get_logger(), "PGM File: %s", map_file_.c_str());

        load_map();
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load map image.");
    }
}

void MapLoader::load_map() {
    for (const auto& it : map_data_) {
        std::string key = it.first.as<std::string>();
        std::stringstream value_stream;
        value_stream << it.second;
        std::string value = value_stream.str();
        
        RCLCPP_INFO(node_->get_logger(), " \t%s: %s", key.c_str(), value.c_str());
    }

    map_image_ = cv::imread(map_file_, cv::IMREAD_GRAYSCALE);

    if (!map_image_.empty()) {
        RCLCPP_INFO(node_->get_logger(), "Map loaded successfully");

        // std::thread display_thread([this]() {
        //     this->display_map(this->map_image_);
        // });
        // display_thread.detach();
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load map image.");
    }
}

cv::Mat MapLoader::get_map_image() const {
    return map_image_;
}

YAML::Node MapLoader::get_map_data() const {
    return map_data_;
}

void MapLoader::display_map(const cv::Mat& map_image) {
    cv::imshow("Map Image", map_image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}
