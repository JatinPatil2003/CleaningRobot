#ifndef COVERAGE_PLANNER_HPP
#define COVERAGE_PLANNER_HPP

#include "map_loader.hpp"
#include "controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>

class CoveragePlanner : public MapLoader, SendGoal {
public:
    CoveragePlanner(rclcpp::Node::SharedPtr node);

    // Map coordinates conversion
    cv::Point2f pixel_to_map_coordinates(int x_pixel, int y_pixel) const;

    // Distance to the nearest black pixel
    bool distance_to_nearest_black(int y, int x) const;

    // Display modified map
    void display_modified_map();

    // Generate map points
    std::vector<std::vector<cv::Point2f>> generate_map_points(const std::vector<std::vector<cv::Point2f>>& goal_points);

    // Optimize points
    std::vector<std::vector<cv::Point2f>> optimize_points();

    // Generate coverage path
    void get_path(std::vector<std::vector<std::pair<cv::Point2f, int>>>& points);

    void start();

private:
    rclcpp::Node::SharedPtr node_;
    
    // Map parameters
    double origin_x_;
    double origin_y_;
    double resolution_;
    double x_offset_;
    double y_offset_;
    int robot_pixel_;
    int tool_pixel_;

    // Member variables
    cv::Mat map_image_;
    YAML::Node map_data_;
    std::vector<std::vector<cv::Point2f>> goal_points_;
    std::vector<std::vector<std::pair<cv::Point2f, int>>> points_;

    // Generate path
    void generate();

    // Mark visited
    void mark_visited(int x, int y);

    // Get index
    std::pair<int, int> get_index();

    void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;

    void display_map(const cv::Mat& map_image);
};

#endif // COVERAGE_PLANNER_HPP
