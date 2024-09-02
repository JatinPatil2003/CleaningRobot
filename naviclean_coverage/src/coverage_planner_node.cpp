#include "map_loader.hpp"
#include "coverage_planner.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Node initialization
    auto node = std::make_shared<rclcpp::Node>("Coverage_Planner_Node");

    // MapLoader and CoveragePlanner objects
    CoveragePlanner coverage_planner(node);
    // map_loader.load_map();

    // Tool and robot dimensions for CoveragePlanner
    // int tool_pixel = static_cast<int>(round(0.45 / map_loader.pixel_to_map_coordinates(1, 1).first));
    // int robot_pixel = static_cast<int>(round(0.22 / map_loader.pixel_to_map_coordinates(1, 1).first));

    // CoveragePlanner coverage_planner(tool_pixel, robot_pixel);

    // // Generate and display coverage plan on map
    // coverage_planner.generate_map_points(map_loader.get_map_image());
    // coverage_planner.display_modified_map(map_loader.get_map_image());

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
