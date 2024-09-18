#include "map_loader.hpp"
#include "coverage_planner.hpp"
#include "controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Node initialization
    auto node = std::make_shared<rclcpp::Node>("Coverage_Planner_Node");

    // MapLoader and CoveragePlanner objects
    CoveragePlanner coverage_planner(node);
    RCLCPP_INFO(node->get_logger(), "Starting CoveragePlanner");

    std::thread spin_thread([&coverage_planner]() {
        coverage_planner.start();
    });
    spin_thread.detach();

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
