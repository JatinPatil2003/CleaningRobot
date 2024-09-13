#include "coverage_planner.hpp"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <thread>

CoveragePlanner::CoveragePlanner(rclcpp::Node::SharedPtr node)
    : MapLoader(node), SendGoal(node), node_(node)
{
    // Initialize map parameters
    map_data_ = MapLoader::get_map_data();
    map_image_ = MapLoader::get_map_image();

    origin_x_ = map_data_["origin"][0].as<double>();
    origin_y_ = map_data_["origin"][1].as<double>();
    resolution_ = map_data_["resolution"].as<double>();

    // Default offsets and pixels
    x_offset_ = 0.0;
    y_offset_ = 0.0;
    robot_pixel_ = static_cast<int>(0.15 / resolution_); // Example value, adjust as needed
    tool_pixel_ = static_cast<int>(0.15 * 2 / resolution_); // Example value, adjust as needed

    display_modified_map();
}

bool CoveragePlanner::distance_to_nearest_black(int y, int x) const
{
    for (int i = -robot_pixel_; i <= robot_pixel_; ++i)
    {
        for (int j = -robot_pixel_; j <= robot_pixel_; ++j)
        {
            // Ensure the coordinates are within the map bounds
            if (y + i >= 0 && y + i < map_image_.rows &&
                x + j >= 0 && x + j < map_image_.cols)
            {

                // Check if the pixel is black (or close to black)
                if (map_image_.at<uchar>(y + i, x + j) < 10)
                {                 // Assuming < 250 is black
                    return false; // Too close to a black pixel
                }
            }
        }
    }

    return true; // No black pixel within the radius
}

void CoveragePlanner::mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy) const
{
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = - origin_y_ - (my + 0.5) * resolution_;
}

cv::Point2f CoveragePlanner::pixel_to_map_coordinates(int x, int y) const
{
    double wx, wy;
    mapToWorld(x, y, wx, wy); // Call the mapToWorld function to get world coordinates
    return cv::Point2f(static_cast<float>(wx), static_cast<float>(wy));
}

void CoveragePlanner::display_modified_map()
{
    if (!map_image_.empty())
    {
        cv::Mat modified_map = map_image_.clone();

        std::vector<cv::Point2f> goal_pixel_positions;
        std::vector<std::vector<cv::Point2f>> goal_positions;
        std::vector<std::vector<std::pair<cv::Point2f, int>>> res;

        int goal_index = 1;

        for (int x = 0; x < modified_map.cols; x += tool_pixel_)
        {
            std::vector<std::pair<cv::Point2f, int>> col;
            for (int y = 0; y < modified_map.rows; y += tool_pixel_)
            {
                if (modified_map.at<uchar>(y, x) >= 250 && distance_to_nearest_black(y, x))
                {
                    // Mark the first and last goals with different colors

                    modified_map.at<uchar>(y, x) = 180;

                    goal_pixel_positions.push_back(cv::Point2f(x, y));
                    cv::Point2f map_coords = pixel_to_map_coordinates(x, y);
                    goal_positions.push_back({map_coords});
                    col.push_back({cv::Point2f(x, y), 255});
                    // std::cout << res << std::endl;
                    ++goal_index;
                }
                else
                {
                    col.push_back({cv::Point2f(x, y), 0});
                }
            }
            res.push_back(col);
        }

        get_path(res);

        goal_points_ = optimize_points();

        for (const auto &points : goal_points_)
        {
            if (points.size() < 2)
            {
                // Skip if there are fewer than 2 points to connect
                continue;
            }

            // Iterate through points in the current list
            for (size_t i = 1; i < points.size(); ++i)
            {
                const cv::Point2f &start_point = points[i - 1];
                const cv::Point2f &end_point = points[i];

                // Draw a line between consecutive points
                cv::line(modified_map, cv::Point(static_cast<int>(start_point.x), static_cast<int>(start_point.y)),
                         cv::Point(static_cast<int>(end_point.x), static_cast<int>(end_point.y)),
                         cv::Scalar(100), // Line color (green in this case)
                         1);              // Line thickness
            }
        }

        goal_points_ = generate_map_points(goal_points_);

        for (auto i = 0; i < static_cast<int>(goal_points_.size()); ++i)
        {
            for (auto j = 0; j < static_cast<int>(goal_points_[i].size()); ++j)
            {
                std::cout << goal_points_[i][j] << std::endl;
            }
            std::cout << std::endl
                      << std::endl;
        }

        std::thread display_thread([this, modified_map]() {
            this->display_map(modified_map);
        });
        display_thread.detach();

    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Map image not available. Call load_map() first.");
    }
}

void CoveragePlanner::get_path(std::vector<std::vector<std::pair<cv::Point2f, int>>> &points)
{
    points_ = points;
    goal_points_.clear();
    generate();
}

std::pair<int, int> CoveragePlanner::get_index()
{
    for (int i = 0u; i < static_cast<int>(points_.size()); ++i)
    {
        for (int j = 0u; j < static_cast<int>(points_[i].size()); ++j)
        {
            if (points_[i][j].second == 255)
            {
                return {i, j};
            }
        }
    }
    return std::make_pair(-1, -1); // Return (-1, -1) if no valid index is found
}

void CoveragePlanner::generate()
{
    auto [x, y] = get_index();
    std::vector<cv::Point2f> goal;
    goal.clear();
    if (x == -1 && y == -1)
    { // Assuming -1 is the invalid index
        return;
    }
    goal.push_back(points_[x][y].first);
    mark_visited(x, y);

    int i = 1;
    while (true)
    {
        if (points_[x][y + i].second == 255)
        {
            y += i;
            if (x > 0 && points_[x - 1][y].second == 255)
            {
                goal_points_.push_back(goal);
                return generate(); // Continue generating with new goal points
            }
            goal.push_back(points_[x][y].first);
            mark_visited(x, y);
        }
        else
        {
            if (x + 1 < static_cast<int>(points_.size()) && points_[x + 1][y].second == 255)
            {
                x += 1;
                goal.push_back(points_[x][y].first);
                mark_visited(x, y);
                i = (i == 1) ? -1 : 1;
            }
            else
            {
                goal_points_.push_back(goal);
                return generate(); // Continue generating with new goal points
            }
        }
    }
}

void CoveragePlanner::mark_visited(int x, int y)
{
    points_[x][y].second = 0; // Mark as visited
}

std::vector<std::vector<cv::Point2f>> CoveragePlanner::generate_map_points(const std::vector<std::vector<cv::Point2f>> &goal_points)
{
    std::string msg_data;
    std::vector<std::vector<cv::Point2f>> map_points;

    for (const auto &path : goal_points)
    {
        std::vector<cv::Point2f> map_path_points;
        for (const auto &point : path)
        {
            cv::Point2f map_coords = pixel_to_map_coordinates(point.x, point.y);
            map_path_points.push_back(map_coords);
        }
        map_points.push_back(map_path_points);
    }

    // Convert map_points to a string representation and publish
    // Example using ROS message (adjust according to actual ROS setup)
    // std_msgs::msg::String msg;
    // msg.data = to_string(map_points); // Implement to_string for serialization
    // points_publisher_->publish(msg);

    return map_points;
}

std::vector<std::vector<cv::Point2f>> CoveragePlanner::optimize_points()
{
    std::vector<std::vector<cv::Point2f>> final_points;
    for (const auto &goal : goal_points_)
    {
        std::vector<cv::Point2f> points;
        bool add = true;

        for (size_t index = 0; index < goal.size(); ++index)
        {
            if (index + 1 < goal.size() && goal[index].x == goal[index + 1].x)
            {
                if (add)
                {
                    points.push_back(goal[index]);
                    add = false;
                }
            }
            else
            {
                points.push_back(goal[index]);
                add = true;
            }
        }
        final_points.push_back(points);
    }
    return final_points;
}

void CoveragePlanner::display_map(const cv::Mat& map_image) {
    cv::namedWindow("Modified Map", cv::WINDOW_NORMAL); // Make the window resizable
    cv::imshow("Modified Map", map_image);           // Display the modified grayscale map
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void CoveragePlanner::start(){
    for (auto i = 0; i < static_cast<int>(goal_points_.size()); ++i)
    {
        for (auto j = 0; j < static_cast<int>(goal_points_[i].size()); ++j)
        {
            std::cout << goal_points_[i][j] << std::endl;
            set_goal(goal_points_[i][j].x, goal_points_[i][j].y);
            do
            {
                std::cout << get_feedback() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Prevent busy-waiting
            } while (get_feedback() >= 0.15);

            cancel_goal();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // while(get_feedback() >= 0.15){
            //     rclcpp::spin_some(node_);
            // }
            
        }
        std::cout << std::endl
                    << std::endl;
    }
}