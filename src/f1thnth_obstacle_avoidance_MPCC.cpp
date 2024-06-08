#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "tf2/utils.h"
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>
#include <tuple>
#include <queue>
#include <unordered_map>

using namespace std::placeholders;

struct Cell {
    int x;
    int y;
    bool operator<(const Cell& other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }
    bool operator==(const Cell& other) const {
        return x == other.x && y == other.y;
    }
};

class TrajSelector : public rclcpp::Node {
public:
    TrajSelector() : Node("Traj_selector") {
        // Subscription to topics
        racing_planner_traj_sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
            "/planning/racing_planner/trajectory", 10,
            std::bind(&TrajSelector::racingPlannerTrajectoryCallback, this, _1));

        astar_traj_sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
            "/planning/astar/trajectory", 10,
            std::bind(&TrajSelector::astarCallback, this, _1));

        obstacle_detected_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/obstacle_alarm", 10,
            std::bind(&TrajSelector::obstacleDetectedCallback, this, _1));

        // Publisher
        avoidance_traj_pub_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
            "/planning/racing_planner/avoidance/trajectory", 10);

        clock_ = this->get_clock();
        last_callback_time_ = clock_->now();
    }

private:
    autoware_auto_planning_msgs::msg::Trajectory astar_traj;
    float out_time = 1.;
    bool obstacle_detected = false;
    rclcpp::Time last_callback_time_;
    double accumulated_time_ = 0.0;
    rclcpp::Clock::SharedPtr clock_;
    
    void astarCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg){
        astar_traj = *msg;
    }
    
    void racingPlannerTrajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) {
        rclcpp::Time current_time = clock_->now();
        rclcpp::Duration duration = current_time - last_callback_time_;
        double dt = duration.seconds();
        accumulated_time_ += dt;
        last_callback_time_ = current_time;

        trajectory_reference = *msg;
        if (obstacle_detected) {
            if (accumulated_time_ > out_time) {
                obstacle_detected = false;
                accumulated_time_ = 0.0;
            } else {
                avoidance_traj_pub_->publish(astar_traj);
            }
        } else {
            avoidance_traj_pub_->publish(*msg);
        }
    }

    void obstacleDetectedCallback(const std_msgs::msg::String::SharedPtr msg) {
        obstacle_detected = true;
        accumulated_time_ = 0.0;
        last_callback_time_ = clock_->now();
    }

    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr racing_planner_traj_sub_;
    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr astar_traj_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obstacle_detected_sub_;
    rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr avoidance_traj_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajSelector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
