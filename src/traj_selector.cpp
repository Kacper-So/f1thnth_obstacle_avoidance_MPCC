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
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "freespace_planning_algorithms/astar_search.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>
#include <tuple>
#include <queue>
#include <unordered_map>

using namespace std::placeholders;

class TrajSelector : public rclcpp::Node {
public:
    TrajSelector() : Node("Traj_selector") {
        // Declare parameters
        this->declare_parameter<bool>("only_behind_solutions", false);
        this->declare_parameter<bool>("use_back", false);
        this->declare_parameter<double>("distance_heuristic_weight", 1.0);
        this->declare_parameter<double>("time_limit", 10000.0); // in milliseconds
        this->declare_parameter<double>("minimum_turning_radius", 9.0);
        this->declare_parameter<double>("maximum_turning_radius", 9.0);
        this->declare_parameter<int>("turning_radius_size", 1);
        this->declare_parameter<int>("theta_size", 144);
        this->declare_parameter<double>("curve_weight", 1.0);
        this->declare_parameter<double>("reverse_weight", 1.0);
        this->declare_parameter<double>("lateral_goal_range", 0.5);
        this->declare_parameter<double>("longitudinal_goal_range", 2.0);
        this->declare_parameter<double>("angle_goal_range", 6.0);
        this->declare_parameter<int>("obstacle_threshold", 100);
        this->declare_parameter<double>("vehicle_length", 0.48);
        this->declare_parameter<double>("vehicle_width", 0.25);
        this->declare_parameter<double>("vehicle_base2back", 0.33);
        this->declare_parameter<double>("out_time", 1.);

        // Get parameter values
        astarParam.only_behind_solutions = this->get_parameter("only_behind_solutions").as_bool();
        astarParam.use_back = this->get_parameter("use_back").as_bool();
        astarParam.distance_heuristic_weight = this->get_parameter("distance_heuristic_weight").as_double();

        plannerCommonParam.time_limit = this->get_parameter("time_limit").as_double();
        plannerCommonParam.minimum_turning_radius = this->get_parameter("minimum_turning_radius").as_double();
        plannerCommonParam.maximum_turning_radius = this->get_parameter("maximum_turning_radius").as_double();
        plannerCommonParam.turning_radius_size = this->get_parameter("turning_radius_size").as_int();
        plannerCommonParam.theta_size = this->get_parameter("theta_size").as_int();
        plannerCommonParam.curve_weight = this->get_parameter("curve_weight").as_double();
        plannerCommonParam.reverse_weight = this->get_parameter("reverse_weight").as_double();
        plannerCommonParam.lateral_goal_range = this->get_parameter("lateral_goal_range").as_double();
        plannerCommonParam.longitudinal_goal_range = this->get_parameter("longitudinal_goal_range").as_double();
        plannerCommonParam.angle_goal_range = this->get_parameter("angle_goal_range").as_double();
        plannerCommonParam.obstacle_threshold = this->get_parameter("obstacle_threshold").as_int();

        vehicleShape.length = this->get_parameter("vehicle_length").as_double();
        vehicleShape.width = this->get_parameter("vehicle_width").as_double();
        vehicleShape.base2back = this->get_parameter("vehicle_base2back").as_double();
        out_time = this->get_parameter("out_time").as_double();

        // Subscription to topics
        racing_planner_traj_sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
            "/traj", 10,
            std::bind(&TrajSelector::racingPlannerTrajectoryCallback, this, _1));

        start_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/start_point", 10,
            std::bind(&TrajSelector::startCallback, this, _1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/goal_point", 10,
            std::bind(&TrajSelector::goalCallback, this, _1));

        OG_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_occupancy_grid", 10,
            std::bind(&TrajSelector::OGCallback, this, _1));

        obstacle_detected_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/obstacle_alarm", 10,
            std::bind(&TrajSelector::obstacleDetectedCallback, this, _1));

        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TrajSelector::odometryCallback, this, std::placeholders::_1));

        // Publisher
        avoidance_traj_pub_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
            "/planning/racing_planner/avoidance/trajectory", 10);

        waypoint_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/waypoint_pointcloud", 10);

        clock_ = this->get_clock();
        last_callback_time_ = clock_->now();

        // Create Astar object
        astar = std::make_shared<freespace_planning_algorithms::AstarSearch>(plannerCommonParam, vehicleShape, astarParam);
    }

private:
    freespace_planning_algorithms::AstarParam astarParam;
    freespace_planning_algorithms::PlannerCommonParam plannerCommonParam;
    freespace_planning_algorithms::VehicleShape vehicleShape;
    std::shared_ptr<freespace_planning_algorithms::AstarSearch> astar;
    autoware_auto_planning_msgs::msg::Trajectory astar_traj;
    freespace_planning_algorithms::PlannerWaypoints astar_waypoints;
    geometry_msgs::msg::Pose start;
    geometry_msgs::msg::Pose goal;
    double out_time;
    bool obstacle_detected = false;
    rclcpp::Time last_callback_time_;
    double accumulated_time_ = 0.0;
    rclcpp::Clock::SharedPtr clock_;
    nav_msgs::msg::OccupancyGrid OG;
    nav_msgs::msg::Odometry::SharedPtr curr_odometry;

    void startCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        start = *msg;
    }

    void goalCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        goal = *msg;
    }

    void OGCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        OG = *msg;
    }

    void racingPlannerTrajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) {
        rclcpp::Time current_time = clock_->now();
        rclcpp::Duration duration = current_time - last_callback_time_;
        double dt = duration.seconds();
        accumulated_time_ += dt;
        last_callback_time_ = current_time;
        // RCLCPP_INFO(this->get_logger(), "accumulated_time: %f", accumulated_time_);
        RCLCPP_INFO(this->get_logger(), "outtime: %f", out_time);
        if (accumulated_time_ > out_time){
            obstacle_detected = false;
            accumulated_time_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "ref traj publish");
            avoidance_traj_pub_->publish(*msg);
        } else {
            if(obstacle_detected){
                astar->setMap(OG);
                if(astar->makePlan(start, goal)){
                    astar_waypoints = astar->getWaypoints();
                    convertWaypointsToTrajectory(astar_waypoints, astar_traj);
                    avoidance_traj_pub_->publish(astar_traj);
                }
            }
        }
    }

    void obstacleDetectedCallback(const std_msgs::msg::String::SharedPtr msg) {
        obstacle_detected = true;
        accumulated_time_ = 0.0;
        last_callback_time_ = clock_->now();
    }

    void convertWaypointsToTrajectory(const freespace_planning_algorithms::PlannerWaypoints& waypoints, autoware_auto_planning_msgs::msg::Trajectory& traj) {
        traj.header = waypoints.header;
        traj.points.clear();
        std::vector<geometry_msgs::msg::Point> waypoint_points;
        for (const auto& wp : waypoints.waypoints) {
            autoware_auto_planning_msgs::msg::TrajectoryPoint point;
            point.time_from_start.sec = 0;
            point.time_from_start.nanosec = 0;
            point.pose = wp.pose.pose;
            point.pose.position.x += curr_odometry.x;
            point.pose.position.y += curr_odometry.y;
            traj.points.push_back(point);
            waypoint_points.push_back(point.pose.position);
        }
        publishPointCloud(waypoint_pointcloud_pub_, waypoint_points);
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
        curr_odometry = odometry_msg;
    }

    void publishPointCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
                        const std::vector<geometry_msgs::msg::Point>& points) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (const auto& point : points) {
            pcl::PointXYZ pcl_point;
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = point.z; // Adjust z value if needed
            cloud->push_back(pcl_point);
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);

        output.header.frame_id = "map"; // Adjust frame ID if needed
        output.header.stamp = this->now();

        publisher->publish(output);
    }

    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr racing_planner_traj_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr OG_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obstacle_detected_sub_;
    rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr avoidance_traj_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr waypoint_pointcloud_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajSelector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
