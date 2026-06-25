/*
This node (working as a server for the service) will receive a service request
and respond with a vector of positions, that are the positions of the three
tables in the room.
It recieve laserscans of the environment from the LiDAR
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "cmath"
#include "vector"
#include "memory"

#include "group11_assignament_1/srv/find_tables.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TableDetectionNode : public rclcpp::Node {
  public:
    TableDetectionNode() : Node("table_detection_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {

        // --- Subscription to the LiDAR ---
        lidar = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&TableDetectionNode::scanCallback, this, _1));

        // --- Publisher for the vector of tables poses ---
        publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/tables", 10);

        // --- Server for the custom service ---
        service = this->create_service<group11_assignament_1::srv::FindTables>("find_tables", std::bind(&TableDetectionNode::handleService, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Table-Detection node waiting for a request");
    }

  private:
    struct Cluster {   // Struct to identify a candidate detected table
        int start_index;
        int end_index;
        float avg_range;
    };

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher;
    rclcpp::Service<group11_assignament_1::srv::FindTables>::SharedPtr service;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<geometry_msgs::msg::Pose> detected_tables_;

    // =======================================================================
    // Manage the scan returned from the LiDAR 
    // =======================================================================
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        detected_tables_.clear();
        auto clusters = clusterLaser(msg);

        for (auto &cl : clusters) {
            if (isTable(msg, cl)) {
                auto pose = computePose(msg, cl);
                detected_tables_.push_back(pose);
            }
        }

        geometry_msgs::msg::PoseArray tables;
        tables.header.stamp = this->now();
        tables.header.frame_id = "odom";
        tables.poses = detected_tables_;
        publisher->publish(tables);
    }

    // =======================================================================
    // Create a cluster from a valid LaserScan
    // =======================================================================
    std::vector<Cluster> clusterLaser(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        
        std::vector<Cluster> clusters;
        const auto &ranges = scan->ranges;
        const float thresh = 0.15;
        const int min_points = 3;
        const int max_points = 100;

        int start = -1;

        for (size_t i = 0; i < ranges.size(); i++) {
            float r = ranges[i];

            if (!std::isfinite(r)) {
                if (start >= 0) {
                    int n = (int)i - start;
                    if (n >= min_points && n <= max_points) {
                        Cluster c;
                        c.start_index = start;
                        c.end_index   = (int)i - 1;
                        c.avg_range   = computeAvgRange(ranges, start, (int)i - 1);
                        clusters.push_back(c);
                    }
                    start = -1;
                }
                continue;
            }

            if (start < 0) {
                start = (int)i;
                continue;
            }

            if (std::fabs(r - ranges[i - 1]) > thresh && std::isfinite(ranges[i - 1])) {
                int n = (int)i - start;
                if (n >= min_points && n <= max_points) {
                    Cluster c;
                    c.start_index = start;
                    c.end_index   = (int)i - 1;
                    c.avg_range   = computeAvgRange(ranges, start, (int)i - 1);
                    clusters.push_back(c);
                }
                start = (int)i;
            }
        }

        if (start >= 0) {
            int n = (int)ranges.size() - start;
            if (n >= min_points && n <= max_points) {
                Cluster c;
                c.start_index = start;
                c.end_index   = (int)ranges.size() - 1;
                c.avg_range   = computeAvgRange(ranges, start, (int)ranges.size() - 1);
                clusters.push_back(c);
            }
        }

        return clusters;
    }

    // =======================================================================
    // HELPER - compute the average among the ranges
    // =======================================================================
    float computeAvgRange(const std::vector<float> &ranges, int a, int b)
    {
        float sum = 0;
        int n = 0;
        for (int i = a; i <= b; i++) {
            if (std::isfinite(ranges[i])) {
                sum += ranges[i];
                n++;
            }
        }
        return n > 0 ? sum / n : 0.0;
    }

    // =======================================================================
    // Distinguish if a detected cluster is a table or not
    // =======================================================================
    bool isTable(const sensor_msgs::msg::LaserScan::SharedPtr scan, const Cluster &cl) {
        int mid = (cl.start_index + cl.end_index) / 2;

        // --- Take right-most, left-most and central points
        float center = scan->ranges[mid];
        float left   = scan->ranges[cl.start_index];
        float right  = scan->ranges[cl.end_index];

        if (!std::isfinite(center) || !std::isfinite(left) || !std::isfinite(right))
            return false;

        // --- Look at convexity
        return (left > center + 0.005f) && (right > center + 0.005f);
    }

    // =======================================================================
    // Compute the pose of a cluster (table)
    // =======================================================================
    geometry_msgs::msg::Pose computePose(const sensor_msgs::msg::LaserScan::SharedPtr scan, const Cluster &cl){
        
        int center = (cl.start_index + cl.end_index) / 2;
        float r = scan->ranges[center];
        float angle = scan->angle_min + center * scan->angle_increment;

        geometry_msgs::msg::PoseStamped p_laser;
        p_laser.header.stamp = rclcpp::Time(0);
        p_laser.header.frame_id = scan->header.frame_id;
        p_laser.pose.position.x = r * cos(angle);
        p_laser.pose.position.y = r * sin(angle);
        p_laser.pose.position.z = 0.0;
        p_laser.pose.orientation.w = 1.0;

        // --- Transforming the table position in odom frame
        geometry_msgs::msg::PoseStamped p_odom;
        try {
            p_odom = tf_buffer_.transform(p_laser, "odom", tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF transform to odom failed: %s. Returning laser-frame pose.", ex.what());
            return p_laser.pose;
        }

        return p_odom.pose;
    }

    // =======================================================================
    // Create and send the service response back to the Brain node
    // =======================================================================
    void handleService(const std::shared_ptr<group11_assignament_1::srv::FindTables::Request> request, std::shared_ptr<group11_assignament_1::srv::FindTables::Response> response) {
        (void)request;

        response->tables.header.stamp = this->now();
        response->tables.header.frame_id = "odom";
        response->tables.poses = detected_tables_;
        response->success = true;

        RCLCPP_INFO(this->get_logger(), "Service request: detected_tables=%ld", detected_tables_.size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TableDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
