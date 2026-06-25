/*
This node (working as a server for the service) will receive a service request
and respond with a position, that is the goal position, the one between the
two apriltags.
It recieve the pose of the apriltags from the camera from the /tf topic,
like said in the documentation of apriltag_ros
*/

#include <rclcpp/rclcpp.hpp>
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "group11_assignament_1/srv/find_april.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AprilDetectionNode : public rclcpp::Node {
  public:
    AprilDetectionNode() : Node("april_detection_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        
        // --- Subscriber to detections (still used only to know if we received tags), dont know if still useful ---
        subscriber = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>("/apriltag/detections", 10, std::bind(&AprilDetectionNode::scanCallback, this, _1));

        // --- Server for the custom service ---
        service = this->create_service<group11_assignament_1::srv::FindApril>("find_april", std::bind(&AprilDetectionNode::handleService, this, _1, _2));
    }
    
    
  private:
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscriber;
    rclcpp::Service<group11_assignament_1::srv::FindApril>::SharedPtr service;

    apriltag_msgs::msg::AprilTagDetectionArray latest_detection_;
    bool got_detection_ = false;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // =======================================================================
    // Get the response from the camera
    // =======================================================================
    void scanCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        latest_detection_ = *msg;
        got_detection_ = true;
    }
    
    // =======================================================================
    // Create and send the service response back to the Brain node
    // =======================================================================
    void handleService(
        const std::shared_ptr<group11_assignament_1::srv::FindApril::Request> request,
        std::shared_ptr<group11_assignament_1::srv::FindApril::Response> response)
    {
        if(!got_detection_) {
            RCLCPP_WARN(this->get_logger(), "No AprilTag detections");
            response->success = false;
            return;
        }


        int id1 = 1;
        int id2 = 10;

        // --- Frame names published by the apriltag node
        std::string frame1 = "tag36h11:" + std::to_string(id1);
        std::string frame2 = "tag36h11:" + std::to_string(id2);

        geometry_msgs::msg::TransformStamped tf1, tf2;

        try {
            tf1 = tf_buffer_.lookupTransform("external_camera/link/rgb_camera", frame1, tf2::TimePointZero);
            tf2 = tf_buffer_.lookupTransform("external_camera/link/rgb_camera", frame2, tf2::TimePointZero);
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", e.what());
            response->success = false;
            return;
        }

        
        double x1 = tf1.transform.translation.x;
        double y1 = tf1.transform.translation.y;
        double z1 = tf1.transform.translation.z;

        double x2 = tf2.transform.translation.x;
        double y2 = tf2.transform.translation.y;
        double z2 = tf2.transform.translation.z;

        // --- Finding a point in between apriltags
        geometry_msgs::msg::PoseStamped midpoint;
        midpoint.header.stamp = rclcpp::Time(0);
        midpoint.header.frame_id = "external_camera/link/rgb_camera";
        midpoint.pose.position.x = (x1 + x2) / 2.0;
        midpoint.pose.position.y = (y1 + y2) / 2.0;
        midpoint.pose.position.z = (z1 + z2) / 2.0;
        midpoint.pose.orientation.w = 1.0;
        
        // --- Transforming the midpoint in map frame
        geometry_msgs::msg::PoseStamped midpoint_map_pose;
        try {
            midpoint_map_pose = tf_buffer_.transform(midpoint, "map", tf2::durationFromSec(0.1));
        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF transform to map failed: %s", ex.what());
            response->success = false;
            return;
        }

        response->pose = midpoint_map_pose;
        response->success = true;
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
