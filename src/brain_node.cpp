/*
Brain (master) node.
Flow:
  1. Activate Nav2 (lifecycle STARTUP)
  2. Wait until the navigate_to_pose action server is up
  3. Publish initial pose for AMCL on /initialpose
  4. Call find_april to get the midpoint pose (expected in 'map' frame)
  5. Send the goal to Nav2 in 'map' frame
  6. After arrival, call find_tables and print results
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

#include "group11_assignament_1/srv/find_april.hpp"
#include "group11_assignament_1/srv/find_tables.hpp"

#include <cmath> 
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class BrainNode : public rclcpp::Node {
  public:
    BrainNode() : Node("brain_node") {
      
        // --- Clients for the two custom services ---
        client_detection = this->create_client<group11_assignament_1::srv::FindApril>("find_april");
        client_tables    = this->create_client<group11_assignament_1::srv::FindTables>("find_tables");

        // --- Clients for Nav2's lifecycle manager ---
        client_lifecycle = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");

        client_lifecycle_loc = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_localization/manage_nodes");

        // --- Action client for Nav2 ---
        client_nav = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // --- Publisher for AMCL initial pose ---
        initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        // --- Bootstrap timer (runs once) ---
        timer_ = this->create_wall_timer(10s, std::bind(&BrainNode::startup, this));
    }

  private:
    rclcpp::Client<group11_assignament_1::srv::FindApril>::SharedPtr client_detection;
    rclcpp::Client<group11_assignament_1::srv::FindTables>::SharedPtr client_tables;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_lifecycle;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_nav;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_lifecycle_loc;

    // --- One-shot retry timer (replaces std::thread().detach()) ---
    rclcpp::TimerBase::SharedPtr retry_timer_;


    // =======================================================================
    // STEP 0: bootstrap
    // =======================================================================
    void startup()
    {
        timer_->cancel();
        activate_nav2();
    }


    // =======================================================================
    // STEP 1: activate Nav2 (STARTUP) and wait until it's up
    // =======================================================================
    void activate_nav2()
    {
      // --- Activate LOCALIZATION (map_server + amcl) ---
      while (!client_lifecycle_loc->wait_for_service(1s)) {
          if (!rclcpp::ok()) return;
          RCLCPP_INFO(this->get_logger(), "Localization lifecycle not available, waiting...");
      }

      RCLCPP_INFO(this->get_logger(), "Sending STARTUP to localization...");
      auto req_loc = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
      req_loc->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
      client_lifecycle_loc->async_send_request(req_loc);
  
      // Wait until AMCL becomes active (few seconds)
      rclcpp::sleep_for(3s);
  
      // --- Publish initial pose (AMCL needs to be active)
      set_initial_pose(0.0, 0.0, 0.0);
      rclcpp::sleep_for(5s);  // processing time for AMCL
  
      // --- Activate NAVIGATION
      while (!client_lifecycle->wait_for_service(1s)) {
          if (!rclcpp::ok()) return;
          RCLCPP_INFO(this->get_logger(), "Navigation lifecycle not available, waiting...");
      }
  
      RCLCPP_INFO(this->get_logger(), "Sending STARTUP to navigation...");
      auto req_nav = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
      req_nav->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
      client_lifecycle->async_send_request(req_nav);
  
      // Wait until navigate_to_pose sia is available
      RCLCPP_INFO(this->get_logger(), "Waiting for navigate_to_pose action server...");
      if (!client_nav->wait_for_action_server(30s)) {
          RCLCPP_ERROR(this->get_logger(), "Nav2 didn't come up in time");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Nav2 is fully active!");
  
      send_april_request();
    }


    // =======================================================================
    // Publish the initial pose to AMCL (one-shot on /initialpose)
    // =======================================================================
    void set_initial_pose(double x, double y, double yaw)
    {
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header.frame_id = "map";
      msg.header.stamp = this->now();
      msg.pose.pose.position.x = x;
      msg.pose.pose.position.y = y;
      msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
      msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
      msg.pose.covariance[0]  = 0.25;
      msg.pose.covariance[7]  = 0.25;
      msg.pose.covariance[35] = 0.07;
      initial_pose_pub->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Initial pose published: x=%.2f y=%.2f yaw=%.2f", x, y, yaw);
    }


    // =======================================================================
    // STEP 2: ask find_april for the midpoint pose (already in 'map')
    // =======================================================================
    void send_april_request(int attempt = 1, int max_attempts = 10)
    {
      if (attempt > max_attempts) {   // After max_attemps stop sending requests
          RCLCPP_ERROR(this->get_logger(), 
              "find_april failed after %d attempts", max_attempts);
          return;
      }
  
      while (!client_detection->wait_for_service(1s)) {
          if (!rclcpp::ok()) return;
      }
  
      RCLCPP_INFO(this->get_logger(), "Requesting midpoint from find_april (attempt %d/%d)...", attempt, max_attempts);
  
      auto request = std::make_shared<group11_assignament_1::srv::FindApril::Request>();
  
      auto callback =
          [this, attempt, max_attempts]
          (rclcpp::Client<group11_assignament_1::srv::FindApril>::SharedFuture future)
          {
              auto response = future.get();
              if (!response->success) {
                  RCLCPP_WARN(this->get_logger(), "find_april failed, retrying in 2s...");
  
                  // One-shot timer instead of std::thread().detach()
                  retry_timer_ = this->create_wall_timer(
                      2s,
                      [this, attempt, max_attempts]() {
                          retry_timer_->cancel();  // make it fire only once
                          send_april_request(attempt + 1, max_attempts);
                      });
                  return;
              }

            
              RCLCPP_INFO(this->get_logger(),
                  "Midpoint received in frame '%s': (%.2f, %.2f)",
                  response->pose.header.frame_id.c_str(),
                  response->pose.pose.position.x,
                  response->pose.pose.position.y);
  
              send_nav_goal(response->pose);
          };
  
      client_detection->async_send_request(request, callback);
    }


    // =======================================================================
    // STEP 3: send the goal to Nav2 in 'map' frame
    // =======================================================================
    void send_nav_goal(geometry_msgs::msg::PoseStamped pose)
    {
        NavigateToPose::Goal goal_msg;
        goal_msg.pose = pose;
        goal_msg.pose.header.frame_id = "map";  // explicit: goal is in map
        goal_msg.pose.header.stamp = this->now();

        RCLCPP_INFO(this->get_logger(), "Sending Nav2 goal in 'map': (%.2f, %.2f)", goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        // --- Goal
        send_goal_options.goal_response_callback =
          [this, pose](const GoalHandleNav::SharedPtr & goal_handle)
          {
              if (!goal_handle) {
                  RCLCPP_WARN(this->get_logger(), 
                      "Goal rejected by Nav2, retrying in 3s...");
      
                  // One-shot timer instead of std::thread().detach()
                  retry_timer_ = this->create_wall_timer(
                      3s,
                      [this, pose]() {
                          retry_timer_->cancel();  // make it fire only once
                          send_nav_goal(pose);     // retry same goal
                      });
                  return;
              }
              RCLCPP_INFO(this->get_logger(), "Goal accepted, robot moving...");
          };

        // --- Feedback
        send_goal_options.feedback_callback =
          [this](GoalHandleNav::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback)
            {};

        // --- Result
        send_goal_options.result_callback =
          [this](const GoalHandleNav::WrappedResult & result)
          {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Destination reached!");
                        request_tables();
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "Navigation canceled");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        break;
                }
            };

        client_nav->async_send_goal(goal_msg, send_goal_options);
     }


    // =======================================================================
    // STEP 4: ask find_tables and print
    // =======================================================================
    void request_tables()
    {
        while (!client_tables->wait_for_service(1s)) {   // Wait for the node
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(this->get_logger(), "find_tables not available, waiting...");
        }

        RCLCPP_INFO(this->get_logger(), "Asking find_tables...");
        auto request = std::make_shared<group11_assignament_1::srv::FindTables::Request>();

        auto callback =
            [this](rclcpp::Client<group11_assignament_1::srv::FindTables>::SharedFuture future)
            {
                auto response = future.get();
                if (!response->success) {
                    RCLCPP_WARN(this->get_logger(), "Table detection failed");
                    return;
                }

                RCLCPP_INFO(this->get_logger(),
                    "Found %ld tables:", response->tables.poses.size());
                for (size_t i = 0; i < response->tables.poses.size(); i++) {
                    const auto &p = response->tables.poses[i].position;
                    RCLCPP_INFO(this->get_logger(),
                        "  Table %ld -> x: %.2f, y: %.2f", i, p.x, p.y);
                }
            };

        client_tables->async_send_request(request, callback);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrainNode>());
    rclcpp::shutdown();
    return 0;
}
