#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>

class RockStacking : public rclcpp::Node {
private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gcode_publisher_;
  bool processing_rock_;

public:
  RockStacking() : Node("rock_stacking")
  {
    // Declare parameters
    this->declare_parameter("center_x", 0.0);
    this->declare_parameter("center_y", 0.0);
    this->declare_parameter("center_z", 0.1); // Some height above the table
    this->declare_parameter("approach_height", 0.05); // Height to approach rocks
    this->declare_parameter("pick_height", 0.01); // Height to pick at
    this->declare_parameter("feedrate", 1000); // Feedrate for G-code
    this->declare_parameter("gcode_file", "rock_pick_place.gcode"); // Output file name
    this->declare_parameter("gripper_open_command", "M8"); // Command to open gripper
    this->declare_parameter("gripper_close_command", "M9"); // Command to close gripper
    this->declare_parameter("homing_command", "G28"); // Command for homing
    this->declare_parameter("pick_wait_time", 3); // Time to wait after picking (seconds)
    this->declare_parameter("home_wait_time", 10); // Time to wait at home position (seconds)
    
    // Physical limits of the robot
    this->declare_parameter("x_min", -60.0); // Minimum X coordinate (mm)
    this->declare_parameter("x_max", 60.0);  // Maximum X coordinate (mm)
    this->declare_parameter("y_min", -60.0); // Minimum Y coordinate (mm)
    this->declare_parameter("y_max", 60.0);  // Maximum Y coordinate (mm)
    this->declare_parameter("z_min", 0.0);   // Minimum Z coordinate (mm)
    this->declare_parameter("z_max", 100.0); // Maximum Z coordinate (mm)

    // Subscribe to rock centers from rock detector
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "rock_centers", 10, std::bind(&RockStacking::centers_callback, this, std::placeholders::_1));

    // Publisher for G-code commands
    gcode_publisher_ = this->create_publisher<std_msgs::msg::String>("gcode_commands", 10);
    
    // Flag to track whether we're currently processing a rock
    processing_rock_ = false;

    // Send homing command at startup
    send_homing_command();

    RCLCPP_INFO(this->get_logger(), "Rock Stacking node has started");
  }

private:
  void send_homing_command()
  {
    std::string homing_command = this->get_parameter("homing_command").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Sending homing command: %s", homing_command.c_str());
    
    // Publish the homing command
    std_msgs::msg::String msg;
    msg.data = homing_command;
    gcode_publisher_->publish(msg);
    
    // Wait for homing to complete (you might want to implement feedback in a real system)
    std::this_thread::sleep_for(std::chrono::seconds(5)); 
    
    RCLCPP_INFO(this->get_logger(), "Homing completed. Ready to process rocks.");
    processing_rock_ = false;
  }

  void centers_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    // Skip if we're already processing a rock
    if (processing_rock_) {
      RCLCPP_INFO(this->get_logger(), "Already processing a rock, skipping new detection");
      return;
    }
    
    if (msg->poses.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty PoseArray, no rocks detected");
      return;
    }

    processing_rock_ = true;
    RCLCPP_INFO(this->get_logger(), "Starting new rock processing sequence");
    
    // Get parameters
    double center_x = this->get_parameter("center_x").as_double();
    double center_y = this->get_parameter("center_y").as_double();
    double center_z = this->get_parameter("center_z").as_double();
    double approach_height = this->get_parameter("approach_height").as_double();
    double pick_height = this->get_parameter("pick_height").as_double();
    int feedrate = this->get_parameter("feedrate").as_int();
    std::string gcode_file = this->get_parameter("gcode_file").as_string();
    std::string gripper_open = this->get_parameter("gripper_open_command").as_string();
    std::string gripper_close = this->get_parameter("gripper_close_command").as_string();
    int pick_wait_time = this->get_parameter("pick_wait_time").as_int();
    int home_wait_time = this->get_parameter("home_wait_time").as_int();
    
    // Get physical limits
    double x_min = this->get_parameter("x_min").as_double();
    double x_max = this->get_parameter("x_max").as_double();
    double y_min = this->get_parameter("y_min").as_double();
    double y_max = this->get_parameter("y_max").as_double();
    double z_min = this->get_parameter("z_min").as_double();
    double z_max = this->get_parameter("z_max").as_double();

    // Find the closest rock
    size_t closest_index = find_closest_rock(msg, center_x, center_y);
    double rock_x = msg->poses[closest_index].position.x;
    double rock_y = msg->poses[closest_index].position.y;
    double rock_z = msg->poses[closest_index].position.z;

    // Convert point coordinates to machine coordinates (mm)
    double x_machine = rock_x * 1000.0; // Convert to mm
    double y_machine = rock_y * 1000.0; // Convert to mm
    
    // Check if rock position is within robot limits
    bool rock_in_limits = (x_machine >= x_min && x_machine <= x_max &&
                           y_machine >= y_min && y_machine <= y_max);
                           
    // Check if center position is within robot limits
    double center_x_mm = center_x * 1000.0; // Convert to mm
    double center_y_mm = center_y * 1000.0; // Convert to mm
    double center_z_mm = center_z * 1000.0; // Convert to mm
    
    bool center_in_limits = (center_x_mm >= x_min && center_x_mm <= x_max &&
                           center_y_mm >= y_min && center_y_mm <= y_max &&
                           center_z_mm >= z_min && center_z_mm <= z_max);
    
    if (!rock_in_limits)
    {
      RCLCPP_WARN(this->get_logger(), 
                "Rock position (%.3f, %.3f) is outside robot limits [X: %.1f to %.1f, Y: %.1f to %.1f]", 
                x_machine, y_machine, x_min, x_max, y_min, y_max);
      processing_rock_ = false;
      return;
    }
    
    if (!center_in_limits)
    {
      RCLCPP_WARN(this->get_logger(), 
                "Center position (%.3f, %.3f, %.3f) is outside robot limits [X: %.1f to %.1f, Y: %.1f to %.1f, Z: %.1f to %.1f]", 
                center_x_mm, center_y_mm, center_z_mm, x_min, x_max, y_min, y_max, z_min, z_max);
      processing_rock_ = false;
      return;
    }
    
    // Generate G-code for picking and placing
    std::vector<std::string> gcode_commands;
    
    // G-code preamble
    gcode_commands.push_back("; Rock pick and place operation");
    gcode_commands.push_back("; Generated by rock_stacking node");
    gcode_commands.push_back("; Robot limits: X[" + std::to_string(x_min) + ":" + std::to_string(x_max) + 
                            "], Y[" + std::to_string(y_min) + ":" + std::to_string(y_max) + 
                            "], Z[" + std::to_string(z_min) + ":" + std::to_string(z_max) + "]");
    gcode_commands.push_back("G21 ; Set units to millimeters");
    gcode_commands.push_back("G90 ; Use absolute positioning");
    
    // Step 1: Open gripper first
    gcode_commands.push_back(gripper_open + " ; Open gripper");
    
    // Step 2: Move to approach height
    double safe_z_approach = std::max(std::min(approach_height * 1000.0, z_max), z_min);
    gcode_commands.push_back("G0 Z" + std::to_string(safe_z_approach) + " F" + std::to_string(feedrate) + " ; Move to safe height");

    // Step 3: Move above the closest rock
    double safe_x = std::max(std::min(x_machine, x_max), x_min);
    double safe_y = std::max(std::min(y_machine, y_max), y_min);
    
    gcode_commands.push_back("G0 X" + std::to_string(safe_x) + " Y" + std::to_string(safe_y) + 
                             " F" + std::to_string(feedrate) + " ; Move above rock");
    
    RCLCPP_INFO(this->get_logger(), "Moving to closest rock at (%.1f, %.1f)", safe_x, safe_y);
    
    // Step 4: Move down to pick the rock
    double safe_z_pick = std::max(std::min(pick_height * 1000.0, z_max), z_min);
    gcode_commands.push_back("G1 Z" + std::to_string(safe_z_pick) + " F" + std::to_string(feedrate/2) + " ; Move down to pick");
    
    // Step 5: Close gripper to grab rock
    gcode_commands.push_back(gripper_close + " ; Close gripper to grab rock");
    
    // Step 6: Wait for the rock to be securely grasped
    gcode_commands.push_back("G4 P" + std::to_string(pick_wait_time * 1000) + " ; Wait for " + std::to_string(pick_wait_time) + " seconds");
    
    RCLCPP_INFO(this->get_logger(), "Picking rock, waiting %d seconds to secure grasp", pick_wait_time);
    
    // Step 7: Move back up to safe height
    gcode_commands.push_back("G0 Z" + std::to_string(safe_z_approach) + " F" + std::to_string(feedrate) + " ; Move up with rock");
    
    // Step 8: Move to center position
    double safe_center_x = std::max(std::min(center_x * 1000.0, x_max), x_min);
    double safe_center_y = std::max(std::min(center_y * 1000.0, y_max), y_min);
    
    gcode_commands.push_back("G0 X" + std::to_string(safe_center_x) + " Y" + std::to_string(safe_center_y) + 
                             " F" + std::to_string(feedrate) + " ; Move to center position");
    
    RCLCPP_INFO(this->get_logger(), "Moving to center position (%.1f, %.1f)", safe_center_x, safe_center_y);
    
    // Step 9: Move down to place the rock
    gcode_commands.push_back("G1 Z" + std::to_string(safe_z_pick) + " F" + std::to_string(feedrate/2) + " ; Move down to place");
    
    // Step 10: Open gripper to release rock
    gcode_commands.push_back(gripper_open + " ; Open gripper to release rock");
    
    // Step 11: Move back up to safe height
    gcode_commands.push_back("G0 Z" + std::to_string(safe_z_approach) + " F" + std::to_string(feedrate) + " ; Move back up to safe height");
    
    // Step 12: Return to home position
    gcode_commands.push_back(this->get_parameter("homing_command").as_string() + " ; Return to home position");
    
    RCLCPP_INFO(this->get_logger(), "Returning to home position");
    
    // Step 13: Wait at home position for specified time
    gcode_commands.push_back("G4 P" + std::to_string(home_wait_time * 1000) + " ; Wait for " + std::to_string(home_wait_time) + " seconds at home");
    
    RCLCPP_INFO(this->get_logger(), "Waiting at home position for %d seconds", home_wait_time);

    // Save G-code to file
    std::ofstream outfile(gcode_file);
    if (outfile.is_open())
    {
      for (const auto& cmd : gcode_commands)
      {
        outfile << cmd << std::endl;
      }
      outfile.close();
      RCLCPP_INFO(this->get_logger(), "G-code saved to %s", gcode_file.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open G-code output file");
    }

    // Execute G-code commands
    execute_gcode_sequence(gcode_commands);
  }
  
  size_t find_closest_rock(const geometry_msgs::msg::PoseArray::SharedPtr msg, double center_x, double center_y)
  {
    size_t closest_index = 0;
    double min_distance = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < msg->poses.size(); ++i)
    {
      double dx = msg->poses[i].position.x - center_x;
      double dy = msg->poses[i].position.y - center_y;
      double distance = std::sqrt(dx*dx + dy*dy);
      
      if (distance < min_distance)
      {
        min_distance = distance;
        closest_index = i;
      }
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "Closest rock found at (%.3f, %.3f, %.3f), distance: %.3f", 
                msg->poses[closest_index].position.x,
                msg->poses[closest_index].position.y,
                msg->poses[closest_index].position.z,
                min_distance);
                
    return closest_index;
  }
  
  void execute_gcode_sequence(const std::vector<std::string>& commands)
  {
    for (const auto& cmd : commands)
    {
      // Create message
      std_msgs::msg::String gcode_msg;
      gcode_msg.data = cmd;
      
      // Publish command
      gcode_publisher_->publish(gcode_msg);
      
      // Log the command being sent
      RCLCPP_INFO(this->get_logger(), "Sending G-code: %s", cmd.c_str());
      
      // Wait a short time between commands to allow for processing
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      
      // Add additional wait time for movement commands (G0, G1) to complete
      if (cmd.find("G0") != std::string::npos || cmd.find("G1") != std::string::npos) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      
      // Add additional wait time for wait commands (G4)
      if (cmd.find("G4") != std::string::npos) {
        // Extract the wait time from the command (P parameter is in milliseconds)
        size_t p_pos = cmd.find("P");
        if (p_pos != std::string::npos) {
          std::string p_value = cmd.substr(p_pos + 1);
          size_t space_pos = p_value.find(" ");
          if (space_pos != std::string::npos) {
            p_value = p_value.substr(0, space_pos);
          }
          try {
            int wait_ms = std::stoi(p_value);
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
          } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse wait time: %s", e.what());
          }
        }
      }
    }
    
    // After executing all commands, reset the processing flag to allow new operations
    processing_rock_ = false;
    RCLCPP_INFO(this->get_logger(), "Rock stacking sequence completed, ready for next rock");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RockStacking>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}