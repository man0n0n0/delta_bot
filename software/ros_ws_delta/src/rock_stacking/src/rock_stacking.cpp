#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

class RockStacking : public rclcpp::Node
{
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

    RCLCPP_INFO(this->get_logger(), "Rock Stacking node has started");
  }

private:
  void centers_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty PoseArray, no rocks detected");
      return;
    }

    // Get the center position from parameters
    double center_x = this->get_parameter("center_x").as_double();
    double center_y = this->get_parameter("center_y").as_double();
    double center_z = this->get_parameter("center_z").as_double();
    double approach_height = this->get_parameter("approach_height").as_double();
    double pick_height = this->get_parameter("pick_height").as_double();
    int feedrate = this->get_parameter("feedrate").as_int();
    std::string gcode_file = this->get_parameter("gcode_file").as_string();
    std::string gripper_open = this->get_parameter("gripper_open_command").as_string();
    std::string gripper_close = this->get_parameter("gripper_close_command").as_string();
    
    // Get physical limits
    double x_min = this->get_parameter("x_min").as_double();
    double x_max = this->get_parameter("x_max").as_double();
    double y_min = this->get_parameter("y_min").as_double();
    double y_max = this->get_parameter("y_max").as_double();
    double z_min = this->get_parameter("z_min").as_double();
    double z_max = this->get_parameter("z_max").as_double();

    // Find the closest rock to the center
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

    // Get the closest rock position
    double rock_x = msg->poses[closest_index].position.x;
    double rock_y = msg->poses[closest_index].position.y;
    double rock_z = msg->poses[closest_index].position.z;

    RCLCPP_INFO(this->get_logger(), 
                "Closest rock found at (%.3f, %.3f, %.3f), distance: %.3f", 
                rock_x, rock_y, rock_z, min_distance);

    // Convert point coordinates to machine coordinates (mm)
    double x_machine = rock_x * 1000.0; // Convert to mm
    double y_machine = rock_y * 1000.0; // Convert to mm
    double z_machine_approach = approach_height * 1000.0; // Convert to mm
    double z_machine_pick = pick_height * 1000.0; // Convert to mm
    double center_x_mm = center_x * 1000.0; // Convert to mm
    double center_y_mm = center_y * 1000.0; // Convert to mm
    double center_z_mm = center_z * 1000.0; // Convert to mm
    
    // Check if rock position is within robot limits
    bool rock_in_limits = (x_machine >= x_min && x_machine <= x_max &&
                           y_machine >= y_min && y_machine <= y_max &&
                           z_machine_pick >= z_min && z_machine_approach <= z_max);
                           
    // Check if center position is within robot limits
    bool center_in_limits = (center_x_mm >= x_min && center_x_mm <= x_max &&
                             center_y_mm >= y_min && center_y_mm <= y_max &&
                             center_z_mm >= z_min && center_z_mm <= z_max);
    
    if (!rock_in_limits)
    {
      RCLCPP_WARN(this->get_logger(), 
                 "Rock position (%.3f, %.3f, %.3f) is outside robot limits [X: %.1f to %.1f, Y: %.1f to %.1f, Z: %.1f to %.1f]", 
                 x_machine, y_machine, z_machine_pick, x_min, x_max, y_min, y_max, z_min, z_max);
      return;
    }
    
    if (!center_in_limits)
    {
      RCLCPP_WARN(this->get_logger(), 
                 "Center position (%.3f, %.3f, %.3f) is outside robot limits [X: %.1f to %.1f, Y: %.1f to %.1f, Z: %.1f to %.1f]", 
                 center_x_mm, center_y_mm, center_z_mm, x_min, x_max, y_min, y_max, z_min, z_max);
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
    
    // Open gripper
    gcode_commands.push_back(gripper_open + " ; Open gripper");
    
    // Move to approach position above rock
    gcode_commands.push_back("G0 Z" + std::to_string(z_machine_approach) + " F" + std::to_string(feedrate) + " ; Move to safe height");
    
    // Apply limits to ensure we stay within robot's physical capabilities
    double safe_x = std::max(std::min(x_machine, x_max), x_min);
    double safe_y = std::max(std::min(y_machine, y_max), y_min);
    
    gcode_commands.push_back("G0 X" + std::to_string(safe_x) + " Y" + std::to_string(safe_y) + 
                             " F" + std::to_string(feedrate) + " ; Move above rock");
    
    // Move down to pick the rock - with limit check
    double safe_z_pick = std::max(std::min(z_machine_pick, z_max), z_min);
    gcode_commands.push_back("G1 Z" + std::to_string(safe_z_pick) + " F" + std::to_string(feedrate/2) + " ; Move down to pick");
    
    // Close gripper
    gcode_commands.push_back(gripper_close + " ; Close gripper to grab rock");
    
    // Move up with rock - with limit check
    double safe_z_approach = std::max(std::min(z_machine_approach, z_max), z_min);
    gcode_commands.push_back("G0 Z" + std::to_string(safe_z_approach) + " F" + std::to_string(feedrate) + " ; Move up with rock");
    
    // Move to center position - with limit check
    double safe_center_x = std::max(std::min(center_x_mm, x_max), x_min);
    double safe_center_y = std::max(std::min(center_y_mm, y_max), y_min);
    
    gcode_commands.push_back("G0 X" + std::to_string(safe_center_x) + " Y" + std::to_string(safe_center_y) + 
                             " F" + std::to_string(feedrate) + " ; Move to center position");
    
    // Move down to place the rock - with limit check
    gcode_commands.push_back("G1 Z" + std::to_string(safe_z_pick) + " F" + std::to_string(feedrate/2) + " ; Move down to place");
    
    // Open gripper to release
    gcode_commands.push_back(gripper_open + " ; Open gripper to release rock");
    
    // Move back up to safe height - with limit check
    gcode_commands.push_back("G0 Z" + std::to_string(safe_z_approach) + " F" + std::to_string(feedrate) + " ; Move back up to safe height");
    
    // Return to home position - ensuring it's within limits
    double home_x = 0.0;
    double home_y = 0.0;
    
    // Apply limits to home position if needed
    home_x = std::max(std::min(home_x, x_max), x_min);
    home_y = std::max(std::min(home_y, y_max), y_min);
    
    gcode_commands.push_back("G0 X" + std::to_string(home_x) + " Y" + std::to_string(home_y) + 
                             " F" + std::to_string(feedrate) + " ; Return to home position");
                             
    RCLCPP_INFO(this->get_logger(), "Generated G-code with robot limit constraints applied [X: %.1f to %.1f, Y: %.1f to %.1f, Z: %.1f to %.1f]",
                x_min, x_max, y_min, y_max, z_min, z_max);

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

    // Publish G-code commands
    for (const auto& cmd : gcode_commands)
    {
      std_msgs::msg::String gcode_msg;
      gcode_msg.data = cmd;
      gcode_publisher_->publish(gcode_msg);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gcode_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RockStacking>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}