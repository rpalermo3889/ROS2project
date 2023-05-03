#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>

// Lab 7 topics
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// end of Lab 7 topics

// Project code
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp> // Include the occupancy grid message type
#include <tf2_ros/transform_listener.h>
//#include <sensor_msgs/point_cloud2.hpp>
// End of project code

#include <iostream>

void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // Process the laser scan data
  // ...
}

void processScan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  std::vector<float>::const_iterator minval =
    min(msg->ranges.begin(),msg->ranges.end());
  std_msgs::msg::Float32 msg_to_send;
  msg_to_send.data = *minval;
  pubf->publish(msg_to_send);
}

// Define a callback function to process the map data
void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Get the size of the map
  int width = msg->info.width;
  int height = msg->info.height;

  // Loop over each cell in the map
  for (int i = 0; i < width * height; i++) {
    // Convert the map index to an (x,y) coordinate
    int x = i % width;
    int y = i / width;

    // Determine the occupancy value of the cell
    int val = msg->data[i];
    bool is_occupied = (val > 65);

    // If the cell is occupied, it might be a cylinder or a wall
    if (is_occupied) {
      // Check if the cell is a cylinder
      bool is_cylinder = /* your cylinder detection code goes here */;

      if (is_cylinder) {
        // Record the location of the cylinder
        std::cout << "Found cylinder at (" << x << ", " << y << ")" << std::endl;
      } else {
        // Record the location of the wall
        std::cout << "Found wall at (" << x << ", " << y << ")" << std::endl;
      }
    }
  }
}

int main(int argc,char **argv) {
  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose

  // first: it is mandatory to initialize the pose of the robot
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);
  // wait for navigation stack to become operationale
  navigator.WaitUntilNav2Active();
  // spin in place of 90 degrees (default parameter)
  navigator.Spin();
  while ( ! navigator.IsTaskComplete() ) {
    // busy waiting for task to be completed
  }
  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  goal_pos->position.x = 2;
  goal_pos->position.y = 1;
  goal_pos->orientation.w = 1;
  // move to new pose
  navigator.GoToPose(goal_pos);
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  // move to new pose
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  // backup of 0.15 m (deafult distance)
  navigator.Backup();
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  goal_pos->position.x = -2;
  goal_pos->position.y = -0.5;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  // move to new pose
  while ( ! navigator.IsTaskComplete() ) {
    
  }

  //  Project code

 // Create a node handle
  ros::NodeHandle nh;

  // Create a transform listener
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Subscribe to the laser scanner data
  ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::PointCloud2>("/scan", 10, [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
      // Get the transform from laser scanner to base_link
      geometry_msgs::TransformStamped tf;
      try {
        tf = tf_buffer.lookupTransform("base_link", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
      }
      catch (tf2::TransformException& ex) {
        ROS_WARN("Failed to get transform from laser scanner to base_link: %s", ex.what());
        return;
      }

      // Convert the laser scanner data to base_link frame
      sensor_msgs::PointCloud2 transformed_scan;
      tf2::doTransform(*msg, transformed_scan, tf);

      // Process the transformed laser scanner data to detect cylinders and walls
      // ...
  });
  
  auto scan_subscriber = navigator.CreateSubscription<sensor_msgs::msg::LaserScan>("/scan", 10, [&](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      // process laser scan data here
  });

  // Subscribe to /scan and /map topics
  auto scan_sub = navigator.CreateSubscription<sensor_msgs::msg::LaserScan>("/scan", 10, scanCallback);
  // auto map_sub = navigator.CreateSubscription<nav_msgs::msg::OccupancyGrid>("/map", 10, mapCallback);

  // Create a node
  auto node = rclcpp::Node::make_shared("map_processor");

  // Subscribe to the map topic
  auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, mapCallback);
  // ...

  rclcpp::spin(navigator.GetNode()); // spin the node
  // End of project code

  // Lab 7 code
  // Get current path and print to screen
  nav_msgs::msg::Path::SharedPtr path = navigator.GetPath(goal_pos);
  for (auto pose : path->poses) {
    tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    std::cout << "x: " << pose.pose.position.x
              << "  y: " << pose.pose.position.y
              << "  yaw: " << pose.pose.orientation.z << "\n";
  }
  // End of Lab 7 code

  // complete here....

  rclcpp::shutdown(); // shutdown ROS
  return 0;
}