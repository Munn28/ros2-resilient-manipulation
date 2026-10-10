#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "planning_scene_initializer"
  );

  RCLCPP_INFO(
    node->get_logger(),
    "Initialising MoveIt planning scene..."
  );

  moveit::planning_interface::PlanningSceneInterface
    planning_scene_interface;

  // Give MoveIt services time to become available.
  std::this_thread::sleep_for(
    std::chrono::seconds(2)
  );

  moveit_msgs::msg::CollisionObject table;

  table.header.frame_id = "world";
  table.id = "work_table";

  shape_msgs::msg::SolidPrimitive table_shape;

  table_shape.type =
    shape_msgs::msg::SolidPrimitive::BOX;

  // Same dimensions used in Gazebo.
  table_shape.dimensions = {
    0.50,
    0.80,
    0.40
  };

  geometry_msgs::msg::Pose table_pose;

  table_pose.position.x = 0.45;
  table_pose.position.y = 0.0;
  table_pose.position.z = 0.20;

  table_pose.orientation.w = 1.0;

  table.primitives.push_back(
    table_shape
  );

  table.primitive_poses.push_back(
    table_pose
  );

  table.operation =
    moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_interface.applyCollisionObject(
    table
  );

  RCLCPP_INFO(
    node->get_logger(),
    "Work table added to MoveIt planning scene."
  );

  std::this_thread::sleep_for(
    std::chrono::seconds(2)
  );

  rclcpp::shutdown();

  return 0;
}
