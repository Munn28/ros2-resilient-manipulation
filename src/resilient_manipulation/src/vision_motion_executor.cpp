#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>


using std::placeholders::_1;


class VisionMotionExecutor : public rclcpp::Node
{
public:

  explicit VisionMotionExecutor(const rclcpp::NodeOptions & options)
  : Node("vision_motion_executor", options)
  {
    target_subscription_ =
      this->create_subscription<
        geometry_msgs::msg::PoseStamped>(
        "/perception/red_object_pose_world",
        10,
        std::bind(
          &VisionMotionExecutor::target_callback,
          this,
          _1
        )
      );

    RCLCPP_INFO(
      this->get_logger(),
      "Vision motion executor started."
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for red-object pose..."
    );
  }


  void initialise_move_group()
  {
    move_group_ =
      std::make_shared<
        moveit::planning_interface::
          MoveGroupInterface>(
            shared_from_this(),
            "ur_manipulator"
          );

    move_group_->setPoseReferenceFrame(
      "base_link"
    );

    move_group_->setEndEffectorLink(
      "tool0"
    );

    // Keep initial autonomous motion conservative.
    move_group_->setMaxVelocityScalingFactor(
      0.15
    );

    move_group_->setMaxAccelerationScalingFactor(
      0.15
    );

    move_group_->setPlanningTime(
      10.0
    );

    RCLCPP_INFO(
      this->get_logger(),
      "MoveIt interface ready."
    );
  }


private:

  void target_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(
      mutex_
    );

    if (!move_group_)
    {
      return;
    }

    // Only perform this first test once.
    if (motion_started_)
    {
      return;
    }

    // Average several perception samples before moving.
    sum_x_ += msg->pose.position.x;
    sum_y_ += msg->pose.position.y;
    sum_z_ += msg->pose.position.z;

    sample_count_++;

    if (sample_count_ < 10)
    {
      return;
    }

    const double object_x =
      sum_x_ / sample_count_;

    const double object_y =
      sum_y_ / sample_count_;

    const double object_z =
      sum_z_ / sample_count_;

    motion_started_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Stable target detected: X=%.3f Y=%.3f Z=%.3f",
      object_x,
      object_y,
      object_z
    );

    // --------------------------------------------------------
    // PRE-GRASP
    //
    // We intentionally stop 20 cm above the cube.
    // --------------------------------------------------------

    geometry_msgs::msg::Pose target_pose;

    target_pose.position.x =
      object_x;

    target_pose.position.y =
      object_y;

    target_pose.position.z =
      object_z + 0.15;

    // Use the robot's CURRENT tool orientation.
    //
    // This greatly reduces the chance of an unnecessary
    // orientation-related IK failure during our first test.

    target_pose.orientation.x = -0.7071068;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.7071068;

    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.05);
    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(target_pose,"tool0");

    moveit::planning_interface::
      MoveGroupInterface::Plan plan;

    RCLCPP_INFO(
      this->get_logger(),
      "Planning safe pre-grasp motion..."
    );

    const bool planned =
      static_cast<bool>(
        move_group_->plan(plan)
      );

    if (!planned)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "MoveIt could not find a pre-grasp trajectory."
      );

      motion_started_ = false;

      reset_samples();

      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Trajectory found. Executing..."
    );

    const auto result =
      move_group_->execute(
        plan
      );

    if (
  result !=
  moveit::core::MoveItErrorCode::SUCCESS
)
{
  RCLCPP_ERROR(
    this->get_logger(),
    "Pre-grasp trajectory execution failed."
  );

  return;
}


RCLCPP_INFO(
  this->get_logger(),
  "PRE-GRASP SUCCESS."
);


// ========================================================
// CONTROLLED APPROACH
//
// Perception gives us approximately the visible TOP of
// the cube.
//
// We therefore stop 8 cm above the detected surface.
// This keeps tool0 clear of the object while proving
// that approach motion works.
// ========================================================

geometry_msgs::msg::Pose approach_pose =
  target_pose;

approach_pose.position.z =
  object_z + 0.08;


// Slower movement near the object.

move_group_->setMaxVelocityScalingFactor(
  0.08
);

move_group_->setMaxAccelerationScalingFactor(
  0.08
);


// Remove previous pose target.

move_group_->clearPoseTargets();

move_group_->setPoseTarget(
  approach_pose,
  "tool0"
);


moveit::planning_interface::
  MoveGroupInterface::Plan approach_plan;


RCLCPP_INFO(
  this->get_logger(),
  "Planning controlled approach..."
);


const bool approach_planned =
  static_cast<bool>(
    move_group_->plan(
      approach_plan
    )
  );


if (!approach_planned)
{
  RCLCPP_ERROR(
    this->get_logger(),
    "Approach planning failed."
  );

  return;
}


RCLCPP_INFO(
  this->get_logger(),
  "Approach trajectory found. Executing..."
);


const auto approach_result =
  move_group_->execute(
    approach_plan
  );


if (
  approach_result ==
  moveit::core::MoveItErrorCode::SUCCESS
)
{
  RCLCPP_INFO(
    this->get_logger(),
    "APPROACH SUCCESS."
  );
}
else
{
  RCLCPP_ERROR(
    this->get_logger(),
    "Approach execution failed."
  );
}
  }


  void reset_samples()
  {
    sum_x_ = 0.0;
    sum_y_ = 0.0;
    sum_z_ = 0.0;

    sample_count_ = 0;
  }


  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    target_subscription_;

  std::shared_ptr<
    moveit::planning_interface::
      MoveGroupInterface>
    move_group_;

  std::mutex mutex_;

  bool motion_started_{false};

  int sample_count_{0};

  double sum_x_{0.0};
  double sum_y_{0.0};
  double sum_z_{0.0};
};


int main(
  int argc,
  char ** argv
)
{
  rclcpp::init(
    argc,
    argv
  );
rclcpp::NodeOptions node_options;

node_options.parameter_overrides({
rclcpp::Parameter("use_sim_time", true)
});

  auto node =
    std::make_shared<VisionMotionExecutor>(node_options);

  node->initialise_move_group();

  rclcpp::executors::
    MultiThreadedExecutor executor;

  executor.add_node(
    node
  );

  executor.spin();

  rclcpp::shutdown();

  return 0;
}

