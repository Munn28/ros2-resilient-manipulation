#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>


using std::placeholders::_1;


class VisionMotionExecutor : public rclcpp::Node
{
public:

  explicit VisionMotionExecutor(
    const rclcpp::NodeOptions & options
  )
  : Node(
      "vision_motion_executor",
      options
    )
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

    gripper_publisher_ =
      this->create_publisher<std_msgs::msg::Bool>(
        "/virtual_gripper/command",
        10
      );

    RCLCPP_INFO(
      this->get_logger(),
      "Vision-guided manipulation executor started."
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for stable red-object pose..."
    );
  }


  void initialise_move_group()
  {
    move_group_ =
      std::make_shared<
        moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(),
          "ur_manipulator"
        );

    move_group_->setPoseReferenceFrame(
      "base_link"
    );

    move_group_->setEndEffectorLink(
      "tool0"
    );

    move_group_->setPlanningTime(
      8.0
    );

    move_group_->setGoalPositionTolerance(
      0.01
    );

    move_group_->setGoalOrientationTolerance(
      0.05
    );

    RCLCPP_INFO(
      this->get_logger(),
      "MoveIt interface ready."
    );
  }


private:

  // ==========================================================
  // PLAN + EXECUTE WITH RETRIES
  // ==========================================================

  bool move_to_pose(
    const std::string & name,
    const geometry_msgs::msg::Pose & pose,
    double velocity,
    double acceleration,
    int max_attempts = 3
  )
  {
    move_group_->setMaxVelocityScalingFactor(
      velocity
    );

    move_group_->setMaxAccelerationScalingFactor(
      acceleration
    );

    move_group_->clearPoseTargets();

    move_group_->setPoseTarget(
      pose,
      "tool0"
    );


    for (
      int attempt = 1;
      attempt <= max_attempts;
      ++attempt
    )
    {
      RCLCPP_INFO(
        this->get_logger(),
        "%s: planning attempt %d/%d",
        name.c_str(),
        attempt,
        max_attempts
      );


      moveit::planning_interface::
        MoveGroupInterface::Plan plan;


      const bool planned =
        static_cast<bool>(
          move_group_->plan(
            plan
          )
        );


      if (!planned)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "%s: planning failed on attempt %d.",
          name.c_str(),
          attempt
        );

        std::this_thread::sleep_for(
          std::chrono::milliseconds(300)
        );

        continue;
      }


      RCLCPP_INFO(
        this->get_logger(),
        "%s: trajectory found. Executing...",
        name.c_str()
      );


      const auto result =
        move_group_->execute(
          plan
        );


      if (
        result ==
        moveit::core::MoveItErrorCode::SUCCESS
      )
      {
        RCLCPP_INFO(
          this->get_logger(),
          "%s SUCCESS.",
          name.c_str()
        );

        move_group_->clearPoseTargets();

        return true;
      }


      RCLCPP_WARN(
        this->get_logger(),
        "%s: execution failed.",
        name.c_str()
      );


      std::this_thread::sleep_for(
        std::chrono::milliseconds(300)
      );
    }


    RCLCPP_ERROR(
      this->get_logger(),
      "%s FAILED after %d attempts.",
      name.c_str(),
      max_attempts
    );

    move_group_->clearPoseTargets();

    return false;
  }


  // ==========================================================
  // VIRTUAL GRIPPER COMMAND
  // ==========================================================

  bool set_gripper(
    bool attached
  )
  {
    if (
      gripper_publisher_->get_subscription_count()
      == 0
    )
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Virtual gripper node is not connected."
      );

      return false;
    }


    std_msgs::msg::Bool command;

    command.data = attached;


    // Publish several times for robustness.
    for (
      int i = 0;
      i < 3;
      ++i
    )
    {
      gripper_publisher_->publish(
        command
      );

      std::this_thread::sleep_for(
        std::chrono::milliseconds(100)
      );
    }


    if (attached)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "GRIPPER ON."
      );
    }
    else
    {
      RCLCPP_INFO(
        this->get_logger(),
        "GRIPPER OFF."
      );
    }


    return true;
  }


  // ==========================================================
  // PERCEPTION CALLBACK
  // ==========================================================

  void target_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg
  )
  {
    std::lock_guard<std::mutex> lock(
      mutex_
    );


    if (!move_group_)
    {
      return;
    }


    if (motion_started_)
    {
      return;
    }


    // Average 10 perception measurements.
    sum_x_ += msg->pose.position.x;
    sum_y_ += msg->pose.position.y;
    sum_z_ += msg->pose.position.z;

    sample_count_++;


    if (
      sample_count_ < 10
    )
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
      "Stable target: X=%.3f Y=%.3f Z=%.3f",
      object_x,
      object_y,
      object_z
    );


    // ========================================================
    // DOWNWARD GRASP ORIENTATION
    //
    // 180 degrees about X.
    // tool0 +Z therefore points approximately downward
    // toward the table.
    // ========================================================

    geometry_msgs::msg::Quaternion grasp_orientation;

    grasp_orientation.x = 1.0;
    grasp_orientation.y = 0.0;
    grasp_orientation.z = 0.0;
    grasp_orientation.w = 0.0;


    // ========================================================
    // ORIGINAL SAFE/HOME ORIENTATION
    // ========================================================

    geometry_msgs::msg::Quaternion home_orientation;

    home_orientation.x = -0.7071068;
    home_orientation.y = 0.0;
    home_orientation.z = 0.0;
    home_orientation.w = 0.7071068;

    // ========================================================
    // 1. PRE-GRASP
    // ========================================================

    geometry_msgs::msg::Pose pregrasp;

    pregrasp.position.x = object_x;
    pregrasp.position.y = object_y;
    pregrasp.position.z = object_z + 0.15;

    pregrasp.orientation =
      grasp_orientation;


    if (
      !move_to_pose(
        "PRE-GRASP",
        pregrasp,
        0.15,
        0.15
      )
    )
    {
      fail_before_grasp();

      return;
    }


    // ========================================================
    // 2. CONTROLLED APPROACH
    // ========================================================

    geometry_msgs::msg::Pose approach =
      pregrasp;

    approach.position.z =
      object_z + 0.08;


    if (
      !move_to_pose(
        "APPROACH",
        approach,
        0.08,
        0.08
      )
    )
    {
      fail_before_grasp();

      return;
    }


    // ========================================================
    // 3. VIRTUAL GRASP
    // ========================================================

    if (
      !set_gripper(
        true
      )
    )
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Task aborted: gripper unavailable."
      );

      return;
    }


    std::this_thread::sleep_for(
      std::chrono::milliseconds(700)
    );


    // ========================================================
    // 4. LIFT
    // ========================================================

    geometry_msgs::msg::Pose lift =
      approach;

    lift.position.z =
      object_z + 0.25;


    if (
      !move_to_pose(
        "LIFT",
        lift,
        0.10,
        0.10
      )
    )
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Lift failed. Keeping object attached."
      );

      return;
    }


    // ========================================================
    // 5. TRANSFER ABOVE GREEN TARGET AREA
    //
    // Green target centre from resilient_cell.sdf:
    // X = 0.42
    // Y = +0.17
    // ========================================================

    geometry_msgs::msg::Pose transfer =
      lift;

    transfer.position.x = 0.42;
    transfer.position.y = 0.17;


    if (
      !move_to_pose(
        "TRANSFER",
        transfer,
        0.12,
        0.12
      )
    )
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Transfer failed. Keeping object attached."
      );

      return;
    }


    // ========================================================
    // 6. LOWER OBJECT TO TARGET AREA
    // ========================================================

    geometry_msgs::msg::Pose place =
      transfer;

    place.position.z =
      object_z + 0.08;


    if (
      !move_to_pose(
        "PLACE APPROACH",
        place,
        0.06,
        0.06
      )
    )
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Placement approach failed."
      );

      return;
    }


    // ========================================================
    // 7. RELEASE
    // ========================================================

    if (
      !set_gripper(
        false
      )
    )
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to send release command."
      );

      return;
    }


    std::this_thread::sleep_for(
      std::chrono::milliseconds(700)
    );


    RCLCPP_INFO(
      this->get_logger(),
      "OBJECT RELEASED IN TARGET AREA."
    );


    // ========================================================
    // 8. RETREAT
    // ========================================================

    geometry_msgs::msg::Pose retreat =
      place;

    retreat.position.z =
      object_z + 0.25;


    if (
      !move_to_pose(
        "RETREAT",
        retreat,
        0.10,
        0.10
      )
    )
    {
      return;
    }


    // ========================================================
    // 9. RETURN TO KNOWN SAFE START POSE
    //
    // Measured from the initial UR5e simulation configuration.
    // ========================================================

    geometry_msgs::msg::Pose safe_home;

    safe_home.position.x = 0.001;
    safe_home.position.y = 0.233;
    safe_home.position.z = 1.079;

    safe_home.orientation =
      home_orientation;


    if (
      !move_to_pose(
        "RETURN HOME",
        safe_home,
        0.15,
        0.15
      )
    )
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Pick-and-place succeeded, but return-home failed."
      );

      return;
    }


    // ========================================================
    // DAY 3 SUCCESS
    // ========================================================

    RCLCPP_INFO(
      this->get_logger(),
      "========================================"
    );

    RCLCPP_INFO(
      this->get_logger(),
      "AUTONOMOUS PICK-AND-PLACE COMPLETE"
    );

    RCLCPP_INFO(
      this->get_logger(),
      "========================================"
    );
  }


  void fail_before_grasp()
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Task stopped before grasp."
    );

    motion_started_ = false;

    reset_samples();
  }


  void reset_samples()
  {
    sum_x_ = 0.0;
    sum_y_ = 0.0;
    sum_z_ = 0.0;

    sample_count_ = 0;
  }


  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped
  >::SharedPtr
    target_subscription_;


  rclcpp::Publisher<
    std_msgs::msg::Bool
  >::SharedPtr
    gripper_publisher_;


  std::shared_ptr<
    moveit::planning_interface::MoveGroupInterface
  >
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

  node_options.parameter_overrides(
    {
      rclcpp::Parameter(
        "use_sim_time",
        true
      )
    }
  );


  auto node =
    std::make_shared<
      VisionMotionExecutor>(
        node_options
      );


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
