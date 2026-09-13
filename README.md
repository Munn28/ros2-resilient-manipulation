# Resilient Vision-Guided Manipulation Cell

A ROS 2 robotic manipulation system designed to investigate task-level
fault detection and automatic recovery in vision-guided industrial
robotic manipulation.

## Project Goal

Develop a simulated 6-DOF robotic cell capable of:

- vision-guided object localization
- autonomous pick-and-place
- collision-aware motion planning
- task-level fault detection
- fault classification
- automatic recovery from recoverable failures
- quantitative evaluation of recovery performance

## Technology Stack

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic
- MoveIt 2
- ros2_control
- RViz
- Python / C++
- OpenCV
- TF2
- GitHub Actions

## Robot

Initial platform: Universal Robots UR5e simulation.

The resilience architecture is intended to remain robot-independent.

## Project Status

Day 2 / 7 — RGB-D perception and 3D object localization complete.

## Planned Fault Scenarios

- object detection failure
- stale/incorrect object pose
- missed grasp
- object drop
- unreachable target / IK failure
- path obstruction
- trajectory execution timeout
- sensor/communication heartbeat failure

### Day 2 Milestone

The current system can:

- simulate a UR5e manipulation cell in Gazebo
- acquire RGB and depth images from a simulated RGB-D camera
- detect a target object using OpenCV HSV segmentation
- estimate object depth from the RGB-D stream
- reconstruct the object's 3D camera-frame position
- transform the detected position into world coordinates using TF2
- publish the detected object as both PoseStamped and TF frames

## Evaluation

The project will measure:

- task success rate
- fault detection rate
- recovery success rate
- fault detection latency
- recovery time
- retry count
- human intervention rate
- cycle time

## License

MIT
