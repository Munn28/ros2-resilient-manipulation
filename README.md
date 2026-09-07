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

Day 1 / 7 — Environment and simulation foundation.

## Planned Fault Scenarios

- object detection failure
- stale/incorrect object pose
- missed grasp
- object drop
- unreachable target / IK failure
- path obstruction
- trajectory execution timeout
- sensor/communication heartbeat failure

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
