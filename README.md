# FuzzyController_ROS2

ROS2-based autonomous navigation system for a differential-drive mobile robot using fuzzy logic controllers optimized with genetic algorithms (GA) for navigation and obstacle avoidance.

## Overview

This project implements a hybrid fuzzy control architecture for autonomous mobile robot navigation in ROS2 and Gazebo. Two fuzzy logic controllers are used:

- **FLCN**: Navigation controller
- **FLCOA**: Obstacle-avoidance controller

Additionally, a Genetic Algorithm (GA) is used to optimize the fuzzy rule base through a binary mask representation, improving the robot’s navigation performance and reducing travel time.

## Features

- Autonomous navigation toward a target position
- Real-time obstacle avoidance using LIDAR
- Differential-drive robot simulation
- ROS2 and Gazebo integration
- Fuzzy Logic Control (FLC)
- Genetic Algorithm optimization (FLCGA)
- RViz visualization

## Technologies Used

- ROS2
- Gazebo
- Python
- Fuzzy Logic
- Genetic Algorithms (DEAP)
- RViz
- LIDAR sensor simulation

## Repository Structure

```bash
src/gpg_urdf/        # Robot description, controllers, and ROS2 nodes
Videos/              # Gazebo simulation videos
