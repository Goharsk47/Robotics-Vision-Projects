# Autonomous Patrolling Robot with ROS 2 Navigation

## Overview

This project demonstrates an autonomous robot that patrols predefined waypoints using ROS 2's Nav2 (Navigation 2) stack. The robot can dynamically adjust its path to avoid obstacles and maintain a detailed log of all patrol activities.

## Features

- **Autonomous Navigation**: Uses Nav2 for path planning and execution
- **Multi-waypoint Patrol**: Navigate through predefined waypoints in sequence
- **Obstacle Avoidance**: Dynamic path replanning when obstacles are detected
- **Activity Logging**: Timestamps and coordinates of all visited waypoints
- **Return-to-Home**: Robot returns to starting position after patrol cycle
- **Continuous Operation**: Support for infinite patrol loops (surveillance mode)
- **Gazebo Simulation**: Full simulation in Gazebo with RViz visualization

## Requirements

### System Requirements
- **OS**: Ubuntu 20.04 / 22.04
- **ROS**: ROS 2 Humble / Foxy
- **Python**: 3.8+

### Required Packages
```bash
ros2-humble-desktop  # or ros2-foxy-desktop
ros2-humble-gazebo-*
ros2-humble-nav2-*
ros2-humble-rviz2
colcon
```

## Installation

### 1. Install ROS 2

Follow the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html)

### 2. Setup Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Clone or copy this project
cp -r patrolling-robot-ros2 src/

# Build
colcon build

# Source the workspace
source install/setup.bash
```

## Usage

### Launch Patrol Demo

```bash
# Terminal 1: Launch Gazebo + Nav2 + Robot
ros2 launch patrol_robot_pkg patrol_demo.launch.py

# Terminal 2: Monitor patrol activities
ros2 topic echo /patrol/status

# Terminal 3: View patrol log
tail -f patrol_log.csv
```

### Send Waypoints via ROS CLI

```bash
# Publish patrol goal
ros2 service call /patrol_robot/patrol_patrol std_srvs/Empty
```

## Project Structure

```
patrolling-robot-ros2/
├── src/
│   └── patrol_robot_pkg/
│       ├── patrol_node.py         # Main patrol logic
│       ├── launch/
│       │   └── patrol_demo.launch.py
│       ├── config/
│       │   ├── nav2_params.yaml    # Nav2 configuration
│       │   └── gazebo_world.sdf   # Gazebo world definition
│       └── resource/
│           └── robot_model.urdf   # Robot description
├── patrol_log.csv                 # Activity log
├── README.md                       # This file
└── setup.py
```

## Patrol Algorithm Flow

```
1. Initialize Robot
   ├── Load Nav2 parameters
   ├── Spawn robot in Gazebo
   └── Set initial pose

2. Define Patrol Waypoints
   ├── Waypoint 1: (x1, y1, θ1)
   ├── Waypoint 2: (x2, y2, θ2)
   └── ...
   └── Return point: (start_x, start_y, start_θ)

3. Patrol Execution Loop
   For each waypoint:
   ├── Send goal to Nav2
   ├── Monitor navigation status
   ├── If obstacle detected:
   │   └── Nav2 replans path
   ├── Log visit time and coordinates
   └── Proceed to next waypoint

4. Complete Cycle
   ├── Navigate back to start
   ├── Log return event
   └── (Optional) Repeat from step 3
```

## Waypoint Configuration

Edit `patrol_node.py` to define custom waypoints:

```python
WAYPOINTS = [
    (2.0, 2.0, 0.0),      # Waypoint 1
    (5.0, 2.0, 1.57),     # Waypoint 2
    (5.0, 5.0, 3.14),     # Waypoint 3
    (2.0, 5.0, -1.57),    # Waypoint 4
    (2.0, 2.0, 0.0)       # Return to start
]
```

## Log File Format

The `patrol_log.csv` contains:

```csv
timestamp,waypoint_id,x,y,theta,status
2024-01-04 15:30:45,0,2.0,2.0,0.0,visited
2024-01-04 15:31:12,1,5.0,2.0,1.57,visited
2024-01-04 15:32:05,2,5.0,5.0,3.14,visited
```

## Visualization

### RViz Visualization

RViz will show:
- Robot footprint
- Global costmap
- Local costmap
- Global plan (full path)
- Local plan (immediate trajectory)
- Navigation goal marker

### Gazebo Simulation

Gazebo displays:
- Robot model and physics
- Simulated sensors
- Obstacles and environment
- Real-time robot movement

## Troubleshooting

### Nav2 Failed to Plan
- Ensure goal is within navigable space
- Check costmap configuration
- Increase planner timeout

### Robot Not Moving
- Verify Nav2 has received goal
- Check `amcl` localization status
- Confirm costmaps are updating

### Log File Not Updated
- Check write permissions on directory
- Verify patrol_node.py has started
- Check ROS 2 logs: `ros2 topic echo /patrol/log`

## References

- [Nav2 Documentation](https://docs.nav2.org/)
- [ROS 2 Official Docs](https://docs.ros.org/)
- [AutomaticAddison ROS2 Tutorials](https://automaticaddison.com/)
- [Gazebo Classic Documentation](https://classic.gazebosim.org/)

## Future Enhancements

- Multi-robot coordination (multiple patrol robots)
- Custom object detection integration (object-detection project)
- Dynamic waypoint generation from images
- Cloud-based mission planning
- Real hardware deployment (TurtleBot, custom robots)
- Extended patrol with sensor fusion
