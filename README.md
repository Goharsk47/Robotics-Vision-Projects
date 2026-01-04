# Robotics-Vision-Projects
I have started from very basics you will probably get any errors.
A comprehensive collection of **robotics and computer vision projects** featuring autonomous systems, object detection, and navigation using modern robotics frameworks.

## 📋 Table of Contents

- [Projects Overview](#projects-overview)
- [Prerequisites](#prerequisites)
- [Project Structure](#project-structure)
- [Installation & Setup](#installation--setup)
- [Projects](#projects)
  - [1. Object Detection for Robotic Vision](#1-object-detection-for-robotic-vision)
  - [2. Autonomous Patrolling Robot with ROS 2 Navigation](#2-autonomous-patrolling-robot-with-ros-2-navigation)
- [Resources](#resources)
- [License](#license)

---

## Projects Overview

This repository contains multiple robotics projects demonstrating:
- **Computer Vision**: Real-time object detection using TensorFlow and CNNs
- **Autonomous Navigation**: ROS 2 Nav2 stack for path planning and obstacle avoidance
- **Image Processing**: OpenCV-based frame processing and bounding box visualization
- **ROS 2 Integration**: Nav2 navigation stack, waypoint following, and patrol behaviors

---

## Prerequisites

### General Requirements
- **OS**: Ubuntu 20.04 / 22.04 (recommended)
- **Python**: 3.8+
- **Git**: For version control

### For Object Detection Project
- TensorFlow 2.x
- OpenCV (cv2)
- NumPy, Matplotlib

### For Patrolling Robot Project
- ROS 2 (Humble/Foxy)
- Nav2 Navigation Stack
- Gazebo Simulator
- RViz

---

## Project Structure

```
Robotics-Vision-Projects/
├── object-detection/                  # Project 1: TensorFlow Object Detection
│   ├── robot_vision_detection.py      # Main detection script
│   ├── requirements.txt               # Python dependencies
│   ├── outputs/                       # Sample detection results
│   └── README.md                      # Project-specific documentation
│
├── patrolling-robot-ros2/             # Project 2: ROS 2 Patrolling Robot
│   ├── src/
│   │   └── patrol_robot_pkg/
│   │       ├── patrol_node.py         # Main patrol script
│   │       ├── launch/
│   │       └── config/
│   ├── patrol_log.csv                 # Patrol activity logs
│   └── README.md                      # Project-specific documentation
│
├── README.md                          # This file
├── LICENSE                            # MIT License
└── .gitignore                         # Python gitignore
```

---

## Installation & Setup

### Global Setup

```bash
# Clone the repository
git clone https://github.com/Goharsk47/Robotics-Vision-Projects.git
cd Robotics-Vision-Projects

# (Optional) Create a Python virtual environment
python3 -m venv venv  #to create seperate python environment
source venv/bin/activate  # On Windows: venv\\Scripts\\activate
```

---

## Projects

### 1. Object Detection for Robotic Vision

**Overview**: Build a real-time object detection system using TensorFlow's pre-trained models that simulates a robot's perception system.

**Key Features**:
- ✅ Pre-trained CNN model from TensorFlow Hub
- ✅ Real-time object detection from webcam or images
- ✅ Bounding box visualization with class labels and confidence scores
- ✅ Support for COCO dataset classes (person, bottle, chair, etc.)
- ✅ Works on single images or video streams

**Technology Stack**:
- TensorFlow 2.x & TensorFlow Hub
- OpenCV (cv2)
- Python 3.8+

**Quick Start**:

```bash
cd object-detection/

# Install dependencies
pip install -r requirements.txt

# Run on webcam (real-time)
python robot_vision_detection.py --source webcam

# Run on image file
python robot_vision_detection.py --source path/to/image.jpg

# Run on video file
python robot_vision_detection.py --source path/to/video.mp4
```

**Output**: Detection results with drawn bounding boxes saved to `outputs/` folder.

**Resources**:
- [TensorFlow Object Detection API Tutorial](https://tensorflow-object-detection-api-tutorial.readthedocs.io/)
- [TensorFlow Hub Object Detection](https://www.tensorflow.org/hub/tutorials/object_detection)
- [LearnOpenCV: Object Detection with TensorFlow](https://learnopencv.com/object-detection-tensorflow-hub/)

---

### 2. Autonomous Patrolling Robot with ROS 2 Navigation

**Overview**: Develop an autonomous patrolling robot using ROS 2's Nav2 stack with waypoint following, obstacle avoidance, and activity logging.

**Key Features**:
- ✅ Multi-waypoint patrol mission planning
- ✅ Real-time obstacle detection and dynamic path adjustment
- ✅ Return-to-home functionality
- ✅ Patrol activity logging (timestamps, coordinates, waypoints visited)
- ✅ Integration with Gazebo simulator and RViz visualization
- ✅ Support for continuous patrol loops (security/surveillance use-case)

**Technology Stack**:
- ROS 2 (Humble/Foxy)
- Nav2 Navigation Stack
- Gazebo Simulator
- RViz Visualization
- Python 3

**Quick Start**:

```bash
cd patrolling-robot-ros2/

# Build the ROS 2 package (in workspace root)
colcon build
source install/setup.bash

# Launch Gazebo + Nav2 + Patrol Node
ros2 launch patrol_robot_pkg patrol_demo.launch.py

# View patrol logs
cat patrol_log.csv
```

**Expected Output**:
- Gazebo simulation with robot patrolling predefined waypoints
- RViz visualization of costmap, global/local plans
- `patrol_log.csv` with timestamps and coordinates of visited waypoints

**Patrol Logic**:
1. Robot receives list of waypoints (x, y, θ)
2. Navigates to each waypoint sequentially using Nav2
3. If obstacles are detected, Nav2 dynamically replans path
4. Records visit timestamp and coordinates in log file
5. Returns to starting position after patrol cycle
6. (Optional) Repeats patrol in loop for continuous surveillance

**Resources**:
- [AutomaticAddison: ROS 2 Inspection Robot](https://automaticaddison.com/how-to-run-an-inspection-with-a-robot-ros-2-navigation/)
- [Nav2 Documentation](https://docs.nav2.org/)
- [ROS 2 Official Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

---

## Next Steps

### To Get Started:
1. **Clone** this repository
2. **Choose a project** based on your interest
3. **Follow the setup instructions** in each project's README
4. **Customize and extend** the code for your use-case

### Project Enhancement Ideas:
- Multi-robot coordination (multiple patrol robots)
- Machine learning model fine-tuning for custom object classes
- Integration with cloud services (AWS RoboMaker, Google Cloud)
- Real hardware deployment (TurtleBot, robot arms, drones)
- Advanced behaviors (person-following, dynamic obstacle avoidance)

---

## Resources & References

### Computer Vision
- [TensorFlow Object Detection API](https://www.tensorflow.org/hub/tutorials/object_detection)
- [OpenCV Documentation](https://docs.opencv.org/)
- [COCO Dataset](https://cocodataset.org/)

### ROS 2 & Navigation
- [ROS 2 Official Documentation](https://docs.ros.org/)
- [Nav2 Navigation Stack](https://docs.nav2.org/)
- [Gazebo Simulator](https://classic.gazebosim.org/)
- [RViz Visualization](http://wiki.ros.org/rviz)

### Learning Resources
- [Learn OpenCV](https://learnopencv.com/)
- [ROS for Beginners](https://wiki.ros.org/ROS/Tutorials)
- [TensorFlow Tutorials](https://www.tensorflow.org/tutorials)

---

## Contributing

Contributions are welcome! Feel free to:
- Open issues for bugs or feature requests
- Submit pull requests with improvements
- Share your extensions and use-cases

---

## License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

---

## Author

**Goharsk47** - Robotics & Automation Enthusiast

- GitHub: [@Goharsk47](https://github.com/Goharsk47)
- Interests: ROS, Computer Vision, Robotics, Automation, Deep Learning

---

## Support

If you have questions or need help, please:
1. Check the individual project READMEs
2. Review the linked documentation and tutorials
3. Open a GitHub issue with detailed description

Happy Coding! 🤖🚀
