# Complete Guide: How to Run Both Projects 🚀

## 📌 Project 1: Object Detection for Robotic Vision

###  (Step-by-Step)

#### 1️⃣ Setup (5 minutes)

```bash
# Open Terminal:
cd object-detection/
pip install -r requirements.txt
```

**What will happen?**
- TensorFlow, OpenCV and all dependencies will download.
- First it will take 3-5 mins

#### 2️⃣ Webcam Live Detection

```bash
# Easiest wayy:
python robot_vision_detection.py --source webcam
```

**What will appear?:**
- Real-time camera feed
- Green boxes objects 
- Class name + Confidence percentage
- Press 'q' to quit

**What If camera will not work?:**
```bash
# Ubuntu:
sudo usermod -a -G video $USER
```

#### 3️⃣ Test on the Image file

```bash
# Give path of your Imange file:
python robot_vision_detection.py --source /path/to/image.jpg

python robot_vision_detection.py --source ~/Desktop/photo.png
```

**Result**
- `outputs/detection_result.jpg` 
- Detected objects with boxes 

#### 4️⃣ Test on the Video File

```bash
# Video process:
python robot_vision_detection.py --source /path/to/video.mp4
```

**Output:**
- `outputs/detection_video.avi` 
- All objects will get detected 

#### 5️⃣ Adjust the sensitivity 

```bash
# show less confident detections:
python robot_vision_detection.py --source webcam --confidence 0.3

# Only high confidence detections:
python robot_vision_detection.py --source webcam --confidence 0.8
```

### What will the code do?

```python
RobotVisionDetector class
├── Load pre-trained model from tensorflow
├── Neural network will run on each frame.
├── Objects detect (person, car, dog, etc.)
├── Green boxes and labels are drawn
└── Output frames are saved 
```

### Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| "Cannot open webcam" | USB camera check , Ubuntu: Run usermod command |
| Very slow (FPS Less) | Increase Confidence (0.7 or 0.8) |
| Model is not downloading | Check the internet, Set the proxy|
| Memory error | Decrease the resolution and use the GPU|

---

## 📌 Project 2: Patrolling Robot with ROS 2 Navigation

### (Step-by-Step)

#### 1️⃣ Install ROS 2

```bash
# Ubuntu 22.04 :
wget https://repo.ros2.org/ros.key -O - | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
sudo apt update
sudo apt install ros-humble-desktop
```

#### 2️⃣ Workspace Setup

```bash
# Create new workspace:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Copy the Project files:
cp -r /path/to/patrolling-robot-ros2 src/

# Build:
colcon build

# Source:
source install/setup.bash
```

#### 3️⃣ Gazebo + Nav2 + Robot Launch

```bash
# Terminal 1 :
cd ~/ros2_ws
source install/setup.bash
ros2 launch patrol_robot_pkg patrol_demo.launch.py
```

**What will happen:**
1. It will open Gazebo simulator
2. It will open RViz visualization
3. It will open Robot simulation 

#### 4️⃣ Patrol Start

```bash
# Terminal 2 :
ros2 service call /patrol_robot/start_patrol std_srvs/Empty
```

**What will the Robot do:**
- It will go on Predefined waypoints
- Will avoid the obstacles  (Nav2 automatically)
- It will maintain Patrol log
- Come back at Starting point 

#### 5️⃣ Monitoring

```bash
# Terminal 3 - Status check:
ros2 topic echo /patrol/status

# Terminal 4  - Log :
tail -f patrol_log.csv
```

**Log Format:**
```csv
timestamp,waypoint_id,x,y,theta,status
2024-01-04 20:30:45,0,0.0,0.0,0.0,initialized
2024-01-04 20:31:12,1,2.0,2.0,0.0,visited
2024-01-04 20:31:45,2,5.0,2.0,1.57,visited
```

### Algorithm Flow

```
├── Robot initialized with pose (0,0,0)
├── Load waypoints:
│   ├── (2.0, 2.0, 0.0)
│   ├── (5.0, 2.0, 1.57)
│   ├── (5.0, 5.0, 3.14)
│   └── (0.0, 0.0, 0.0) [Return]
├── For each waypoint:
│   ├── Send goal to Nav2
│   ├── Monitor navigation
│   ├── If obstacle: replan path
│   ├── Log visit (timestamp + coordinates)
│   └── Move to next waypoint
└── Patrol cycle complete
```

### Customization

** waypoints add:**

1. `patrol_node.py`
2. Find: `WAYPOINTS = [...] `
3. Add the coordinates 

```python
WAYPOINTS = [
    (2.0, 2.0, 0.0),      #  waypoint 1
    (5.0, 2.0, 1.57),     # waypoint 2
    (0.0, 0.0, 0.0)       # Return home
]
```

---

## 🎯 If you want to use both the project together

### Scenario: Robot with Vision

```bash
# Terminal 1: Object Detection
cd object-detection/
python robot_vision_detection.py --source webcam

# Terminal 2: Robot Patrol
cd ~/ros2_ws
source install/setup.bash
ros2 launch patrol_robot_pkg patrol_demo.launch.py

# Terminal 3: Start Patrol
ros2 service call /patrol_robot/start_patrol std_srvs/Empty
```

**Integration idea:**
- Robot patrols on predefined path (Project 2)
-Sees The detected objects (Project 1)
- If any object detected:
  - It can give an alert.
  - It can change the path.
  -It can record in the log.


---

## 📚 Key Files

### Object Detection
- `robot_vision_detection.py` - Main detection script
- `requirements.txt` - Dependencies
- `QUICKSTART.md` - Quick reference
- `outputs/` - Result files

### Patrolling Robot
- `src/patrol_robot_pkg/patrol_node.py` - Main navigation
- `src/patrol_robot_pkg/launch/` - Launch files
- `patrol_log.csv` - Activity log
- `src/patrol_robot_pkg/config/` - Nav2 config

---

## ⚡ Performance Tips

### Object Detection
✅ Use the GPU (5-10x faster)
✅Increase the Confidence threshold (faster, less accurate)
✅ Lower resolution 
✅ Work in Good lighting  

### Patrolling Robot
✅ You should keep the waypoint in the navigation map.
✅ Run gazebo on medium details
✅ Match Nav 2 parameter with the environment
✅ Keep costmap resolution appropriate 

---

## 🔗 Useful Links

- [TensorFlow Hub](https://tfhub.dev/)
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [Nav2 Stack](https://docs.nav2.org/)
- [Gazebo Simulator](https://classic.gazebosim.org/)

---

## ✅ Checklist

- [ ] Python 3.8+ installed
- [ ] Dependencies installed (pip install -r requirements.txt)
- [ ] ROS 2 Humble installed (for robot project)
- [ ] Webcam/camera connected (for object detection)
- [ ] Gazebo installed (for robot simulation)
- [ ] First test on webcam done
- [ ] Patrol waypoints customized
- [ ] Both projects working!

---

**Happy Coding & Roboticizing!** 🤖🎓
