# Complete Guide: How to Run Both Projects 🚀

## 📌 Project 1: Object Detection for Robotic Vision

### कैसे करें? (Step-by-Step)

#### 1️⃣ Setup करो (5 minutes)

```bash
# Terminal खोलो और चलाओ:
cd object-detection/
pip install -r requirements.txt
```

**क्या होगा?**
- TensorFlow, OpenCV और सभी dependencies download होंगे
- पहली बार ~3-5 minute लगेगा

#### 2️⃣ Webcam से Live Detection चलाओ

```bash
# सबसे सीधा तरीका:
python robot_vision_detection.py --source webcam
```

**क्या दिखेगा:**
- Real-time camera feed
- Green boxes objects के चारों ओर
- Class name + Confidence percentage
- Press 'q' to quit

**अगर कैमरा नहीं खुला:**
```bash
# Ubuntu पर:
sudo usermod -a -G video $USER
```

#### 3️⃣ Image File पर Test करो

```bash
# अपनी image file का path दो:
python robot_vision_detection.py --source /path/to/image.jpg
# या
python robot_vision_detection.py --source ~/Desktop/photo.png
```

**Result कहाँ मिलेगा?**
- `outputs/detection_result.jpg` में
- Detected objects के साथ boxes होंगे

#### 4️⃣ Video File पर Test करो

```bash
# Video process करो:
python robot_vision_detection.py --source /path/to/video.mp4
```

**Output मिलेगा:**
- `outputs/detection_video.avi` में
- सभी objects detected होंगे

#### 5️⃣ Sensitivity Adjust करो

```bash
# कम confident detections दिखाओ:
python robot_vision_detection.py --source webcam --confidence 0.3

# सिर्फ high confidence detections:
python robot_vision_detection.py --source webcam --confidence 0.8
```

### Code क्या करती है?

```python
RobotVisionDetector class
├── TensorFlow Hub से pre-trained model लोड करता है
├── हर frame पर neural network चलाता है
├── Objects detect करता है (person, car, dog, etc.)
├── Green boxes और labels draw करता है
└── Output frames save करता है
```

### Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| "Cannot open webcam" | USB camera check करो, Ubuntu पर usermod command चलाओ |
| Very slow (FPS कम) | Confidence बढ़ाओ (0.7 या 0.8) |
| Model download नहीं हो रहा | Internet check करो, proxy set करो |
| Memory error | Resolution कम करो या GPU use करो |

---

## 📌 Project 2: Patrolling Robot with ROS 2 Navigation

### कैसे करें? (Step-by-Step)

#### 1️⃣ ROS 2 Install करो

```bash
# Ubuntu 22.04 पर:
wget https://repo.ros2.org/ros.key -O - | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
sudo apt update
sudo apt install ros-humble-desktop
```

#### 2️⃣ Workspace Setup करो

```bash
# नया workspace बनाओ:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Project files copy करो:
cp -r /path/to/patrolling-robot-ros2 src/

# Build करो:
colcon build

# Source करो:
source install/setup.bash
```

#### 3️⃣ Gazebo + Nav2 + Robot Launch करो

```bash
# Terminal 1 में:
cd ~/ros2_ws
source install/setup.bash
ros2 launch patrol_robot_pkg patrol_demo.launch.py
```

**क्या होगा:**
1. Gazebo simulator खुल जाएगा
2. RViz visualization खुल जाएगा
3. Robot simulation शुरू हो जाएगी

#### 4️⃣ Patrol Start करो

```bash
# Terminal 2 में:
ros2 service call /patrol_robot/start_patrol std_srvs/Empty
```

**Robot क्या करेगा:**
- Predefined waypoints पर जाएगा
- Obstacles से बचेगा (Nav2 automatically)
- Patrol log maintain करेगा
- Starting point पर लौटेगा

#### 5️⃣ Monitoring करो

```bash
# Terminal 3 में - Status check करो:
ros2 topic echo /patrol/status

# Terminal 4 में - Log देखो:
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

**अपने waypoints add करने के लिए:**

1. `patrol_node.py` खोलो
2. ढूंढो: `WAYPOINTS = [...] `
3. अपने coordinates add करो:

```python
WAYPOINTS = [
    (2.0, 2.0, 0.0),      # तुम्हारा waypoint 1
    (5.0, 2.0, 1.57),     # तुम्हारा waypoint 2
    (0.0, 0.0, 0.0)       # Return home
]
```

---

## 🎯 दोनों Projects को एक साथ Use करना

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
- नताऱा detected objects को see करता है (Project 1)
- अगर कोई object detect हो तो:
  - Alert दे सकता है
  - Path change कर सकता है
  - Log में record कर सकता है

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
✅ GPU use करो (5-10x faster)
✅ Confidence threshold बढ़ाओ (faster, less accurate)
✅ Lower resolution use करो
✅ Good lighting में work करो

### Patrolling Robot
✅ Waypoints को navigation map के अंदर रखो
✅ Gazebo को medium detail पर चलाओ
✅ Nav2 parameters को environment से match करो
✅ Costmap resolution appropriate रखो

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
