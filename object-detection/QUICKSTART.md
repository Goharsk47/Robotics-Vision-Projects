# Quick Start Guide - Object Detection


### Step 1: Install Python & Dependencies

```bash
#Install Python 3.8+ 
python3 --version

#Install  Dependencies
pip install -r requirements.txt
```

### Step 2: Run Real-Time Detection (Webcam)

```bash

python robot_vision_detection.py --source webcam

# Results outputs/ It will save in the folder.
```

**Look at the screen:**
- Green bounding boxes objects(the objects are covered with the green boxes)
- Class name and confidence percentage
- Real-time detection
- Press 'q' to quit

### Step 3: TEST ON THE IMAGE

```bash
# Give your image file path
python robot_vision_detection.py --source path/to/your_image.jpg

# Result must save in the outputs.
```

### Step 4: Test on the video

```bash
# Give video file path.
python robot_vision_detection.py --source path/to/your_video.mp4

# Processed video must be saved in the outputs
```

### Adjust the confidence threshold

```bash
# Default: 0.5 (50%)
# Less confidence = More detections
python robot_vision_detection.py --source webcam --confidence 0.3

# More confidence = Less false positives
python robot_vision_detection.py --source webcam --confidence 0.7
```

## Code Structure

```python
# Main class
RobotVisionDetector
  ├── detect_objects()      # Detection from tensorflow
  ├── draw_detections()     # Draw boxes and labels
  ├── process_webcam()      # From live webcam
  ├── process_image()       # From static image
  └── process_video()       # From video file
```

## Which object will detect?

90+ COCO classes:
- **People & Animals**: person, cat, dog, horse, cow, bird...
- **Vehicles**: car, bike, bus, truck, train, airplane...
- **Objects**: bottle, cup, keyboard, laptop, phone, clock...
- **Furniture**: chair, table, bed, couch, lamp...
- **Food**: apple, banana, pizza, donut, cake...

## Troubleshooting

### Error: "Cannot open webcam"
```bash
# USB camera properly connected or not check it first?
# Ubuntu: sudo usermod -a -G video $USER
```

### Very Slow Performance
```bash
#Increase Confidence threshold
python robot_vision_detection.py --source webcam --confidence 0.7

# The lite version of the model is being used (it is already faster than the standard one).
```

### Is the model not downloading?
```bash
# Check the Internet 
# If behind a proxy, set these:export HTTP_PROXY=your_proxy_url
export HTTPS_PROXY=your_proxy_url
```

## Output Files

```
outputs/
├── detection_YYYYMMDD_HHMMSS.jpg  # Webcam frames 
├── detection_result.jpg            # Image processing result
└── detection_video.avi             # Processed video
```

## Next Steps

1. ✅Use your own images/videos 
2. ✅ Experiment with confidence threshold
3. ✅ Fine-tune model for custom objects (advanced)

## Tips 💡

- **Webcam Quality**: Better camera = Better detections

- **Webcam Quality**: Better camera → Better detections- **Speed**:  Using GPU makes it 5-10x faster

- **Lighting**: Good lighting provides more accurate results- **Accuracy**: Pre-trained model  is already very good

---

**Happy Detecting!** 🤖👀
