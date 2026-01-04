# Quick Start Guide - Object Detection

## 5 मिनट में शुरुआत करो 🚀

### Step 1: Install Python & Dependencies

```bash
# Python 3.8+ install करो
python3 --version

# Dependencies install करो
pip install -r requirements.txt
```

### Step 2: Run Real-Time Detection (Webcam)

```bash
# सीधे चलाओ
python robot_vision_detection.py --source webcam

# Results outputs/ folder में save होंगे
```

**Screen पर क्या देखोगे:**
- Green bounding boxes objects के चारों ओर
- Class name और confidence percentage
- Real-time detection
- Press 'q' to quit

### Step 3: Image पर Test करो

```bash
# अपनी image file path दो
python robot_vision_detection.py --source path/to/your_image.jpg

# Result outputs/ में save होगा
```

### Step 4: Video पर Test करो

```bash
# Video file path दो
python robot_vision_detection.py --source path/to/your_video.mp4

# Processed video outputs/ में save होगा
```

### Confidence Threshold Adjust करो

```bash
# Default: 0.5 (50%)
# कम confidence = ज्यादा detections
python robot_vision_detection.py --source webcam --confidence 0.3

# ज्यादा confidence = कम false positives
python robot_vision_detection.py --source webcam --confidence 0.7
```

## Code Structure

```python
# Main class
RobotVisionDetector
  ├── detect_objects()      # TensorFlow से detection
  ├── draw_detections()     # Boxes और labels draw करो
  ├── process_webcam()      # Live webcam से
  ├── process_image()       # Static image से
  └── process_video()       # Video file से
```

## कौन से Objects Detect करेगा?

90+ COCO classes:
- **People & Animals**: person, cat, dog, horse, cow, bird...
- **Vehicles**: car, bike, bus, truck, train, airplane...
- **Objects**: bottle, cup, keyboard, laptop, phone, clock...
- **Furniture**: chair, table, bed, couch, lamp...
- **Food**: apple, banana, pizza, donut, cake...

## Troubleshooting

### Error: "Cannot open webcam"
```bash
# USB camera properly connected है?
# Ubuntu पर: sudo usermod -a -G video $USER
```

### Very Slow Performance
```bash
# Confidence threshold increase करो
python robot_vision_detection.py --source webcam --confidence 0.7

# Model lite version का use हो रहा है (पहले से fast)
```

### Model Download नहीं हो रहा?
```bash
# Internet check करो
# Proxy के पीछे हो तो set करो:
export HTTP_PROXY=your_proxy_url
export HTTPS_PROXY=your_proxy_url
```

## Output Files

```
outputs/
├── detection_YYYYMMDD_HHMMSS.jpg  # Webcam frames (हर 30 frames)
├── detection_result.jpg            # Image processing result
└── detection_video.avi             # Processed video
```

## Next Steps

1. ✅ Webcam पर test कर ले
2. ✅ अपनी images/videos use कर
3. ✅ Confidence threshold experiment कर
4. ✅ Custom objects के लिए model fine-tune कर (advanced)

## Tips 💡

- **Webcam Quality**: Better camera = Better detections
- **Lighting**: Good lighting में ज्यादा accurate
- **Speed**: GPU use करने से 5-10x faster
- **Accuracy**: Pre-trained model पहले से बहुत good है

---

**Happy Detecting!** 🤖👀
