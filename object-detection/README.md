# Object Detection for Robotic Vision

## Overview

This project demonstrates real-time object detection using TensorFlow's pre-trained models. It simulates a robot's perception system that can identify objects in its environment.

## Features

- **Real-time Detection**: Process webcam streams or images in real-time
- **Pre-trained Models**: Uses TensorFlow Hub's EfficientDet models
- **COCO Classes**: Detects 90+ object classes from COCO dataset
- **Bounding Boxes**: Visualizes detected objects with confidence scores
- **Flexible Input**: Supports webcam, image files, and video files

## Requirements

```bash
tensorflow>=2.12.0
tensorflow-hub>=0.12.0
opencv-python>=4.8.0
numpy>=1.21.0
matplotlib>=3.5.0
```

## Installation

```bash
# Navigate to project directory
cd object-detection/

# Install dependencies
pip install -r requirements.txt
```

## Usage

### Real-time Webcam Detection

```bash
python robot_vision_detection.py --source webcam
```

### Image File Detection

```bash
python robot_vision_detection.py --source path/to/image.jpg
```

### Video File Detection

```bash
python robot_vision_detection.py --source path/to/video.mp4
```

## Project Structure

```
object-detection/
├── robot_vision_detection.py    # Main detection script
├── requirements.txt              # Project dependencies  
├── README.md                     # This file
└── outputs/                      # Sample detection results
```

## Algorithm Flow

1. Load pre-trained model from TensorFlow Hub
2. Read input from webcam/image/video
3. Preprocess input (resize, normalize)
4. Run inference through neural network
5. Filter detections by confidence threshold (default: 0.5)
6. Draw bounding boxes with class labels
7. Display or save results

## Expected Detectable Classes

Person, bicycle, car, motorcycle, airplane, bus, train, truck, boat, traffic light, fire hydrant, stop sign, parking meter, bench, cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe, backpack, umbrella, handbag, tie, suitcase, frisbee, skis, snowboard, sports ball, kite, baseball bat, baseball glove, skateboard, surfboard, tennis racket, bottle, wine glass, cup, fork, knife, spoon, bowl, banana, apple, sandwich, orange, broccoli, carrot, hot dog, pizza, donut, cake, chair, couch, potted plant, bed, dining table, toilet, tv, laptop, mouse, remote, keyboard, microwave, oven, toaster, sink, refrigerator, book, clock, vase, scissors, teddy bear, hair drier, toothbrush

## Performance Tips

- Use GPU for faster inference (CUDA compatible GPU recommended)
- For real-time performance, consider lower resolution input
- EfficientDet models offer speed-accuracy tradeoff (d0-d7)

## Troubleshooting

- **Model download fails**: Check internet connection, model URL may be temporarily unavailable
- **Low FPS**: Reduce input resolution or use lighter model variant
- **Memory issues**: Reduce batch size or use GPU with more VRAM

## References

- [TensorFlow Object Detection API](https://www.tensorflow.org/hub/tutorials/object_detection)
- [EfficientDet Models](https://tfhub.dev/google/collections/efficientdet/1)
- [COCO Dataset](https://cocodataset.org/)
- [OpenCV Documentation](https://docs.opencv.org/)

## Future Enhancements

- Custom model training for specific object classes
- Multi-object tracking across frames
- Person keypoint detection
- Instance segmentation
- GPU acceleration optimization
- Real robot integration (ROS wrapper)
