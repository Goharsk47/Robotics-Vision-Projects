#!/usr/bin/env python3
"""
Robot Vision Object Detection System
Real-time object detection using TensorFlow and OpenCV
"""

import cv2
import tensorflow as tf
import tensorflow_hub as hub
import numpy as np
import argparse
import os
from datetime import datetime

# TensorFlow Hub model URL (EfficientDet-Lite)
MODEL_URL = "https://tfhub.dev/tensorflow/efficientdet/lite0/detection/1"

# COCO class names
COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
    'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
    'parking meter', 'bench', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
    'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
    'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
    'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
    'laptop', 'mouse', 'remote', 'keyboard', 'microwave', 'oven',
    'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
    'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

class RobotVisionDetector:
    """Object Detection using TensorFlow Hub"""
    
    def __init__(self, model_url=MODEL_URL, confidence_threshold=0.5):
        """Initialize the detector with model"""
        print("[INFO] Loading TensorFlow model from Hub...")
        self.detector = hub.load(model_url)
        self.confidence_threshold = confidence_threshold
        print("[INFO] Model loaded successfully!")
    
    def detect_objects(self, image):
        """Run object detection on image"""
        # Convert BGR to RGB
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Convert to tensor
        tensor_image = tf.convert_to_tensor(rgb_image, dtype=tf.uint8)
        tensor_image = tf.expand_dims(tensor_image, axis=0)
        
        # Run detection
        results = self.detector(tensor_image)
        
        return results
    
    def draw_detections(self, image, detections):
        """Draw bounding boxes and labels on image"""
        height, width = image.shape[:2]
        
        boxes = detections['detection_boxes'][0].numpy()
        scores = detections['detection_scores'][0].numpy()
        classes = detections['detection_class_entities'][0].numpy()
        
        detection_count = 0
        
        for i in range(len(scores)):
            if scores[i] < self.confidence_threshold:
                continue
            
            detection_count += 1
            
            # Get box coordinates
            ymin, xmin, ymax, xmax = boxes[i]
            ymin = int(ymin * height)
            xmin = int(xmin * width)
            ymax = int(ymax * height)
            xmax = int(xmax * width)
            
            # Get class name and confidence
            class_name = classes[i].decode('utf-8')
            confidence = float(scores[i]) * 100
            
            # Draw rectangle
            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            
            # Put label
            label = f"{class_name}: {confidence:.1f}%"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            cv2.rectangle(
                image,
                (xmin, ymin - label_size[1] - 10),
                (xmin + label_size[0], ymin),
                (0, 255, 0),
                -1
            )
            cv2.putText(
                image,
                label,
                (xmin, ymin - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 0),
                2
            )
        
        return image, detection_count
    
    def process_webcam(self):
        """Run real-time detection from webcam"""
        print("[INFO] Starting webcam capture...")
        print("[INFO] Press 'q' to quit")
        
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("[ERROR] Cannot open webcam")
            return
        
        # Create outputs directory
        os.makedirs('outputs', exist_ok=True)
        
        frame_count = 0
        
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("[ERROR] Failed to read frame")
                break
            
            frame_count += 1
            
            # Resize for faster processing
            frame = cv2.resize(frame, (640, 480))
            
            # Run detection
            detections = self.detect_objects(frame)
            
            # Draw results
            frame, count = self.draw_detections(frame, detections)
            
            # Add FPS and detection count
            cv2.putText(
                frame,
                f"Detections: {count}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )
            
            # Display
            cv2.imshow("Robot Vision - Object Detection", frame)
            
            # Save frame every 30 frames
            if frame_count % 30 == 0:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"outputs/detection_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                print(f"[SAVED] {filename}")
            
            # Press q to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
        print("[INFO] Webcam capture ended")
    
    def process_image(self, image_path):
        """Process single image"""
        print(f"[INFO] Processing image: {image_path}")
        
        image = cv2.imread(image_path)
        
        if image is None:
            print(f"[ERROR] Cannot read image: {image_path}")
            return
        
        # Run detection
        detections = self.detect_objects(image)
        
        # Draw results
        image, count = self.draw_detections(image, detections)
        
        # Save output
        os.makedirs('outputs', exist_ok=True)
        output_path = 'outputs/detection_result.jpg'
        cv2.imwrite(output_path, image)
        print(f"[SAVED] Output: {output_path}")
        print(f"[INFO] Objects detected: {count}")
        
        # Display
        cv2.imshow("Detection Result", image)
        print("[INFO] Press any key to close")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def process_video(self, video_path):
        """Process video file"""
        print(f"[INFO] Processing video: {video_path}")
        
        cap = cv2.VideoCapture(video_path)
        
        if not cap.isOpened():
            print(f"[ERROR] Cannot open video: {video_path}")
            return
        
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Create output video
        os.makedirs('outputs', exist_ok=True)
        output_path = 'outputs/detection_video.avi'
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        
        frame_count = 0
        
        while True:
            ret, frame = cap.read()
            
            if not ret:
                break
            
            frame_count += 1
            
            # Run detection
            detections = self.detect_objects(frame)
            
            # Draw results
            frame, count = self.draw_detections(frame, detections)
            
            # Write frame
            out.write(frame)
            
            if frame_count % 30 == 0:
                print(f"[PROGRESS] Processed {frame_count} frames")
        
        cap.release()
        out.release()
        print(f"[SAVED] Output video: {output_path}")

def main():
    parser = argparse.ArgumentParser(description='Robot Vision Object Detection')
    parser.add_argument(
        '--source',
        type=str,
        default='webcam',
        help='Input source: webcam, image_path, or video_path'
    )
    parser.add_argument(
        '--confidence',
        type=float,
        default=0.5,
        help='Confidence threshold (0-1)'
    )
    
    args = parser.parse_args()
    
    # Initialize detector
    detector = RobotVisionDetector(confidence_threshold=args.confidence)
    
    # Process based on source
    if args.source.lower() == 'webcam':
        detector.process_webcam()
    elif args.source.lower().endswith(('.jpg', '.jpeg', '.png')):
        detector.process_image(args.source)
    elif args.source.lower().endswith(('.mp4', '.avi', '.mov')):
        detector.process_video(args.source)
    else:
        print("[ERROR] Invalid source. Use 'webcam', image path, or video path")

if __name__ == "__main__":
    main()
