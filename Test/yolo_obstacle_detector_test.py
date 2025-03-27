#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import time
import numpy as np

class YoloDetector:
    def __init__(self, model_name='yolov8n', confidence_threshold=0.4):
        """
        Initialize YOLO detector with ultralytics
        
        Args:
            model_name: yolov8n (nano), yolov8s (small), etc.
            confidence_threshold: Detection threshold (0.0-1.0)
        """
        try:
            from ultralytics import YOLO
            print(f"Loading {model_name} model...")
            self.model = YOLO(model_name)
            self.confidence_threshold = confidence_threshold
            self.class_names = self.model.names
            print(f"Model loaded with {len(self.class_names)} classes")
        except ImportError:
            print("Error: Ultralytics not installed. Install with: pip install ultralytics")
            raise

    def detect(self, frame, draw=True):
        """
        Run detection on a frame
        Args:
            frame: Input image/frame
            draw: Whether to draw bounding boxes on the frame
        Returns:
            (annotated_frame, detections) where:
            - annotated_frame is the input frame with bounding boxes
            - detections is a list of dictionaries with detection info
        """
       
        results = self.model(frame, conf=self.confidence_threshold)
        result = results[0] 
        boxes = result.boxes
        
        detections = []
        for box in boxes:
            cls_id = int(box.cls.item())
            cls_name = self.class_names[cls_id]
            conf = box.conf.item()
            xyxy = box.xyxy[0].tolist() 
            
            detection = {
                'class': cls_name,
                'confidence': conf,
                'bbox': [int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])]  
            }
            detections.append(detection)
    
        annotated_frame = frame.copy()
        if draw and detections:
            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                label = f"{det['class']}: {det['confidence']:.2f}"
   
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                cv2.rectangle(annotated_frame, (x1, y1 - text_size[1] - 5), (x1 + text_size[0], y1), (0, 255, 0), -1)
                
                cv2.putText(annotated_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        return annotated_frame, detections

def main():
    """
    demo application using webcam
    """
    detector = YoloDetector(model_name='yolov8n', confidence_threshold=0.4)
    
    cap = cv2.VideoCapture(0) 
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading from camera")
            break
    
        start_time = time.time()

        annotated_frame, detections = detector.detect(frame)
   
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        print(f"Detected {len(detections)} objects:")
        for i, det in enumerate(detections):
            print(f"  {i+1}. {det['class']} ({det['confidence']:.2f})")
        
        cv2.imshow("YOLO Detection", annotated_frame)
       
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
