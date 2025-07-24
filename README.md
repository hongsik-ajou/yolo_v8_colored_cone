# YOLOv8 + HSV 색상 인식 ROS2 노드

이 프로젝트는 ROS 2 (Foxy) 환경에서 YOLOv8을 이용해 객체를 탐지하고, 해당 객체의 색상을 HSV 색공간에서 분류합니다.


## 📦 구성
- `/camera/image_raw` 토픽에서 실시간 이미지 구독
- YOLOv8을 통해 bounding box 탐지
- bounding box 내 HSV 색상 분석 (orange, yellow, blue, unknown)

## ✅ 실행 방법

```bash
cd yolo_ws
colcon build
source install/setup.bash
ros2 launch yolov8_launch yolo_system_launch.py
```

## 🎯 주요 파일

- `yolov8_inference/yolo_node.py` : 메인 ROS2 노드
- `scripts/show_hsv.py` : HSV 색상 시각화 유틸리티


