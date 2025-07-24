import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # 웹캠 노드에서 publish하는 토픽
            self.image_callback,
            10)
        self.model = YOLO('/home/hs/yolo_ws/src/yolov8_inference/weights/best.pt')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)[0]

        for box in results.boxes:
            b = box.xyxy[0].cpu().numpy().astype(int)
            x1, y1, x2, y2 = b
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # ▶ 박스 안 잘라내기
            cropped = frame[y1:y2, x1:x2]
            hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)


            # ✅ 평균 HSV 출력 (디버깅용)
            h_mean = int(hsv[..., 0].mean())
            s_mean = int(hsv[..., 1].mean())
            v_mean = int(hsv[..., 2].mean())
            print(f"[HSV 평균] H: {h_mean}, S: {s_mean}, V: {v_mean}")

            # ▶ 색상 범위 정의 (HSV)
            color_ranges = {
                "yellow": ([21, 100, 100], [35, 255, 255]),  # 먼저 검사
                "orange": ([0, 30, 30], [20, 255, 255]),     # 그 다음
                "blue":   ([85, 30, 30], [135, 255, 255]),
            }
            color_name = "unknown"
            for name, (lower, upper) in color_ranges.items():
                lower = np.array(lower, dtype=np.uint8)
                upper = np.array(upper, dtype=np.uint8)
                mask = cv2.inRange(hsv, lower, upper)
                ratio = cv2.countNonZero(mask) / (mask.shape[0] * mask.shape[1])
                        # ✅ 디버그 출력
                print(f"[DEBUG] {name} 비율: {ratio:.2f}")

                if ratio > 0.05:
                    color_name = name
                    break

                if ratio > 0.2:  # 20% 이상이면 해당 색상으로 판단
                    color_name = name
                    break

            # ▶ 색상 텍스트 표시
            cv2.putText(frame, color_name, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("YOLOv8 Detection + Color", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
