# Directory structure:
# finger_control/
# ├── finger_control/
# │   ├── __init__.py
# │   └── finger_controller.py
# └── setup.py

# finger_control/finger_control/finger_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import numpy as np
from filterpy.kalman import KalmanFilter

class FingerController(Node):
    def __init__(self):
        super().__init__('finger_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils

        self.kf = KalmanFilter(dim_x=2, dim_z=1)
        self.kf.x = np.array([[0.], [0.]])
        self.kf.F = np.array([[1., 1.], [0., 1.]])
        self.kf.H = np.array([[1., 0.]])
        self.kf.P *= 1000.
        self.kf.R = 5
        self.kf.Q = np.array([[0.1, 0.], [0., 0.1]])

        self.last_cx = None

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        h, w, _ = image.shape

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            fingertip = hand_landmarks.landmark[8]
            cx = int(fingertip.x * w)
            cy = int(fingertip.y * h)

            self.kf.predict()
            self.kf.update(np.array([[cx]]))
            smoothed_cx = int(self.kf.x[0][0])

            twist = Twist()
            if self.last_cx is not None:
                dx = smoothed_cx - self.last_cx
                twist.linear.x = float(dx) / 3.0
                twist.angular.z = float(-(cy - h // 2)) / 100.0

            self.publisher_.publish(twist)
            self.last_cx = smoothed_cx

            cv2.circle(image, (smoothed_cx, cy), 10, (0, 255, 0), cv2.FILLED)
            self.mp_draw.draw_landmarks(image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        cv2.imshow("Finger Control", image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = FingerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
