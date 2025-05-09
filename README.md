# 🖐️ Finger Tracking TurtleBot3 Controller (ROS 2)

A ROS 2-based project that allows controlling a TurtleBot3 robot using hand tracking via a webcam. This system utilizes OpenCV and MediaPipe to detect finger movement, applies a Kalman Filter for smoothing, and publishes velocity commands to the robot in Gazebo or real-world settings.

---

## 📁 Project Structure

```
finger_control/
├── finger_control/
│   ├── __init__.py
│   └── finger_controller.py
└── setup.py
```

---

## 🚀 Features

- Real-time finger tracking using webcam input
- Hand landmark detection with MediaPipe
- Kalman filtering to smooth hand motion
- ROS 2 Twist messages for robot velocity control
- Visualization with OpenCV
- Tested in simulation with TurtleBot3 in Gazebo

---

## 🔧 Requirements

- ROS 2 Foxy or later
- Python 3.8+
- `mediapipe`
- `opencv-python`
- `filterpy`
- TurtleBot3 simulation packages
- A working webcam

Install dependencies:
```bash
pip install mediapipe opencv-python filterpy
```

---

## 🧠 How It Works

- Uses MediaPipe to detect the index fingertip (landmark 8)
- Applies Kalman filter to stabilize the X-coordinate
- Calculates linear and angular velocity based on fingertip position
- Publishes `geometry_msgs/Twist` to `/cmd_vel` topic

---

## ▶️ Run Instructions

1. Launch TurtleBot3 simulation (e.g., in Gazebo):
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. Build and source your workspace:
```bash
colcon build
source install/setup.bash
```

3. Run the finger tracking node:
```bash
ros2 run finger_control finger_controller
```

4. Press `q` in the OpenCV window to stop.

---

## 📊 Experimental Setup

- Tested in simulation with TurtleBot3 in a standard Gazebo world
- Controlled via finger movements tracked through webcam
- The system was evaluated under different gestures and lighting conditions
- Kalman filtering improved responsiveness and accuracy

---

## 📈 Results Summary

| Gesture         | Result                         |
|----------------|--------------------------------|
| Horizontal swipe | Controls forward speed       |
| Vertical shift   | Controls turning (angular z) |
| Stability        | High with Kalman filter      |

---

## 📷 Demo

![Architecture Diagram](./docs/software_architecture.png)

---

## 🧑‍💻 Authors

- [Your Name](https://github.com/yourusername)

---

## 📝 License

This project is licensed under the MIT License. See `LICENSE` file for details.
