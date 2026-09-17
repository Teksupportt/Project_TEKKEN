# Project Tekken

Lightweight, modular auto-face feature for robotics — functions similarly to movement in the *TEKKEN* franchise and other 3D fighting games, where you are always facing your opponent.

Project Tekken is a ROS2 (C++) aim-assist framework built for teleoperated humanoid robot combat. It continuously tracks a detected opponent and applies a corrective yaw so the robot stays oriented toward its target, while teleoperator input always takes priority.

## Core behavior

- **Always-facing tracking**: fuses sensor input to estimate the opponent's position and computes a corrective yaw to keep the robot facing them, mirroring the auto-face movement in 3D fighting games.
- **Teleop priority**: manual input above a deadzone threshold immediately overrides the assist — the operator is always in control.
- **Full passthrough on loss-of-target**: if the opponent is lost, no correction is applied; the robot fully returns to manual control rather than guessing.
- **Rate-limited output**: yaw rate and acceleration are capped to protect bipedal stability.

## Architecture

Built as a set of modular ROS2 packages:

| Package | Purpose |
|---|---|
| `common_msgs` | Shared message definitions used across the framework |
| `perception_msgs` | Message types specific to detection/perception output |
| `perception` | Opponent detection — four interchangeable variants: 2D lidar, 3D lidar, camera (classical CV), camera (ML/ONNX) |
| `fusion` | Sensor fusion via a linear Kalman Filter, combining detections into a single opponent state estimate |
| `heading_controller` | PD controller (no integral term, to avoid windup on loss-of-target) with a rate limiter enforcing yaw rate/acceleration limits |
| `bringup` | Launch configuration tying the framework together |

### Design notes

- A standard (linear) Kalman Filter is used rather than an EKF, since the state and measurement models are linear — an EKF would add complexity without benefit.
- No integral term in the controller: integral windup during loss-of-target could cause unpredictable behavior on re-acquisition.
- Sensor topics are defined once in a shared YAML config and consumed by both `perception` and `fusion` launch files, eliminating config drift between them.
- `fusion` supports an arbitrary list of sensors from config, including multiple simultaneous instances of the same sensor type (e.g. three 2D lidars).

## Environment

- ROS2, C++
- Docker / WSL

## File architecture

```
combat_ws/
├── Dockerfile
├── docker-compose.yml
├── .dockerignore
└── src/
    ├── common_msgs/
    │   └── msg/
    │       ├── Matrix2.msg
    │       └── Matrix4.msg
    │
    ├── perception_msgs/
    │   └── msg/
    │       ├── TargetDetection.msg
    │       └── TargetEstimate.msg
    │
    ├── perception/
    │   ├── include/perception/
    │   │   ├── lidar_2d_tracker.hpp
    │   │   ├── lidar_3d_tracker.hpp
    │   │   ├── camera_cv_tracker.hpp
    │   │   └── camera_ml_tracker.hpp
    │   ├── src/
    │   │   ├── lidar_2d_tracker_node.cpp / lidar_2d_tracker_main.cpp
    │   │   ├── lidar_3d_tracker_node.cpp / lidar_3d_tracker_main.cpp
    │   │   ├── camera_cv_tracker_node.cpp / camera_cv_tracker_main.cpp
    │   │   └── camera_ml_tracker_node.cpp / camera_ml_tracker_main.cpp   # optional, requires ONNX Runtime
    │   ├── config/
    │   │   ├── sensors_example.yaml
    │   │   ├── lidar_2d_tracker.yaml
    │   │   ├── lidar_3d_tracker.yaml
    │   │   ├── camera_cv_tracker.yaml
    │   │   └── camera_ml_tracker.yaml
    │   └── launch/
    │       └── perception.launch.py
    │
    ├── fusion/
    │   ├── include/fusion/
    │   │   ├── fusion_node.hpp
    │   │   └── ekf.hpp
    │   ├── src/
    │   │   ├── fusion_node.cpp / fusion_main.cpp
    │   │   └── ekf.cpp
    │   ├── config/
    │   │   └── fusion_params.yaml
    │   └── launch/
    │       └── fusion.launch.py
    │
    ├── heading_controller/
    │   ├── include/heading_controller/
    │   │   ├── heading_controller_node.hpp
    │   │   └── rate_limiter.hpp
    │   ├── src/
    │   │   ├── heading_controller_node.cpp / heading_controller_main.cpp
    │   │   └── rate_limiter.cpp
    │   ├── config/
    │   │   └── controller_params.yaml
    │   └── launch/
    │       └── heading_controller.launch.py
    │
    └── bringup/
        └── launch/
            └── combat_framework.launch.py
```
