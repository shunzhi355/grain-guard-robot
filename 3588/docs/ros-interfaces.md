# ROS Interface Contract — Grain Sampling Robot

> **Version**: 1.0  
> **ROS Distro**: Noetic  
> **Last Updated**: 2026-07-15  
> **Owner**: Grain Sampling Team

---

## 1. Purpose

This document defines the ROS topic, service, and transform interfaces used by the grain sampling robot software stack. It serves as the contract between the robot control system (navigation, SLAM, mechanism) and the HMI/cloud layers.

All topics and services follow ROS Noetic conventions. Custom message types are **not** used — all interfaces use standard ROS message types and `std_srvs/Trigger` to minimise cross-team coupling.

---

## 2. Communication Model

| Layer | Protocol | Scope |
|-------|----------|-------|
| Intra-robot real-time | ROS Topics | Sensor data, state, transforms |
| Intra-robot commands | ROS Services | Navigation, mechanism, SLAM |
| Robot ↔ Cloud | MQTT + JSON | External telemetry & commands |

---

## 3. Subscribed Topics

The robot node subscribes to these topics (data flowing **into** the robot control system).

### 3.1 `/odometry/filtered`

| Field | Value |
|-------|-------|
| **Type** | `nav_msgs/Odometry` |
| **Publisher** | robot_localization EKF node |
| **Rate** | ~50 Hz (nominal) |
| **Frame ID** | `odom` |
| **Child Frame ID** | `base_link` |

**Description**: Fused odometry from the EKF filter, combining wheel odometry, IMU, and optionally visual odometry. Primary pose and twist source for navigation and UI display.

**Key fields used**:
- `pose.pose.position.{x, y, z}` — filtered position in odom frame
- `pose.pose.orientation.{x, y, z, w}` — filtered orientation as quaternion
- `twist.twist.linear.{x, y, z}` — linear velocity in base_link frame
- `twist.twist.angular.{x, y, z}` — angular velocity in base_link frame

### 3.2 `/map`

| Field | Value |
|-------|-------|
| **Type** | `nav_msgs/OccupancyGrid` |
| **Publisher** | SLAM node |
| **Rate** | On update (typically 0.1–1 Hz) |
| **Frame ID** | `map` |

**Description**: Current occupancy grid map of the warehouse environment. Used by the navigation stack and UI minimap.

**Expected parameters**:
- Resolution: 0.05 m/cell
- Dimensions: up to 2000 × 2000 cells (100 m × 100 m warehouse)
- Origin: bottom-left corner of map

### 3.3 `/tf`

| Field | Value |
|-------|-------|
| **Type** | `tf2_msgs/TFMessage` |
| **Publisher** | robot_state_publisher + EKF |
| **Rate** | ~50 Hz (continuous) |

**Description**: Coordinate frame transforms. The following frames MUST be defined:

| Frame | Parent | Description |
|-------|--------|-------------|
| `map` | — | World-fixed map frame (origin at startup) |
| `odom` | `map` | Drift-free continuous reference frame |
| `base_link` | `odom` | Robot chassis centre |
| `livox_frame` | `base_link` | Livox LiDAR optical centre |
| `camera_frame` | `base_link` | USB camera optical centre |
| `mechanism_frame` | `base_link` | Sampling mechanism mounting point |

### 3.4 `/livox/lidar`

| Field | Value |
|-------|-------|
| **Type** | `sensor_msgs/PointCloud2` |
| **Publisher** | Livox SDK driver |
| **Rate** | ~10 Hz |
| **Frame ID** | `livox_frame` |

**Description**: 3D point cloud from the Livox MID-360 LiDAR. Used for SLAM mapping and obstacle detection.

**Key fields**:
- `x, y, z` — point coordinates
- `intensity` — reflectance
- `ring` — scan ring number (non-retro reflective mode)

### 3.5 `/camera/image_raw`

| Field | Value |
|-------|-------|
| **Type** | `sensor_msgs/Image` |
| **Publisher** | USB camera driver (v4l2_camera / usb_cam) |
| **Rate** | ~30 Hz |
| **Frame ID** | `camera_frame` |
| **Encoding** | `bgr8` |

**Description**: Raw video frames from the forward-facing USB camera. Used for visual monitoring and potential AR tag detection.

**Expected resolution**: 640 × 480 (configurable up to 1920 × 1080)

### 3.6 `/mechanism/status`

| Field | Value |
|-------|-------|
| **Type** | `std_msgs/String` |
| **Publisher** | mechanism controller (mock or real) |
| **Rate** | 1 Hz |
| **Frame ID** | — |

**Description**: JSON-encoded status report from the grain sampling mechanism. The payload format:

```json
{
  "state": "idle | sampling | paused | error",
  "depth": 0.0,
  "pressure": 0.0,
  "bin_weights": [0, 0, 0]
}
```

| Field | Type | Description |
|-------|------|-------------|
| `state` | string | Current operation state |
| `depth` | float | Probe penetration depth (m) |
| `pressure` | float | Hydraulic pressure (bar) |
| `bin_weights` | array[3] | Weight readings from 3 sampling bins (g) |

---

## 4. Called Services

The robot node calls these services (commands flowing **into** the robot subsystems).

### 4.1 Navigation Services

#### `/navigate_to_pose`

| Field | Value |
|-------|-------|
| **Type** | `nav2_msgs/NavigateToPose` |
| **Server** | Nav2 behavior server |
| **Description** | Send a navigation goal (pose + orientation in `map` frame) |
| **Response** | Standard Nav2 result (success/failure) |

#### `/slam/start_mapping`

| Field | Value |
|-------|-------|
| **Type** | `std_srvs/Trigger` |
| **Server** | SLAM node |
| **Description** | Begin SLAM mapping session |

#### `/slam/stop_mapping`

| Field | Value |
|-------|-------|
| **Type** | `std_srvs/Trigger` |
| **Server** | SLAM node |
| **Description** | Stop SLAM mapping |

#### `/slam/save_map`

| Field | Value |
|-------|-------|
| **Type** | `std_srvs/Trigger` |
| **Server** | SLAM node |
| **Description** | Persist current map to disk |

### 4.2 Mechanism Services

#### `/mechanism/start_sampling`

| Field | Value |
|-------|-------|
| **Type** | `std_srvs/Trigger` |
| **Server** | Mechanism controller |
| **Description** | Begin grain sampling operation |

#### `/mechanism/stop_sampling`

| Field | Value |
|-------|-------|
| **Type** | `std_srvs/Trigger` |
| **Server** | Mechanism controller |
| **Description** | Abort/stop sampling operation immediately |

#### `/mechanism/pause_sampling`

| Field | Value |
|-------|-------|
| **Type** | `std_srvs/Trigger` |
| **Server** | Mechanism controller |
| **Description** | Pause sampling (maintain position) |

#### `/mechanism/resume_sampling`

| Field | Value |
|-------|-------|
| **Type** | `std_srvs/Trigger` |
| **Server** | Mechanism controller |
| **Description** | Resume paused sampling |

### 4.3 Safety Services

#### `/emergency_stop`

| Field | Value |
|-------|-------|
| **Type** | `std_srvs/Trigger` |
| **Server** | Safety controller |
| **Description** | Emergency stop — halt ALL motion immediately. Must latch until manually reset. |

---

## 5. Coordinate Frames

```
map  ←──  odom  ←──  base_link  ←──  livox_frame
                                  ←──  camera_frame
                                  ←──  mechanism_frame
```

| Frame | Description | Static |
|-------|-------------|--------|
| `map` | World-fixed, origin at robot startup position | No (updated by SLAM) |
| `odom` | Continuous, drift-free reference | No (updated by EKF) |
| `base_link` | Robot chassis centre (rotation centre) | — |
| `livox_frame` | Livox sensor origin | Yes (relative to base_link) |
| `camera_frame` | Camera optical centre | Yes (relative to base_link) |
| `mechanism_frame` | Sampling mechanism mount | Yes (relative to base_link) |

---

## 6. Quality of Service (QoS) Profiles

| Topic | Reliability | Durability | History |
|-------|-------------|------------|---------|
| `/odometry/filtered` | Best Effort | Volatile | Keep Last 1 |
| `/map` | Reliable | Transient Local | Keep Last 1 |
| `/tf` | Best Effort | Volatile | Keep All (100) |
| `/livox/lidar` | Best Effort | Volatile | Keep Last 1 |
| `/camera/image_raw` | Best Effort | Volatile | Keep Last 1 |
| `/mechanism/status` | Reliable | Transient Local | Keep Last 1 |

| Service | QoS |
|---------|-----|
| All services | Default (Reliable + Volatile) |

---

## 7. Versioning & Change Process

Interface changes (new topics, modified message types, removed services) require:

1. Update this document (`docs/ros-interfaces.md`)
2. Update all subscribers/publishers in affected packages
3. Update mock robot node (`src/mock_robot/`)
4. PR approval by at least one other team member

When adding a new topic or service, prefer standard message types over custom ones unless performance or type safety requirements dictate otherwise.

---

## 8. Appendix: Message Type Includes

```python
# Python imports for all interface types used above
from nav_msgs.msg import Odometry, OccupancyGrid
from nav_msgs.srv import ...
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String, Header
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose  # (action, but exposed as service via behavior server)
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Quaternion
from tf2_msgs.msg import TFMessage
```
