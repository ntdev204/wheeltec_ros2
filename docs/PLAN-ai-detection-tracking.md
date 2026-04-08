# AI Detection & Tracking Implementation Plan

## Requirements Summary

### Hardware Setup
- **Camera**: Astra S (USB cam) on Jetson
- **Detection Model**: YOLOv8m (medium variant)
- **Tracking Algorithm**: ByteTrack
- **ROS_DOMAIN_ID**: 0
- **DDS**: CycloneDDS
- **Architecture**: Option B (Modular Pipeline)

### Key Features
1. **Object Detection**: YOLOv8m with dynamic obstacle avoidance
2. **Human Tracking**: ByteTrack for multi-object tracking
3. **Image Storage**: Local storage with 2k image limit (raw + annotated)
4. **Video Streaming**: Real-time (25-45 FPS) to website
5. **Distributed Computing**:
   - Jetson Orin Nano: AI tasks only
   - Raspberry Pi 4: wheeltec_nav2 (existing)

### New Capabilities
- **Dynamic Obstacle Avoidance**: Predict moving object trajectories and replan paths
- **Dual Website Pages**:
  - AI Detection page (all objects)
  - Human Tracking page (tracked humans only)

---

## Architecture Overview

```
┌─── Jetson Orin Nano (AI Layer) ──────────────────────────────────┐
│                                                                    │
│  ┌─────────────┐    ┌──────────────────┐    ┌─────────────────┐  │
│  │ camera_node │───→│ detection_node   │───→│ tracker_node    │  │
│  │ (Astra S)   │img │ (YOLOv8m TRT)   │det │ (ByteTrack)     │  │
│  └─────────────┘    └──────────────────┘    └────────┬────────┘  │
│                              │                        │           │
│                              │                        │           │
│  ┌──────────────────────────┼────────────────────────┼─────────┐ │
│  │ image_storage_node       │                        │         │ │
│  │ (2k limit manager)       │                        │         │ │
│  └──────────────────────────┘                        │         │ │
│                                                       │         │ │
│  ┌──────────────────────────────────────────────────┼─────────┐ │
│  │ video_stream_node (WebRTC/ZMQ)                   │         │ │
│  │ - AI Detection stream (annotated)                │         │ │
│  │ - Human Tracking stream (tracked only)           │         │ │
│  └──────────────────────────────────────────────────┘         │ │
│                                                                 │ │
│  Topics:                                                        │ │
│  - /camera/image_raw                                            │ │
│  - /detections (Detection2DArray)                               │ │
│  - /tracked_objects (TrackedObjectArray)                        │ │
│  - /tracked_humans (TrackedHumanArray)                          │ │
│  - /dynamic_obstacles (DynamicObstacleArray) ← NEW              │ │
└────────────────────────────────┬────────────────────────────────┘
                          DDS    │
┌────────────────────────────────┼────────────────────────────────┐
│  Raspberry Pi 4 (Control Layer)│                                │
│                                ▼                                │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ wheeltec_nav2 (existing)                                │   │
│  │ - Base control + lidar                                  │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ dynamic_obstacle_avoider_node ← NEW                     │   │
│  │ - Subscribe /dynamic_obstacles                          │   │
│  │ - Predict trajectories                                  │   │
│  │ - Publish /dynamic_costmap                              │   │
│  │ - Integrate with Nav2 costmap                           │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

---

## Component Breakdown

### 1. Custom Messages Package

**Package**: `wheeltec_robot_msg` (MODIFY)

New message types:

```
# Detection2D.msg
std_msgs/Header header
string class_name
int32 class_id
float32 confidence
float32 x_center        # normalized [0,1]
float32 y_center
float32 width
float32 height
int32 x_min             # pixel coordinates
int32 y_min
int32 x_max
int32 y_max

# Detection2DArray.msg
std_msgs/Header header
Detection2D[] detections

# TrackedObject.msg
std_msgs/Header header
int32 track_id
string class_name
int32 class_id
float32 confidence
int32 x_min
int32 y_min
int32 x_max
int32 y_max
float32 velocity_x      # pixel/s
float32 velocity_y
int32 age               # frames tracked
bool is_confirmed

# TrackedObjectArray.msg
std_msgs/Header header
TrackedObject[] objects

# TrackedHuman.msg (subset of TrackedObject for humans only)
std_msgs/Header header
int32 track_id
float32 confidence
int32 x_min
int32 y_min
int32 x_max
int32 y_max
float32 velocity_x
float32 velocity_y
int32 age
bool is_confirmed

# TrackedHumanArray.msg
std_msgs/Header header
TrackedHuman[] humans

# DynamicObstacle.msg ← NEW for obstacle avoidance
std_msgs/Header header
int32 track_id
string class_name
geometry_msgs/Point position        # 3D position in map frame
geometry_msgs/Vector3 velocity      # m/s in map frame
geometry_msgs/Vector3 predicted_position  # predicted position in 1s
float32 radius                      # obstacle radius (m)
float32 confidence
```

---

### 2. AI Detection Package (Jetson)

**Package**: `wheeltec_robot_detection` (NEW)

```
wheeltec_robot_detection/
├── package.xml
├── setup.py
├── config/
│   ├── detection_params.yaml
│   ├── tracker_params.yaml
│   └── storage_params.yaml
├── models/
│   ├── export_onnx.py
│   ├── build_tensorrt.py
│   └── yolov8m.pt (download from Ultralytics)
├── launch/
│   ├── detection.launch.py
│   ├── tracking.launch.py
│   ├── full_ai_pipeline.launch.py
│   └── video_streaming.launch.py
├── wheeltec_robot_detection/
│   ├── __init__.py
│   ├── detection_node.py           # YOLOv8m inference
│   ├── tracker_node.py              # ByteTrack
│   ├── image_storage_node.py        # 2k limit manager
│   ├── video_stream_node.py         # Dual stream (detection + tracking)
│   ├── dynamic_obstacle_publisher.py # Convert detections → obstacles
│   ├── inference/
│   │   ├── __init__.py
│   │   ├── tensorrt_detector.py     # TensorRT YOLOv8m
│   │   ├── preprocessor.py
│   │   └── postprocessor.py
│   ├── tracking/
│   │   ├── __init__.py
│   │   ├── byte_tracker.py
│   │   └── kalman_filter.py
│   ├── storage/
│   │   ├── __init__.py
│   │   ├── image_manager.py         # FIFO queue, 2k limit
│   │   └── metadata_db.py           # SQLite for image metadata
│   └── streaming/
│       ├── __init__.py
│       ├── zmq_streamer.py          # ZMQ binary stream
│       └── frame_annotator.py       # Draw bboxes
└── test/
    ├── test_detection.py
    ├── test_tracker.py
    └── test_storage.py
```

**Key Implementation Details**:

#### detection_node.py
- Subscribe: `/camera/image_raw` (sensor_msgs/Image)
- Publish: `/detections` (Detection2DArray)
- Model: YOLOv8m TensorRT INT8 engine
- Target FPS: 30-45
- Classes: COCO 80 classes (person, car, bicycle, etc.)

#### tracker_node.py
- Subscribe: `/detections` (Detection2DArray)
- Publish:
  - `/tracked_objects` (TrackedObjectArray) - all objects
  - `/tracked_humans` (TrackedHumanArray) - humans only
- Algorithm: ByteTrack with Kalman filter
- Track ID persistence: 100+ frames

#### image_storage_node.py
- Subscribe:
  - `/camera/image_raw` (raw images)
  - `/detections` (to filter images with objects)
- Storage:
  - Path: `/home/jetson/wheeltec_data/images/`
  - Limit: 2000 images (1000 raw + 1000 annotated)
  - FIFO: Delete oldest when limit reached
  - Format: `{timestamp}_{frame_id}_raw.jpg` and `{timestamp}_{frame_id}_annotated.jpg`
- Metadata: SQLite DB with timestamp, frame_id, object_count, classes

#### video_stream_node.py
- Subscribe:
  - `/camera/image_raw`
  - `/detections`
  - `/tracked_humans`
- Publish ZMQ streams:
  - Port 5558: AI Detection stream (all objects annotated)
  - Port 5559: Human Tracking stream (humans only annotated)
- Encoding: JPEG (quality 75)
- Target FPS: 25-45

#### dynamic_obstacle_publisher.py
- Subscribe: `/tracked_objects` (TrackedObjectArray)
- Publish: `/dynamic_obstacles` (DynamicObstacleArray)
- Logic:
  - Filter moving objects (velocity > threshold)
  - Convert pixel coordinates → map frame (using camera calibration)
  - Predict position 1 second ahead
  - Publish for Nav2 integration

---

### 3. Dynamic Obstacle Avoidance (Raspberry Pi)

**Package**: `wheeltec_dynamic_avoidance` (NEW)

```
wheeltec_dynamic_avoidance/
├── package.xml
├── setup.py
├── config/
│   └── avoidance_params.yaml
├── launch/
│   └── dynamic_avoidance.launch.py
├── wheeltec_dynamic_avoidance/
│   ├── __init__.py
│   ├── obstacle_avoider_node.py
│   ├── trajectory_predictor.py
│   └── costmap_updater.py
└── test/
    └── test_avoidance.py
```

**Key Implementation**:

#### obstacle_avoider_node.py
- Subscribe: `/dynamic_obstacles` (DynamicObstacleArray)
- Publish: `/dynamic_costmap` (nav_msgs/OccupancyGrid)
- Logic:
  1. Receive dynamic obstacles with predicted positions
  2. Project trajectories (linear + constant velocity model)
  3. Inflate obstacle regions in costmap
  4. Publish updated costmap to Nav2
- Integration: Nav2 obstacle layer plugin

---

### 4. Website Integration

**Backend Changes** (`website/server/`):

```python
# app/config.py (ADD)
zmq_ai_detection_port: int = 5558
zmq_human_tracking_port: int = 5559

# app/ws/handler.py (MODIFY)
# Add two new ZMQ subscribers for AI streams
# Forward binary frames to WebSocket clients

# app/routes/ai.py (NEW)
# REST endpoints:
# - GET /api/ai/detections/stats
# - GET /api/ai/tracking/stats
# - GET /api/ai/images?limit=50
# - DELETE /api/ai/images/clear
```

**Frontend Changes** (`website/client/`):

```typescript
// src/app/(default)/ai-detection/page.tsx (NEW)
// Page for AI Detection stream (all objects)
// - Live video feed from ZMQ port 5558
// - Detection stats (FPS, object counts by class)
// - Toggle classes visibility

// src/app/(default)/human-tracking/page.tsx (NEW)
// Page for Human Tracking stream (humans only)
// - Live video feed from ZMQ port 5559
// - Tracked human list with IDs
// - Track history visualization

// src/components/ai/detection-feed.tsx (NEW)
// Reusable component for AI video streams

// src/components/ai/detection-stats.tsx (NEW)
// Real-time detection statistics

// src/lib/zmq-ai-client.ts (NEW)
// ZMQ client for AI streams (similar to existing ros-client.ts)
```

---

## Model Pipeline: YOLOv8m → TensorRT INT8

### Step 1: Export ONNX

```python
# models/export_onnx.py
from ultralytics import YOLO

model = YOLO("yolov8m.pt")  # Medium variant
model.export(
    format="onnx",
    imgsz=640,
    opset=17,
    simplify=True,
    dynamic=False,
    half=False,
)
# Output: yolov8m.onnx
```

### Step 2: Build TensorRT Engine

```bash
# On Jetson Orin Nano (MUST build on target device)
trtexec \
    --onnx=yolov8m.onnx \
    --saveEngine=yolov8m_int8.engine \
    --int8 \
    --workspace=4096 \
    --fp16 \
    --verbose
```

### Expected Performance (Jetson Orin Nano Super)

| Model   | Precision | FPS (est.) | Latency (est.) |
|---------|-----------|------------|----------------|
| YOLOv8m | FP32      | ~25        | ~40ms          |
| YOLOv8m | FP16      | ~50        | ~20ms          |
| YOLOv8m | INT8      | ~70        | ~14ms          |

**Total Pipeline Latency**: ~30-40ms (camera + preprocess + inference + postprocess + tracking)

---

## ROS2 Topic Architecture

```
Jetson Topics:
- /camera/image_raw (sensor_msgs/Image) - BEST_EFFORT
- /detections (Detection2DArray) - BEST_EFFORT
- /tracked_objects (TrackedObjectArray) - RELIABLE
- /tracked_humans (TrackedHumanArray) - RELIABLE
- /dynamic_obstacles (DynamicObstacleArray) - RELIABLE
- /ai/fps (std_msgs/Float32) - BEST_EFFORT
- /ai/latency (std_msgs/Float32) - BEST_EFFORT

Raspberry Pi Topics:
- /dynamic_costmap (nav_msgs/OccupancyGrid) - RELIABLE
- /cmd_vel (geometry_msgs/Twist) - RELIABLE
```

---

## DDS Configuration

### Both Machines (Jetson + Raspi)

```bash
# .bashrc
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
```

```xml
<!-- /etc/cyclonedds/cyclonedds.xml -->
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
<Domain>
<General>
<Interfaces>
<NetworkInterface name="eth0" priority="default" multicast="true"/>
</Interfaces>
</General>
<Discovery>
<ParticipantIndex>auto</ParticipantIndex>
</Discovery>
</Domain>
</CycloneDDS>
```

---

## Implementation Phases

### Phase 1: Foundation (Jetson Setup)
- [ ] Install JetPack 6.x, CUDA, TensorRT on Jetson
- [ ] Verify Astra S camera driver (`ros2_astra_camera`)
- [ ] Setup CycloneDDS on both machines
- [ ] Test DDS discovery between Jetson ↔ Raspi

### Phase 2: Model Pipeline
- [ ] Download YOLOv8m.pt
- [ ] Export to ONNX
- [ ] Build TensorRT INT8 engine on Jetson
- [ ] Benchmark FPS and latency

### Phase 3: Detection & Tracking
- [ ] Create custom messages package
- [ ] Implement detection_node.py
- [ ] Implement tracker_node.py
- [ ] Test end-to-end: camera → detection → tracking

### Phase 4: Image Storage
- [ ] Implement image_storage_node.py
- [ ] Create SQLite metadata DB
- [ ] Test FIFO queue (2k limit)

### Phase 5: Video Streaming
- [ ] Implement video_stream_node.py
- [ ] Setup dual ZMQ streams (ports 5558, 5559)
- [ ] Test streaming at 25-45 FPS

### Phase 6: Dynamic Obstacle Avoidance
- [ ] Implement dynamic_obstacle_publisher.py (Jetson)
- [ ] Implement obstacle_avoider_node.py (Raspi)
- [ ] Integrate with Nav2 costmap
- [ ] Test trajectory prediction and replanning

### Phase 7: Website Integration
- [ ] Add ZMQ ports to backend config
- [ ] Create AI detection page
- [ ] Create human tracking page
- [ ] Implement REST API for image retrieval
- [ ] Test real-time streaming in browser

### Phase 8: Integration & Testing
- [ ] End-to-end test: camera → AI → website
- [ ] Latency profiling
- [ ] Performance optimization
- [ ] Safety testing (obstacle avoidance)

---

## Verification Checklist

- [ ] Detection FPS ≥ 30 on Jetson
- [ ] E2E latency < 50ms
- [ ] Tracking maintains ID across 100+ frames
- [ ] Image storage respects 2k limit
- [ ] Video streams at 25-45 FPS to website
- [ ] Dynamic obstacles trigger path replanning
- [ ] Both website pages display correctly
- [ ] DDS communication stable between machines
- [ ] No memory leaks after 1 hour runtime

---

## Tech Stack Summary

| Component | Technology |
|-----------|------------|
| Detection | YOLOv8m + TensorRT INT8 |
| Tracking | ByteTrack |
| Camera | Astra S (ros2_astra_camera) |
| DDS | CycloneDDS |
| ROS_DOMAIN_ID | 0 |
| Video Streaming | ZMQ binary (JPEG) |
| Image Storage | Local filesystem + SQLite |
| Backend | FastAPI + ZMQ |
| Frontend | Next.js + WebSocket |
| Obstacle Avoidance | Nav2 costmap integration |

---

## Open Questions

1. **Camera calibration**: Do we have intrinsic/extrinsic parameters for Astra S to convert pixel → map coordinates?
2. **Nav2 integration**: Which costmap layer should we use for dynamic obstacles? (obstacle_layer or custom plugin?)
3. **Object classes**: Should we track all COCO classes or filter specific ones (person, car, bicycle)?
4. **Storage location**: Confirm Jetson storage path (`/home/jetson/wheeltec_data/images/`)?
5. **Network bandwidth**: Ethernet or WiFi between Jetson ↔ Raspi? Bandwidth sufficient for dual video streams?

---

## Next Steps

1. Review and approve this plan
2. Start Phase 1: Jetson setup and DDS configuration
3. Proceed with model pipeline (Phase 2)
4. Implement detection and tracking nodes (Phase 3)
