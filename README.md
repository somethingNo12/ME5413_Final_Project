# ME5413 — Autonomous Mobile Robotics (Final Project)

[![Ubuntu20.04](https://img.shields.io/badge/OS-Ubuntu%2020.04-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros&logoColor=white)](http://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**Clearpath Jackal** in a Gazebo multi-floor warehouse (**`me5413_project_2526`**) — **SLAM Toolbox** for mapping, **AMCL** + **`move_base`** for navigation, **YOLOv5** and **EasyOCR** for vision, and a Python **task executor** for the ME5413 mission (AY25/26).

![Course environment overview](src/me5413_world/media/overview2526.png)

## Demo

Autonomous run (single session) — screen recording:

<video src="media/ME5413_demo.mp4" controls muted playsinline width="100%">
  Video not shown in your viewer — open <a href="media/ME5413_demo.mp4"><code>media/ME5413_demo.mp4</code></a>.
</video>

Direct file: [`media/ME5413_demo.mp4`](media/ME5413_demo.mp4)

---

## How this project is run

### Typical session (simulation + autonomy)

Use **two terminals** after you already have a saved map (see [Mapping](#mapping-one-time-or-when-the-world-changes)).

| Step | Terminal | Command | What it does |
|------|----------|---------|----------------|
| **1** | 1 | `roslaunch me5413_world world.launch` | Gazebo world, Jackal spawn, plugins, sim time |
| **2** | 2 | `roslaunch me5413_world navigation.launch` | Map server, AMCL, `move_base`, RViz, perception nodes, `task_executor.py` (see below) |

That is the intended **operating order**: world first, then navigation.

### What `navigation.launch` starts (order in file)

1. **`teleop_twist_keyboard`** — keyboard teleop (also useful if you need a nudge).
2. **`map_server`** — loads `$(find me5413_world)/maps/my_map.yaml` by default.
3. **`amcl`** — Monte Carlo localization on the loaded map.
4. **`move_base`** (from `jackal_navigation`) — global/local planning and control.
5. **Costmap / planner tweaks** — e.g. `tim551` as laser `sensor_frame`, smaller inflation, velocity limits.
6. **RViz** — `me5413_world/rviz/navigation.rviz`.
7. **`initial_pose_publisher`** — publishes an initial pose to AMCL after a **3 s** delay.
8. **`block_detector_yolov5_node`** — YOLOv5 on `/front/image_raw` → `/blockornot` (used when task phase enables it).
9. **`easyocr_digit_node`** — digit reading for early task phases (`enable_easyocr`, default `true`).
10. **`room_digit_detector_node`** — room digit detection when `/room_digit_detector_enable` is on.
11. **`task_executor.py`** — autonomous mission state machine (`auto_task`, default `true`).

Ground-truth topics such as `/gazebo/ground_truth/state` are **not** used in the autonomous pipeline (course rule).

---

## Mapping (one-time, or when the world changes)

This workspace uses **[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)** for 2D LiDAR mapping — **not GMapping**.

1. Start the world:  
   `roslaunch me5413_world world.launch`
2. In another terminal, run mapping:  
   `roslaunch me5413_world slam_toolbox_mapping.launch`
3. Drive the robot (teleop is included) until the map looks good.
4. Save the map (example — adjust path if needed):

```bash
roscd me5413_world/maps
rosrun map_server map_saver -f my_map map:=/map
```

The default `navigation.launch` map argument points at **`me5413_world/maps/my_map.yaml`**. Change the `map_file` arg if you use another name.

---

## Stack summary

| Piece | Choice in this repo |
|--------|---------------------|
| Simulation | Gazebo + `me5413_project_2526`, Jackal, sensors, random props |
| **Mapping** | **SLAM Toolbox** (`slam_toolbox_mapping.launch`) |
| Localization | AMCL |
| Planning | `move_base` (Jackal navigation stack) |
| Vision | YOLOv5 + EasyOCR (`yolov5_detector`) |
| Mission | `me5413_world/scripts/task_executor.py` |

---

## Repository layout (`src/`)

| Package | Role |
|---------|------|
| `me5413_world` | Worlds, launches (`world`, `slam_toolbox_mapping`, `navigation`, …), maps, RViz, plugins, **`task_executor.py`** |
| `jackal_description` | Jackal URDF / meshes |
| `interactive_tools` | RViz panel (spawn / clear random objects) |
| `slam_toolbox` (+ msgs, rviz, `karto_sdk`) | **2D SLAM** used for mapping |
| `amcl` | AMCL built in-tree |
| `yolov5_detector` | Camera / YOLO / OCR nodes |

Fork / upstream course template: [NUS-Advanced-Robotics-Centre/ME5413_Final_Project](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project).

---

## Prerequisites & build

- **Ubuntu 20.04**, **ROS Noetic**, **catkin**
- **Gazebo** models: [osrf/gazebo_models](https://github.com/osrf/gazebo_models) + copy `src/me5413_world/models/*` → `~/.gazebo/models/`

```bash
cd <path-to-workspace>
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y ros-noetic-sick-tim ros-noetic-lms1xx ros-noetic-velodyne-description \
  ros-noetic-pointgrey-camera-description ros-noetic-jackal-control
catkin_make
source devel/setup.bash
```

**Python (perception):** PyTorch, OpenCV, EasyOCR; YOLOv5 code under `src/yolov5_detector/yolov5/`. GPU optional.

---

## Mission (short)

Lower floor: count numbered boxes, publish `/cmd_unblock` when needed, exit, ramp to upper floor, pass corridors, choose the open gap when a cone blocks one side, avoid the moving cylinder “pedestrian”, stop in the room matching the **least frequent** digit from downstairs — without ground-truth odometry or `/box_odom`.

---

## Configuration

- Goals / waypoints: `me5413_world/config/config.yaml`
- Map files: `me5413_world/maps/` (`my_map.yaml` / `my_map.pgm`)
- Navigation RViz: `me5413_world/rviz/navigation.rviz`  
- Mapping RViz: `me5413_world/rviz/slam_toolbox.rviz`

More detail (topics, state machine): `PROJECT_ARCHITECTURE.md` (if present locally).

---

## Screenshots

| SLAM Toolbox mapping | Navigation |
|----------------------|------------|
| ![mapping](src/me5413_world/media/rviz_mapping.png) | ![navigation](src/me5413_world/media/rviz_navigation.png) |

RViz object panel (random props):

![control panel](src/me5413_world/media/control_panel.png)

---

## Contributing & license

Course policy and deadlines: **ME5413 Canvas**. Code template: **MIT** — see [`LICENSE`](LICENSE).
