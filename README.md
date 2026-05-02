# ROS 2 Navigation Package — Actions, TF2, and Components

This workspace is the Research Track II (RT2) course assignment at the University of Genova. It implements an action-based robot navigation system inside the `bme_gazebo_sensors` Gazebo simulation. The robot moves to user-defined target poses `(x, y, theta)` using a proportional controller, ROS 2 Actions, TF2, and composable nodes (Components).

The implementation is designed for open, obstacle-free environments — it does not include path planning or obstacle avoidance. Goals should be chosen in clear areas of the simulated world.

---

## Repository layout

```
rt2_navigation_package_course_level/
├── RT2_Simulation_demo.gif
└── src/
    ├── rt2_navigation_interfaces/      # Custom action definition
    │   ├── action/NavigateToPose.action
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── rt2_navigation_assignment/      # Action server + client components
    │   ├── src/
    │   │   ├── navigation_action_server.cpp
    │   │   └── navigation_client_component.cpp
    │   ├── launch/
    │   │   ├── start_navigation_container.launch.py
    │   │   └── load_navigation_components.launch.py
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── bme_gazebo_sensors/             # Simulated robot and world
        ├── urdf/
        ├── meshes/
        ├── worlds/
        ├── config/ekf.yaml
        ├── rviz/
        └── launch/
```

---

## System overview

```
┌──────────────────────────────────────────────────┐
│              ComponentContainer (MT)              │
│                                                   │
│  ┌───────────────────────────┐                   │
│  │  NavigationClientComponent│  ← keyboard UI     │
│  │  (navigation_user_interface)                   │
│  │    sends goals via Action  │                   │
│  └────────────┬──────────────┘                   │
│               │  /navigate_to_pose (Action)        │
│  ┌────────────▼──────────────┐                   │
│  │  NavigationActionServer   │                   │
│  │  (navigation_action_server)                    │
│  │   reads /odom             │                   │
│  │   publishes /cmd_vel      │                   │
│  │   manages TF2 frames      │                   │
│  └───────────────────────────┘                   │
└──────────────────────────────────────────────────┘

Gazebo simulation (bme_gazebo_sensors)
   publishes: /odom, /imu, sensor topics
   subscribes: /cmd_vel
```

Both components run inside the **same** multithreaded container (`component_container_mt`), so they share the same process and avoid inter-process communication overhead.

---

## Package descriptions

### `rt2_navigation_interfaces`

Defines the single custom ROS 2 action used by both components.

**`action/NavigateToPose.action`**

```
# Goal — desired pose in the odom frame (planar, theta = yaw)
float64 x
float64 y
float64 theta
---
# Result
bool   success          # true when target was reached, false on cancel/abort
string message          # human-readable status string
float64 final_x         # last measured x from /odom at end of action
float64 final_y
float64 final_theta
---
# Feedback — published at control frequency while the robot moves
float64 current_x       # current odometry pose
float64 current_y
float64 current_theta
float64 distance_to_goal   # remaining planar distance to target (x,y)
float64 heading_error      # angular error toward target point
float64 yaw_error          # angular error for final theta alignment
```

---

### `rt2_navigation_assignment`

Contains the two composable node implementations and the launch files.

#### `NavigationActionServer`

The controller node. It:

- accepts goals via the `navigate_to_pose` action;
- waits up to 8 seconds for the first `/odom` message before aborting;
- publishes the goal as the `navigation_goal` TF frame (in the `odom` frame) every control cycle;
- reads the transform `base_footprint → navigation_goal` using TF2 to get the error in the robot frame;
- runs a three-phase proportional controller (see below);
- publishes feedback on every control cycle;
- supports goal cancellation at any time;
- publishes the final odometry pose as the action result.

The action server uses a **Reentrant callback group** so the odometry subscription, TF broadcast, and action callbacks do not block each other inside the multithreaded container. Goal execution runs in a detached thread so it does not block the container's executor.

#### `NavigationClientComponent`

The user interface node. It:

- creates a `rclcpp_action::Client` for `navigate_to_pose`;
- detects whether stdin is an interactive terminal — if not, it warns and skips the UI thread;
- runs a keyboard input loop in a dedicated thread so ROS callbacks remain responsive;
- parses user commands and dispatches them as asynchronous action calls;
- prints throttled feedback (1 Hz to avoid flooding the terminal) and final results.

---

## Motion control — three-phase state machine

The controller cycles through three phases in order:

```
FACE_TARGET ──► GO_TO_TARGET ──► MATCH_FINAL_YAW ──► succeed
     ▲               │
     └───────────────┘  (if heading error exceeds rotate_in_place_threshold)
```

| Phase | What the robot does | Transition condition |
|---|---|---|
| `FACE_TARGET` | Rotates in place toward `(x, y)` | `heading_error < heading_tolerance` |
| `GO_TO_TARGET` | Drives forward while correcting heading | `distance_to_goal < xy_tolerance` |
| `MATCH_FINAL_YAW` | Rotates in place to reach final `theta` | `yaw_error < yaw_tolerance` |

If the heading error grows beyond `rotate_in_place_threshold` during `GO_TO_TARGET`, the robot transitions back to `FACE_TARGET` and re-aligns before continuing.

Velocity commands are proportional:

```
v  = clamp(k_linear  × distance,  0,        max_linear_speed)
ω  = clamp(k_angular × heading,  −max_angular_speed, max_angular_speed)
ωf = clamp(k_yaw     × yaw_error,−max_angular_speed, max_angular_speed)
```

---

## Tunable ROS 2 parameters

All parameters can be overridden at launch time with `-p name:=value`.

### NavigationActionServer parameters

| Parameter | Default | Description |
|---|---|---|
| `action_name` | `navigate_to_pose` | Action server name |
| `odom_topic` | `/odom` | Odometry input topic |
| `cmd_vel_topic` | `/cmd_vel` | Velocity command output topic |
| `odom_frame` | `odom` | Odometry TF frame (auto-updated from message) |
| `base_frame` | `base_footprint` | Robot base TF frame (auto-updated from message) |
| `target_frame` | `navigation_goal` | TF frame name for the active goal |
| `publish_odom_tf` | `true` | Broadcast `odom → base_footprint` from odometry |
| `control_frequency` | `20.0` | Controller loop rate in Hz |
| `linear_gain` | `0.26` | Proportional gain for linear velocity |
| `angular_gain` | `0.32` | Proportional gain for heading correction |
| `yaw_gain` | `0.22` | Proportional gain for final yaw alignment |
| `max_linear_speed` | `0.16` | Maximum forward velocity (m/s) |
| `max_angular_speed` | `0.14` | Maximum angular velocity (rad/s) |
| `xy_tolerance` | `0.20` | Position tolerance to declare x,y reached (m) |
| `yaw_tolerance` | `0.30` | Yaw tolerance to declare final orientation reached (rad) |
| `heading_tolerance` | `0.12` | Heading tolerance before driving forward (rad) |
| `rotate_in_place_threshold` | `0.50` | Heading error that forces a re-align mid-drive (rad) |

### NavigationClientComponent parameters

| Parameter | Default | Description |
|---|---|---|
| `action_name` | `navigate_to_pose` | Must match the server's `action_name` |

---

## TF2 frame tree

```
odom
 └── base_footprint      (published by NavigationActionServer from /odom)
 └── navigation_goal     (published by NavigationActionServer while goal is active)
```

The server looks up `base_footprint → navigation_goal` to compute the error in the robot frame. This makes the heading and distance errors automatically correct for the robot's current orientation without any manual trigonometry.

---

## Dependencies

| Dependency | Version |
|---|---|
| ROS 2 | Jazzy |
| Ubuntu | 24.04 |
| Gazebo | via `ros_gz` bridge |
| `robot_localization` | for EKF (included in `bme_gazebo_sensors`) |

For a fresh workspace without the included simulation package, clone the original:

```bash
cd <your_ros2_workspace>/src
git clone -b rt2 https://github.com/CarmineD8/bme_gazebo_sensors.git
```

---

## Build

```bash
cd <your_ros2_workspace>
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Verify that the components are registered:

```bash
ros2 component types | grep rt2_navigation_assignment
```

Expected output:

```
rt2_navigation_assignment
  rt2_navigation_assignment::NavigationActionServer
  rt2_navigation_assignment::NavigationClientComponent
```

---

## Run

### Terminal 1 — start Gazebo

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch bme_gazebo_sensors spawn_robot_ex.launch.py
```

Wait until Gazebo is fully loaded and the robot is spawned before proceeding.

---

### Option A — single launch (container + components together)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rt2_navigation_assignment start_navigation_container.launch.py
```

This starts a `component_container_mt` container named `rt2_navigation_container` and loads both components into it. `use_sim_time` defaults to `true`.

> **Note:** Some terminal environments do not forward stdin to composable nodes launched this way. If the `rt2-ui>` prompt does not respond to keyboard input, use Option B.

---

### Option B — manual container with separate component loading

**Terminal 2 — start the container (keep this terminal open for the UI):**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rclcpp_components component_container_mt
```

**Terminal 3 — load the components into the running container:**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rt2_navigation_assignment load_navigation_components.launch.py
```

This loads both `NavigationActionServer` and `NavigationClientComponent` into the `/ComponentManager` container. The keyboard prompt appears in Terminal 2 because that terminal's stdin is connected to the container process.

**Manual alternative** (equivalent to the launch file above):

```bash
ros2 component load /ComponentManager \
  rt2_navigation_assignment \
  rt2_navigation_assignment::NavigationActionServer \
  -p use_sim_time:=true

ros2 component load /ComponentManager \
  rt2_navigation_assignment \
  rt2_navigation_assignment::NavigationClientComponent \
  -p use_sim_time:=true
```

---

## Keyboard UI

After the components are loaded, go back to Terminal 2. The prompt looks like:

```
Research Track II navigation UI
  goal X Y THETA   send a pose in odom
  cancel           cancel the active goal
  status           print client state
  help             print this message
  quit             stop the input thread

rt2-ui>
```

### Command reference

| Command | Aliases | Description |
|---|---|---|
| `goal X Y THETA` | `goto`, `target` | Send a target pose. Values must be space-separated decimal numbers. |
| `cancel` | `stop` | Cancel the currently active goal. |
| `status` | — | Print whether a goal is active and whether a cancel was sent. |
| `help` | `?` | Print the command reference. |
| `quit` | `exit` | Stop the keyboard input thread (ROS node keeps running). |

**Correct format:**

```
goal 2.0 1.0 0.0
goal 6.0 3.0 1.57
```

**Common mistakes:**

```
2.0 1.0 0.0          ← missing 'goal' keyword
goal 2.0, 1.0, 0.0  ← comma-separated values not supported
```

The action server rejects goals while another is active. Cancel the current goal first or wait for it to finish.

---

## Useful diagnostic commands

```bash
# Check that the action server is advertising
ros2 action list
ros2 action info /navigate_to_pose

# Check topics
ros2 topic list | grep odom
ros2 topic list | grep cmd_vel

# Watch feedback while a goal is active
ros2 topic echo /navigate_to_pose/_action/feedback

# Check the odom → base_footprint transform
ros2 run tf2_ros tf2_echo odom base_footprint

# Check the goal frame (only exists while a goal is active)
ros2 run tf2_ros tf2_echo base_footprint navigation_goal

# View the full TF tree
ros2 run tf2_tools view_frames
```
