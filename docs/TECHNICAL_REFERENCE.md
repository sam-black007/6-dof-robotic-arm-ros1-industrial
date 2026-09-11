# Technical Reference

Complete engineering reference for the OWR 6-DOF Arm with Robotiq 2F-140 gripper. All values are derived from the URDF source at `owr_description/urdf/owr.urdf.xacro` and the controller configuration at `owr_gazebo/config/controllers.yaml`.

**Last verified:** 2026-09-10 against commit `48c2645`.

---

## 1. Kinematics

### 1.1 Joint Table

| Joint | Axis  | Type       | Lower (rad) | Upper (rad) | Lower (deg) | Upper (deg) | Velocity (rad/s) | Effort (N·m) | Mass (kg) |
|-------|-------|------------|-------------|-------------|-------------|-------------|------------------|--------------|-----------|
| `BJ`  | Z     | Revolute   | −2.0944     | 2.0944      | −120        | 120         | 5                | 200          | 2.004     |
| `SJ`  | Z     | Revolute   | −1.5708     | 1.5708      | −90         | 90          | 5                | 200          | 1.976     |
| `EJ`  | Z     | Revolute   | −3.9270     | 1.0472      | −225        | 60          | 5                | 200          | 6.924     |
| `W1J` | Z     | Revolute   | −1.5708     | 1.5708      | −90         | 90          | 5                | 200          | 1.641     |
| `W2J` | Z     | Revolute   | −1.0472     | 2.6180      | −60         | 150         | 5                | 200          | 2.384     |
| `W3J` | Z     | Revolute   | −3.1416     | 3.1416      | −180        | 180         | 5                | 200          | 2.168     |
| `finger_joint` | Z | Prismatic | 0.0        | 0.04        | 0           | 2.3         | —                | —            | 0.543 (W3Eff) |

All six arm joints use `PositionJointInterface` via `SimpleTransmission` with a 1:1 mechanical reduction ratio.

### 1.2 Joint Origins (URDF Fixed-Transform Chain)

```
world ──(fixed: 0,0,0.75)──► base_link
                               │
                         BJ ──── (0, 0, 0.1181)
                               │
                         SJ ──── (0, 0.1157, 0.0775)
                               │
                         EJ ──── (0, 0, 0.35575)
                               │
                         W1J ─── (0.0695, −0.1157, 0)
                               │
                         W2J ─── (0.28625, 0, 0)
                               │
                         W3J ─── (0.0635, 0, 0.12)
                               │
                       EEF_Link ── (0.0675, 0, 0)  [fixed]
```

Approximate total reach from base to EEF: **~560 mm**.

### 1.3 Forward Kinematics (URDF Approach)

This arm does **not** use standard DH parameters. The URDF describes each joint as a fixed XYZ + RPY transform from the parent link's origin. Forward kinematics is computed by chaining these transforms:

```
T_world→EEF = T_world→base × T_BJ(θ₁) × T_SJ(θ₂) × T_EJ(θ₃) × T_W1J(θ₄) × T_W2J(θ₅) × T_W3J(θ₆) × T_EEF
```

Each `T_Joint(θᵢ)` is:

```
T(θ) = Rot_z(θ) × Trans(link_offset)
     = [ cos(θ)  -sin(θ)  0  0 ]   [ 1  0  0  dx ]
       [ sin(θ)   cos(θ)  0  0 ] × [ 0  1  0  dy ]
       [   0        0     1  0 ]   [ 0  0  1  dz ]
       [   0        0     0  1 ]   [ 0  0  0   1 ]
```

The link offsets (`dx, dy, dz`) are the values from the URDF `<origin xyz=...>` tags listed above.

**Example — EEF position when all joints at 0:**

```
x = 0 + 0 + 0 + 0.0695 + 0.28625 + 0.0635 + 0.0675 = 0.48675 m
y = 0 + 0.1157 + 0 + (−0.1157) + 0 + 0 + 0 = 0 m
z = 0.75 + 0.1181 + 0.0775 + 0.35575 + 0 + 0.12 + 0 = 1.42135 m
```

So in the zero pose the EEF sits approximately at **(0.487, 0, 1.421)** in world frame.

### 1.4 IK Solver

| Parameter | Value |
|-----------|-------|
| Plugin    | `owr_gripper_arm_manipulator_kinematics/IKFastKinematicsPlugin` |
| Search resolution | 0.005 rad |
| Solver timeout | 5 ms |

IKFast is an analytical (closed-form) solver — deterministic, fast, and pose-limited (no redundancy resolution since 6-DOF = 6-DOF IK, no null-space).

**IKFast limitations:**
- Cannot handle multiple solutions gracefully — returns the closest solution to the seed pose.
- Seed pose is critical — always set it to the current joint configuration before calling IK.
- Fails silently if no solution exists — always check the return value.

---

## 2. Controller Architecture

### 2.1 Active Controllers (`owr_gazebo/config/controllers.yaml`)

#### `arm_manipulator_controller`

| Field | Value |
|-------|-------|
| Type | `position_controllers/JointTrajectoryController` |
| Joints | BJ, SJ, EJ, W1J, W2J, W3J |
| Goal time | 1.0 s |
| Stopped velocity tolerance | 0.05 rad/s |
| Stop trajectory duration | 0.5 s |
| State publish rate | 50 Hz |
| Action monitor rate | 10 Hz |
| Allow partial joints | true |
| Trajectory tolerance | ±0.1 rad per joint |
| Goal tolerance | ±0.1 rad per joint |

**Action server:** `/arm_manipulator_controller/follow_joint_trajectory` (`control_msgs/FollowJointTrajectory`)

#### `gripper_trajectory_controller`

| Field | Value |
|-------|-------|
| Type | `position_controllers/JointTrajectoryController` |
| Joints | finger_joint |
| Goal time | 0.6 s |
| Stopped velocity tolerance | 0.05 rad/s |
| State publish rate | 50 Hz |
| Action monitor rate | 20 Hz |

**Action server:** `/gripper_trajectory_controller/follow_joint_trajectory` (`control_msgs/FollowJointTrajectory`)

#### `joint_state_controller`

Publishes `/joint_states` at 50 Hz. Type: `joint_state_controller/JointStateController`.

#### `joint_group_position_controller` (backup / direct position mode)

| Field | Value |
|-------|-------|
| Type | `position_controllers/JointGroupPositionController` |
| Joints | BJ, SJ, EJ, W1J, W2J, W3J |

**Topic:** `/joint_group_position_controller/command` (`trajectory_msgs/JointTrajectory`)

### 2.2 Gazebo PID Gains (`gazebo_ros_control_params.yaml`)

The arm is controlled in Gazebo via the `gazebo_ros_control` ROS interface with **position** PID control. Gains are loaded from `owr_gazebo/config/gazebo_ros_control_params.yaml` in `owr_control.launch`:

```yaml
gazebo_ros_control/pid_gains:
  BJ:  {p: 100, d: 0.2,  i: 100}
  SJ:  {p: 100, d: 1.0,  i: 200}
  EJ:  {p: 100, d: 0.2,  i: 100}
  W1J: {p: 100, d: 0.2,  i: 100}
  W2J: {p: 100, d: 0.2,  i: 100}
  W3J: {p: 100, d: 0.2,  i: 100}
```

**Tuning guide:**
- Increase `p` for faster response (risk: oscillation).
- Keep `d` in the 0.2–1.0 range to damp overshoot; `SJ` carries the heaviest link (6.9 kg) and uses the largest derivative.
- `i` is used here to eliminate steady-state droop from gravity; values >200 risk integral windup under sustained collision contact.
- These gains are position-loop gains on the raw joint actuator (transmission ratio 1:1).

### 2.3 Transmission Hardware Interface

All joints use `hardware_interface/PositionJointInterface`. This is the simplest `ros_control` interface — the controller sends a target position and the Gazebo PID plugin (or real servo firmware) drives to it.

**Transmission URDF snippet (`owr.transmission.xacro`):**

```xml
<transmission name="BJ_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="BJ">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="BJ_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

No velocity or effort interfaces are currently active. To switch to effort control:

1. Change `transmission_hw_interface` arg in `owr_robot.urdf.xacro`
2. Update controller types in `owr_gazebo/config/controllers.yaml`
3. Update PID gains in `owr.gazebo.xacro`

---

## 3. ROS Topic / Service / Action Reference

### 3.1 Published Topics

| Topic | Type | Publisher | Rate |
|-------|------|-----------|------|
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_controller` | 50 Hz |
| `/arm_manipulator_controller/state` | `control_msgs/JointTrajectoryControllerStatus` | `arm_manipulator_controller` | 50 Hz |
| `/gripper_trajectory_controller/state` | `control_msgs/JointTrajectoryControllerStatus` | `gripper_trajectory_controller` | 50 Hz |
| `/emergency_stop` | `std_msgs/Bool` | `safety_node` | 10 Hz |

### 3.2 Subscribed Topics

| Topic | Type | Subscriber |
|-------|------|------------|
| `/emergency_stop` | `std_msgs/Bool` | `arm_manipulator_controller`, `gripper_trajectory_controller`, all C++ nodes |

### 3.3 Action Servers

| Action | Type | Server |
|--------|------|--------|
| `/arm_manipulator_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | `arm_manipulator_controller` |
| `/gripper_trajectory_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | `gripper_trajectory_controller` |

### 3.4 Action Clients

| Action | Type | Client |
|--------|------|--------|
| `/arm_manipulator_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | `MoveGroupInterface`, `ArmMotion`, `PickNPlace` |
| `/gripper_trajectory_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | `MoveGroupInterface`, `PickNPlace` |

> `safety_node` publishes `/emergency_stop` as a plain `std_msgs/Bool` **topic** (see §3.1); it does not use an action server.

---

## 4. C++ Node Reference (`owr_manipulation`)

All nodes are built against `roscpp`, `actionlib`, `control_msgs`, `trajectory_msgs`, `moveit_core`, `moveit_ros_planning_interface`, `moveit_ros_perception`, `moveit_visual_tools`, `octomap`, and `tf2_ros`.

| Node | Source | Function |
|------|--------|----------|
| `ArmMotion` | `src/ArmMotion.cpp` | Generates random joint-space trajectories; publishes to `arm_manipulator_controller` action server. Useful for motion smoke-testing. |
| `joint_trajectory_control` | `src/joint_trajectory_control.cpp` | Subscribes to a `trajectory_msgs/JointTrajectory` on `/move_group/goal` and forwards to the arm controller. Exposes `/emergency_stop` subscriber to halt. |
| `PickNPlace` | `src/PickNPlace.cpp` | Full pick-and-place sequence: move to pre-grasp pose, close gripper, lift, move to place pose, open gripper. Uses MoveIt MoveGroup interface. |
| `PickNPlaceTest` | `src/PickNPlaceTest.cpp` | Unit-test harness for `PickNPlace`; runs a canned sequence and checks joint-state convergence. |
| `planning_scene_node` | `src/planning_scene.cpp` | Builds a static planning scene (collision objects, world octomap seed). Loads point cloud from file and publishes as a `moveit_msgs/PlanningScene`. |

### `state_publisher_node` (`owr_gazebo/src/move_to_joint_sim.cpp`)

Moves joints from a hardcoded initial pose to a target pose with cubic interpolation. Used in Gazebo warm-up before MoveIt takes over. Publishes joint states on `/joint_states`.

---

## 5. MoveIt Configuration (`owr_moveit_config`)

### 5.1 Planning Group

| Group | Joints | IK Solver |
|-------|--------|-----------|
| `arm_manipulator` | BJ, SJ, EJ, W1J, W2J, W3J | IKFast (`owr_gripper_arm_manipulator_kinematics`) |

End-effector group: `gripper` → `finger_joint` only.

### 5.2 MoveIt Controller Interface (`simple_moveit_controllers.yaml`)

```yaml
controller_list:
  - name: arm_manipulator_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    joints: [BJ, SJ, EJ, W1J, W2J, W3J]
  - name: gripper_trajectory_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    joints: [finger_joint]
```

### 5.3 Key Parameters (`kinematics.yaml`)

```yaml
arm_manipulator:
  kinematics_solver: owr_gripper_arm_manipulator_kinematics/IKFastKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

### 5.4 Joint Limits Override (`joint_limits.yaml`)

The MoveIt config provides its own `joint_limits.yaml` which may override the URDF limits for planning purposes (velocity scaling, soft-limits). Check the file directly for exact values used during planning.

### 5.5 Planning Pipeline

The default planning pipeline is OMPL (Open Motion Planning Library). Available planners:

| Planner | Type | Best for |
|---------|------|----------|
| `RRTConnect` | Sampling-based, bidirectional | Fast, general-purpose (default) |
| `RRT*` | Sampling-based, asymptotically optimal | Optimal paths, slower |
| `PRM` | Sampling-based, multi-query | Static environments |
| `CHOMP` | Optimization-based | Smooth trajectories, needs good seed |
| `STOMP` | Optimization-based | Smooth, stochastic |

**Planning request parameters:**

| Parameter | Default | Notes |
|-----------|---------|-------|
| `planning_time` | 5.0 s | Max time for planner to find a solution |
| `num_planning_attempts` | 10 | Retry count if first solution fails validation |
| `max_velocity_scaling_factor` | 1.0 | Scale down for slower, safer motion |
| `max_acceleration_scaling_factor` | 1.0 | Scale down for smoother acceleration |

### 5.6 Planning Scene Components

```
┌─────────────────────────────────────────────────┐
│                 Planning Scene                  │
│                                                 │
│  ┌──────────────┐  ┌──────────────┐            │
│  │  Robot State │  │  World OctoMap│            │
│  │  (joint pos) │  │  (from RGBD)  │            │
│  └──────┬───────┘  └──────┬───────┘            │
│         │                  │                    │
│         ▼                  ▼                    │
│  ┌──────────────────────────────┐              │
│  │     Allowed Collision Matrix │              │
│  │  (ACM)                       │              │
│  └──────────────┬───────────────┘              │
│                 │                               │
│                 ▼                               │
│  ┌──────────────────────────────┐              │
│  │   Collision World (FCL)      │              │
│  │  - Robot link geometries     │              │
│  │  - Octomap voxels            │              │
│  │  - Attached objects           │              │
│  └──────────────┬───────────────┘              │
│                 │                               │
│                 ▼                               │
│  ┌──────────────────────────────┐              │
│  │   Motion Planner (OMPL)      │              │
│  │  - Collision-free paths      │              │
│  │  - Joint limit bounds        │              │
│  └──────────────────────────────┘              │
└─────────────────────────────────────────────────┘
```

The planning scene merges:
1. **Robot state** — current joint positions from `/joint_states`
2. **World octomap** — voxelized collision environment from RGB-D camera
3. **ACM** — pairs of links that are allowed to collide (e.g., adjacent links)
4. **Attached objects** — objects currently grasped by the gripper

---

## 6. Safety System

### 6.1 `owr_manipulation/safety_node.py`

Python 2/3 ROS node (under development) implementing:

| Feature | Implementation |
|---------|----------------|
| Joint-limit monitoring | Subscribes to `/joint_states`, compares against URDF limits per joint |
| Emergency stop output | Publishes `std_msgs/Bool` on `/emergency_stop` |
| Hardware E-stop input | Subscribes to `/emergency_stop` from external button (hardware) |
| Watchdog | Timer-based; triggers E-stop if no heartbeat received within timeout |

### 6.2 Safety Node Implementation

**Canonical source:** [`owr_manipulation/safety_node.py`](../owr_manipulation/safety_node.py)

The node runs as a standalone ROS process (`rosrun owr_manipulation safety_node.py`) and implements:

| Component | Detail |
|-----------|--------|
| Joint-limit watchdog | Subscribes to `/joint_states`; compares each joint's position against the URDF limits with a **0.05 rad margin**; latches emergency stop on breach. |
| Emergency-stop latch | `std_msgs/Bool` published on `/emergency_stop` at **10 Hz**; once latched, remains active until the node is restarted. |
| Hardware E-stop input | Subscribes to `/emergency_stop` from an external button; mirrors it by re-publishing. |
| Finger guard | Checks `finger_joint` against 0.0–0.04 m and trips on over-travel. |

### 6.3 Joint Limit Enforcement

Hard limits are set in URDF (see §1.1) and enforced at three levels:

```
┌─────────────────────────────────────────────────────────┐
│               Joint Limit Enforcement Stack             │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  Layer 3: MoveIt Planning                              │
│  ┌──────────────────────────────────────┐              │
│  │  joint_limits.yaml bounds            │              │
│  │  + velocity/acceleration scaling     │              │
│  │  Rejects goals beyond bounds         │              │
│  └──────────────────┬───────────────────┘              │
│                     │                                   │
│  Layer 2: Controller Runtime                            │
│  ┌──────────────────▼───────────────────┐              │
│  │  trajectory_msgs/JointTrajectory     │              │
│  │  ±0.1 rad goal tolerance             │              │
│  │  velocity threshold: 0.05 rad/s      │              │
│  └──────────────────┬───────────────────┘              │
│                     │                                   │
│  Layer 1: Safety Node                                   │
│  ┌──────────────────▼───────────────────┐              │
│  │  safety_node.py watchdog             │              │
│  │  Compares /joint_states vs URDF      │              │
│  │  Triggers /emergency_stop on breach  │              │
│  └──────────────────┬───────────────────┘              │
│                     │                                   │
│  Layer 0: URDF Hard Limits (Physical)                   │
│  ┌──────────────────▼───────────────────┐              │
│  │  BJ: ±120°  SJ: ±90°  EJ: −225°/60° │              │
│  │  W1J: ±90°  W2J: −60°/150°  W3J: ±180° │          │
│  └──────────────────────────────────────┘              │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

> **Important:** The URDF hard limits are the physical constraint. MoveIt soft-limits may be tighter. Always validate against the URDF.

### 6.4 Emergency Stop Wiring (Hardware)

For real hardware, the E-stop button must be wired as:

```
E-stop button ──► RAMPS 1.4 VIN cut ──► Motors disabled
              └──► Arduino D2 (interrupt) ──► ROS /emergency_stop publisher
```

**Software E-stop alone is not sufficient for safe operation.** The hardware E-stop must切断电机电源 independently of the ROS system.

---

## 7. Gazebo Simulation

### 7.1 Launch Arguments (`robot_6dof_gazebo_spawn.launch`)

| Argument | Default | Description |
|----------|---------|-------------|
| `paused` | false | Start Gazebo in paused state |
| `gui` | false | Launch Gazebo GUI |
| `debug` | false | Enable GDB debug |
| `headless` | false | Run without rendering |
| `use_sim_time` | true | Use simulated clock |
| `world_name` | `demo.world` | Gazebo world file |
| `world_pose` | `-x 0 -y 0 -z 0 -R 0 -P 0 -Y 0` | Spawn pose |
| `initial_joint_positions` | All joints at 0, finger at 0.04 | Initial joint configuration |

### 7.2 Available Worlds

| World | Description |
|-------|-------------|
| `demo.world` | Empty world with Kinect 3D camera |
| `setup_1.world` | Table with single object + Kinect camera |
| `setup_2.world` | Table with multiple objects + Kinect camera |
| `pick_place.world` | Dedicated pick-and-place workspace |
| `factory.world` | Factory setup with suspended chair |

### 7.3 Gazebo Plugins (`owr.gazebo.xacro`)

- `gazebo_ros_control` — joint transmission interface
- `gazebo_ros_force_tracking` — external force/torque sensor
- `ros_control` — PID position control (default gains)

---

## 8. Perception Pipeline

### 8.1 Data Flow

```
┌──────────────┐     ┌──────────────────┐     ┌──────────────┐     ┌──────────────┐
│  RGB-D Camera │────►│  Point Cloud      │────►│  Voxel Grid  │────►│  OctoMap      │
│  (Kinect)     │     │  /camera/point_   │     │  Filter      │     │  /world_octo  │
│               │     │  cloud             │     │  (downsample)│     │  map           │
└──────────────┘     └──────────────────┘     └──────────────┘     └───────┬──────┘
                                                                          │
                                                                          ▼
                                                                 ┌──────────────┐
                                                                 │  MoveIt      │
                                                                 │  Planning   │
                                                                 │  Scene      │
                                                                 └──────────────┘
```

### 8.2 Point Cloud Processing

| Stage | Topic | Message Type | Rate |
|-------|-------|--------------|------|
| Raw RGB-D | `/camera/rgb/image_raw` | `sensor_msgs/Image` | 30 Hz |
| Raw depth | `/camera/depth/image_raw` | `sensor_msgs/Image` | 30 Hz |
| Combined point cloud | `/camera/point_cloud` | `sensor_msgs/PointCloud2` | 30 Hz |
| Filtered cloud | `/camera/filtered_cloud` | `sensor_msgs/PointCloud2` | 10 Hz |
| OctoMap | `/world_octomap` | `octomap_msgs/Octomap` | 10 Hz |

### 8.3 Voxel Grid Filter Parameters

```yaml
leaf_size: 0.01          # 1 cm voxel size
min_x: -1.0              # Scene bounds
max_x:  1.0
min_y: -1.0
max_y:  1.0
min_z:  0.0
max_z:  2.0
```

### 8.4 OctoMap Parameters

```yaml
resolution: 0.05         # 5 cm voxels
max_range: 3.0           # Max sensor range
min_range: 0.1           # Min sensor range (too close = noise)
```

---

## 9. Pick-and-Place Sequence (`owr_manipulation/PickNPlace`)

### 9.1 State Machine

```
┌────────────┐
│  START     │
└─────┬──────┘
      ▼
┌────────────┐
│  APPROACH  │──── Move to pre-grasp pose (above object)
└─────┬──────┘
      ▼
┌────────────┐
│  DESCEND   │──── Move down to grasp height
└─────┬──────┘
      ▼
┌────────────┐
│  GRASP     │──── Close gripper (finger_joint → 0.0)
└─────┬──────┘
      ▼
┌────────────┐
│  LIFT      │──── Move up to safe height
└─────┬──────┘
      ▼
┌────────────┐
│  TRANSPORT │──── Move to place pose
└─────┬──────┘
      ▼
┌────────────┐
│  PLACE     │──── Move down to place height
└─────┬──────┘
      ▼
┌────────────┐
│  RELEASE   │──── Open gripper (finger_joint → 0.04)
└─────┬──────┘
      ▼
┌────────────┐
│  RETREAT   │──── Move back to home pose
└────────────┘
```

### 9.2 PickNPlace Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Pre-grasp height | 0.15 m above object | Safe approach distance |
| Grasp height | 0.05 m above table | Close enough to grasp |
| Lift height | 0.20 m above table | Clear obstacles |
| Place height | 0.05 m above table | Gentle placement |
| Grasp pose | `ee_pos + (0, 0, -0.05)` | Approach from above |
| Place pose | `ee_pos + (0.3, 0, 0)` | Offset from pick |

### 9.3 MoveIt MoveGroup Interface (C++)

```cpp
#include <moveit/move_group_interface/move_group_interface.h>

// Initialize
moveit::planning_interface::MoveGroupInterface move_group("arm_manipulator");

// Set target
move_group.setNamedTarget("home");
// or
geometry_msgs::Pose target_pose;
target_pose.position.x = 0.4;
target_pose.position.y = 0.0;
target_pose.position.z = 0.3;
target_pose.orientation.w = 1.0;
move_group.setPoseTarget(target_pose);

// Plan and execute
moveit::planning_interface::MoveGroupInterface::Plan plan;
move_group.plan(plan);
move_group.execute(plan);
```

---

## 10. URDF Package Structure (`owr_description`)

```
owr_description/
├── urdf/
│   ├── owr_robot.urdf.xacro      # Top-level: includes everything, sets world→base_link
│   ├── owr.urdf.xacro            # Arm kinematics, joints, links, meshes, inertials
│   ├── owr.transmission.xacro    # SimpleTransmission per joint (PositionJointInterface)
│   ├── owr.gazebo.xacro          # Gazebo plugins, PID, material colors
│   ├── common.gazebo.xacro       # Gazebo physics, gravity, wind, default materials
│   ├── kinect_sensor.urdf.xacro  # Kinect RGB-D sensor definition
│   └── gripper/                  # Robotiq 2F-140 URDF (currently excluded from owr_robot)
├── meshes/
│   ├── collision/                # STL files for collision detection
│   │   ├── base_link.STL
│   │   ├── BS_Link.STL
│   │   ├── SE_Link.STL
│   │   ├── EW1_Link.STL
│   │   ├── W12_Link.STL
│   │   ├── W23_Link.STL
│   │   ├── W3Eff_Link.STL
│   │   └── gripper/              # Robotiq 2F-140 collision meshes
│   └── visual/                   # DAE files for rendering (currently missing from repo)
└── config/
    └── joint_limits.yaml         # MoveIt joint limit overrides
```

---

## 11. Build & Dependencies

### 11.1 Required System

| Component | Specification |
|-----------|---------------|
| OS | Ubuntu 20.04 LTS |
| ROS | ROS Noetic (full desktop install) |
| Gazebo | Gazebo 11 (bundled with ros-noetic-desktop-full) |
| CPU | 4+ cores recommended |
| RAM | 8 GB minimum, 16 GB recommended |
| GPU | OpenGL 3.3+ (for Gazebo rendering) |

### 11.2 ROS Package Dependencies

#### `owr_description`
No ROS package dependencies (URDF only).

#### `owr_gazebo`
| Dependency | Type |
|------------|------|
| `gazebo_ros` | exec |
| `roscpp` | exec |
| `rospy` | exec |
| `std_msgs` | exec |
| `owr_description` | exec |

#### `owr_moveit_config`
| Dependency | Type |
|------------|------|
| `boost` | exec |
| `orocos_kdl` | exec |
| `trac_ik_lib` | exec |
| `pluginlib` | exec |

#### `owr_manipulation`
| Dependency | Type |
|------------|------|
| `actionlib` | exec |
| `control_msgs` | exec |
| `roscpp` | exec |
| `trajectory_msgs` | exec |
| `moveit_core` | exec |
| `moveit_ros_planning_interface` | exec |
| `moveit_ros_perception` | exec |
| `moveit_visual_tools` | exec |
| `octomap` | exec |
| `tf2_ros` | exec |

### 11.3 System Install

```bash
# Install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update && sudo apt install ros-noetic-desktop-full

# Install MoveIt + Gazebo plugins
sudo apt install ros-noetic-moveit ros-noetic-gazebo-ros ros-noetic-ros-control

# IKFast plugin (for this specific arm — must be built from source or use provided binary)
# See owr_moveit_config/CMakeLists.txt for the IKFast plugin package name
```

### 11.4 Workspace Build

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/sam-black007/6-dof-robotic-arm-ros1-industrial.git
cd ..
catkin_make
source devel/setup.bash
```

---

## 12. Quick-Start Launch Sequence

```bash
# Terminal 1 — Start Gazebo with arm
roslaunch owr_gazebo robot_6dof_gazebo_spawn.launch gui:=true

# Terminal 2 — Start MoveIt
roslaunch owr_moveit_config robot_6dof_moveit_sim.launch

# Terminal 3 — (Optional) Run safety node
rosrun owr_manipulation safety_node.py

# Terminal 4 — (Optional) Pick and place
rosrun owr_manipulation PickNPlace
```

**Verify running nodes:**

```bash
rosnode list
rostopic echo /joint_states -n1
rosservice call /gazebo/unpause_physics
```

---

## 13. Frame Tree

```
world
 └── base_link  (fixed at z=0.75)
      └── BJ_link
           └── SJ_link
                └── SE_Link
                     └── EW1_Link
                          └── W12_Link
                               └── W23_Link
                                    └── W3Eff_Link
                                         └── EEF_Link  (fixed, tool frame)
```

All transforms are published by `joint_state_publisher` / `robot_state_publisher` from `/joint_states`.

---

## 14. Industrial Hardening Checklist

See `docs/ROS1_INDUSTRIAL_CHECKLIST.md` for the full item-by-item checklist. Key items completed in this repository:

- [x] CI on every push (`ros1-ci.yml`)
- [x] `requirements.txt` pinned with hashes
- [x] Safety node skeleton (`owr_manipulation/safety_node.py`)
- [x] URDF mesh reference fix (base_link duplicate geometry removed)
- [x] Sole contributor (`sam-black007`) in all metadata
- [x] MIT license
- [x] `.gitignore` with ROS/Python/C++ patterns

---

## Appendix A: IKFast Joint Order

IKFast is generated per planning-group joint order. For this arm the solver expects:

```
[BJ, SJ, EJ, W1J, W2J, W3J]
```

The solver file is compiled into the `owr_gripper_arm_manipulator_kinematics` plugin package and loaded at runtime via `pluginlib`.

## Appendix B: Diagram Generation Prompts

Nine technical diagrams are used in this documentation. Use these prompts with your preferred image generator (e.g. ImageMagick, DALL·E, Midjourney, Stable Diffusion) to regenerate or extend the set. Keep the same flat vector style — dark navy background, white sans-serif text, cyan/amber/red accents — so all diagrams match.

### B.1 `kinematic_chain.png` — Forward Kinematics Chain

> Flat vector technical diagram, dark navy background. Draw a 6-DOF robotic arm as a stick-figure chain of 7 rectangles (base_link, BJ_link, SJ_link, SE_Link, EW1_Link, W12_Link, W23_Link, W3Eff_Link) connected by circular revolute joints labeled BJ, SJ, EJ, W1J, W2J, W3J. Next to each joint print the URDF origin offset in meters: BJ (0, 0, 0.1181), SJ (0, 0.1157, 0.0775), EJ (0, 0, 0.35575), W1J (0.0695, −0.1157, 0), W2J (0.28625, 0, 0), W3J (0.0635, 0, 0.12). Add a coordinate axes triad at the base (world frame, z-up) and at the EEF. Title top-center: "OWR Forward Kinematics — URDF Transform Chain". White sans-serif text, cyan joint markers, thin amber dimension lines. Clean, professional, no photorealism.

### B.2 `dh_table.png` — Joint Limits & Mass Properties

> Flat vector technical data table diagram, dark navy background. Draw a clean bordered table with 7 columns (Joint, Lower rad, Upper rad, Velocity rad/s, Effort N·m, Mass kg, Axis) and 6 rows per revolute joint plus one for finger_joint. Values: BJ ±2.0944, v5, e200, m2.004; SJ ±1.5708, v5, e200, m1.976; EJ −3.9270/1.0472, v5, e200, m6.924; W1J ±1.5708, v5, e200, m1.641; W2J −1.0472/2.6180, v5, e200, m2.384; W3J ±3.1416, v5, e200, m2.168; finger_joint 0.0–0.04 prismatic. Header row filled amber, joint names in cyan bold. Title top-center: "OWR 6-DOF Joint Specification". Flat, monospace-friendly grid, export at 1600×1000 PNG, sharp crisp text.

### B.3 `controller_stack.png` — ROS Control Controller Stack

> Flat vector layered architecture diagram, dark navy background. Three stacked layers from top: (1) "User / MoveIt" box with action client arrows; (2) "Controller Manager" box (ros_control) holding three child boxes labeled arm_manipulator_controller (position_controllers/JointTrajectoryController, 6 joints), gripper_trajectory_controller (1 joint finger_joint), joint_state_controller (50 Hz); (3) "Gazebo Hardware" box labeled gazebo_ros_control + PositionJointInterface + PID. Connect layers with labeled arrows: follow_joint_trajectory action server (control_msgs), /joint_states (sensor_msgs). Add side annotation: goal tolerance ±0.1 rad, stopped velocity 0.05 rad/s. Cyan boxes, amber arrows, white sans-serif text. Title top-center: "ros_control Controller Architecture".

### B.4 `moveit_pipeline.png` — MoveIt Motion Planning Pipeline

> Flat vector data-flow diagram, dark navy background. Horizontal pipeline of 6 boxes connected by arrows: "Goal Pose" → "IKFast Solver" (annotated seed ← current joints) → "Planning Scene" (robot state + world octomap + ACM) → "OMPL Planner" (RRTConnect default, 5 s limit) → "Trajectory Validation" (joint limits ±0.1 rad) → "Trajectory Execution" (FollowJointTrajectory action → arm controller). Color-code: green for planning-internal nodes, amber for validation, cyan for execution. Label each arrow with the message/service used. Title top-center: "MoveIt Motion Planning Pipeline". Whitespace-friendly, crisp vector lines.

### B.5 `safety_stack.png` — Three-Layer Safety Enforcement

> Vertical stack diagram, dark navy background, four horizontal layers stacked top-to-bottom connected by downward arrows: Layer 4 "MoveIt Planning — joint_limits.yaml bounds, rejects goals", Layer 3 "Controller Runtime — ±0.1 rad goal tolerance", Layer 2 "safety_node.py Watchdog — compares /joint_states vs URDF, triggers /emergency_stop", Layer 1 "URDF Hard Limits — physical stop". Layer 1 is red, Layer 2 is amber, Layers 3-4 cyan. Add a right-side red panic button icon labeled "Hardware E-Stop (power cut, independent of ROS)". White sans-serif text. Title top-center: "Joint Limit Enforcement Stack".

### B.6 `pid_control.png` — Gazebo PID Position Control Loop

> Classical control-loop block diagram, dark navy background. Standard feedback loop: setpoint (desired joint position) → subtractor (▲ error symbol) → PID box (P=100, I=0, D=0.1 per joint) → "PositionJointInterface / actuator" → output joint position, feedback line tapped back into the subtractor from a measurement point annotated "Gazebo physics 1000 Hz". One loop per joint (draw 6 identical small loops in a column labeled BJ, SJ, EJ, W1J, W2J, W3J). Amber/cyan color scheme, white sans-serif monospace labels. Title top-center: "Gazebo PID Position Control (ros_control)".

### B.7 `perception_pipeline.png` — RGB-D Perception to OctoMap

> Flat vector pipeline diagram, dark navy background. Four-stage horizontal flow: "Kinect RGB-D" (boxes rgb/image_raw, depth/image_raw at 30 Hz) → "Point Cloud /camera/point_cloud PointCloud2" → "Voxel Grid Filter" (leaf 1 cm, scene bounds ±1 m) → "OctoMap /world_octomap" (5 cm voxels, max range 3 m) → "MoveIt Planning Scene". Under each box print the topic name and message type in monospace. Green voxel grid icon at stage 3, 3D grid icon at stage 4. White sans-serif text, cyan connectors. Title top-center: "RGB-D Perception Pipeline".

### B.8 `pick_place_sm.png` — Pick-and-Place State Machine

> Flat vector UML-style state machine diagram, dark navy background. Nine rounded-rectangle states in a vertical flow connected by annotated arrows: APPROACH (move to 0.15 m above object), DESCEND (to 0.05 m height), GRASP (finger_joint → 0.0), LIFT (to 0.20 m), TRANSPORT (move to place pose), PLACE (descend to 0.05 m), RELEASE (finger_joint → 0.04), RETREAT (return to home). Initial state marker (•) pointing into APPROACH, final state marker (⦿) at RETREAT. State fill: cyan for arm motion, amber for gripper actions. White sans-serif text. Title top-center: "PickNPlace Sequence — Pick-and-Place State Machine".

### B.9 `ros_interface.png` — ROS Topic Graph (rqt_graph style)

> Node-and-cloud graph in rqt_graph style, dark navy background. Draw these nodes as rounded boxes: joint_state_controller, arm_manipulator_controller, gripper_trajectory_controller, safety_node, cifm_group (command_iframe), PickNPlace, ArmMotion, move_group. Connect with labeled arrows: joint_state_controller → /joint_states (sensor_msgs/JointState, 50 Hz) → safety_node, move_group; ArmMotion & PickNPlace → /arm_manipulator_controller/follow_joint_trajectory (control_msgs/FollowJointTrajectory) → arm_manipulator_controller; safety_node → /emergency_stop (std_msgs/Bool) → all controllers. Color nodes: ROS core in cyan, user nodes in amber, safety in red. White sans-serif labels, clean edges. Title top-center: "OWR ROS Computation Graph".
