# 6-dof-robotic-arm-ros1-industrial
6 DOF Robotic Arm ROS 1 Noetic Industrial Hardening

<img src="images/overview.png" width=800>

## Architecture

<img src="images/architecture.png" width=800>

### Why ROS 1 Noetic

This project uses ROS 1 Noetic because it is the **industrialized standard** for robotic arm deployments. ROS 1 has been battle-tested in production environments for over a decade, with proven reliability for:

- **Real-time control**: ROS 1 provides deterministic performance critical for 6-DOF manipulator safety
- **MoveIt stability**: MoveIt on ROS 1 has extensive industrial validation with thousands of production deployments
- **Gazebo 11 integration**: Mature simulation environment used by major robotics companies
- **Hardware compatibility**: Most industrial robot drivers, controllers, and sensors have ROS 1 drivers
- **Legacy support**: Existing hardware stacks, PLCs, and safety systems are certified for ROS 1

ROS 2 migration is planned but requires re-validation of all safety certifications. ROS 1 Noetic remains the industrialized standard for production robotic arm systems through 2025 and beyond.

<img src="images/ros1_vs_ros2.png" width=800>

## Repository Structure
```
6-dof-robotic-arm-ros1-industrial/ 
├── owr_description/ # Robot URDF, meshes, and models 
│ ├── meshes/ # STL/DAE collision and visual meshes 
│ ├── objects/ # Table and environment URDFs 
│ └── urdf/ # Robot URDF/Xacro files 
├── owr_gazebo/ # Gazebo simulation launch and config 
│ ├── launch/ # Gazebo launch files 
│ ├── config/ # Controller configurations 
│ └── worlds/ # Gazebo world files 
├── owr_moveit_config/ # MoveIt configuration 
│ ├── launch/ # MoveIt launch files 
│ └── config/ # Planning, kinematics, controllers 
├── owr_manipulation/ # Arm control and safety nodes 
│ └── src/ # C++ source files 
├── owr_gripper_ikfast_arm_manipulator_plugin/ # IKFast solver 
├── docs/ # Documentation 
├── images/ # README images 
└── .github/workflows/ # CI pipeline
```

<img src="images/workspace_structure.png" width=800>

### Robot Model
The robot URDF/Xacro model is included in `owr_description/urdf/`. Meshes are under `owr_description/meshes/`. The model is loaded via `owr_gazebo/robot_6dof_gazebo_spawn.launch`.

<img src="images/robot_model_preview.png" width=800>

The 6-DOF kinematic chain consists of 7 links:

| Link | Mesh (collision) | Role |
|------|------------------|------|
| base_link | `base_link.STL` | Fixed base |
| BS_Link | `BS_Link.STL` | Base shoulder (BJ) |
| SE_Link | `SE_Link.STL` | Shoulder-elbow (SJ/EJ) |
| EW1_Link | `EW1_Link.STL` | Elbow-wrist 1 (EJ/W1J) |
| W12_Link | `W12_Link.STL` | Wrist 1-2 (W1J/W2J) |
| W23_Link | `W23_Link.STL` | Wrist 2-3 (W2J/W3J) |
| W3Eff_Link | `W3Eff_Link.STL` | Wrist 3-end effector (W3J) |

Gripper meshes (Robotiq Arg2f 140) are under `owr_description/meshes/gripper/`.

The kinematic chain (6 revolute joints + gripper):

<img src="images/dof_axes.png" width=650>

### Installation
Clone the repository using:

    git clone https://github.com/sam-black007/6-dof-robotic-arm-ros1-industrial.git

Run catkin_make in your ROS source directory

    $ cd ~/catkin_ws
    $ catkin_make

Start the simulation using:

    $ roslaunch owr_gazebo robot_6dof_gazebo_spawn.launch

Launch moveit and rviz:

    $ roslaunch owr_moveit_config robot_6dof_moveit_sim.launch

### Disable Collision in Rviz
<table>
  <tr>
    <td>Filtered PointCloud Data</td>
     <td>Generated Octomap</td>
     <td>Collision disabled for 3 objects</td>
  </tr>
  <tr>
    <td><img src="images/processed_pointcloud_data.png" width=350 height=150></td>
    <td><img src="images/octomap_generated_from_processed_pointcloud_data.png" width=350 height=150></td>
    <td><img src="images/collision_disabled_for_3_objects.png" width=350 height=150></td>
  </tr>
 </table>

### Perception Pipeline
The arm uses a RGB-D camera to build the collision scene. Here is the full data flow:

<img src="images/perception_pipeline.png" width=800>

### Motion Planning
MoveIt plans obstacle-free motion using the IKFast solver and collision scene:

<img src="images/moveit_planning_flow.png" width=800>

### Inverse Kinematics
The IKFast plugin solves 6 joint angles from a desired end-effector pose:

<img src="images/ik_solver_flow.png" width=800>

### Collision Checking
The planning scene merges the octomap with the Allowed Collision Matrix:

<img src="images/collision_checking.png" width=800>

### Controlling the Arm
You can command the arm through the RViz GUI, C++ nodes, or joint-trajectory messages:

<img src="images/control_modes.png" width=800>

### Pick and Place
End-to-end manipulation sequence from detection to placement:

<img src="images/pick_place_flow.png" width=800>

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| Robot Arm | 6-DOF with Robotiq 2F-140 gripper |
| Controller | Arduino Mega 2560 + RAMPS 1.4 |
| Motors | NEMA 17 stepper motors (42BYGH47) |
| Power Supply | 12V 5A DC (DO NOT use USB power) |
| Computer | Ubuntu 20.04, 8GB RAM minimum |

## Software Stack

| Component | Version | Purpose |
|-----------|---------|---------|
| ROS Noetic | 1.16.0 | Middleware framework |
| Gazebo | 11.11.0 | Physics simulation |
| MoveIt | 1.1.11 | Motion planning |
| Ubuntu | 20.04 LTS | Operating system |
| Python | 3.8 | Scripting |
| C++ | 17 | Performance-critical nodes |

### Requirements
* [robot_vision package](https://github.com/anubhav1772/robot_vision)
* [pcl v1.13.1](https://github.com/PointCloudLibrary/pcl/releases) - [Installation](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html)
* ROS Noetic (Ubuntu 20.04)
* Gazebo v11.11.0

### Parameters
Key launch parameters are in `owr_moveit_config/config/`. Joint limits and DH params are in `owr_description/urdf/`.

### Safety
Safety is enforced at two levels: joint-limit monitoring and a hardware emergency-stop topic.

<img src="images/safety.png" width=800>
<img src="images/safety_workflow.png" width=800>

* Never power servos directly from USB. Use external regulated supply.
* Emergency stop topic: `/emergency_stop`
* Validate joint limits before execution.
* Safety node `owr_manipulation/safety_node.py` monitors limits and emergency stop.

### Launch Flow
<img src="images/launch_flow.png" width=800>

### Quick Start

Follow this workflow to get from clone to running simulation:

<img src="images/user_workflow.png" width=800>

1. Build workspace: `catkin_make`
2. Start Gazebo simulation: `roslaunch owr_gazebo robot_6dof_gazebo_spawn.launch`
3. Start MoveIt: `roslaunch owr_moveit_config robot_6dof_moveit_sim.launch`
4. Verify safety node is running: `rosnode list | grep safety_node`

## Troubleshooting

### Common Issues

**Issue: Gazebo fails to spawn robot**
```bash
# Solution: Kill existing Gazebo instances
pkill -9 gazebo
roslaunch owr_gazebo robot_6dof_gazebo_spawn.launch
```

**Issue: MoveIt can't connect to controller**
```bash
# Solution: Check controller status
rosnode list | grep controller
rosservice call /controller_manager/list_controllers
```

**Issue: Emergency stop not working**
```bash
# Solution: Verify safety node is running
rosnode list | grep safety_node
rostopic echo /emergency_stop
```

**Issue: Joint limits exceeded**
```bash
# Solution: Check joint limits in config
cat owr_moveit_config/config/joint_limits.yaml
```

## Performance Tuning
* **MoveIt Planning**: Use ompl planner for faster planning
* **Gazebo Physics**: Reduce real_time_factor for slower but stable simulation
* **Camera Data**: Reduce point cloud density for faster processing

## Continuous Integration
Every push and pull request is built and validated on Ubuntu 20.04 + ROS Noetic:

<img src="images/ci_pipeline.png" width=800>

## Industrial Hardening Summary
This repository was hardened from the original project into an industrial baseline:

<img src="images/hardening_summary.png" width=800>

## Contributing
1. Fork the repository
2. Create feature branch: `git checkout -b feature/amazing-feature`
3. Commit changes: `git commit -m 'Add amazing feature'`
4. Push to branch: `git push origin feature/amazing-feature`
5. Open Pull Request

## License
This project is licensed under the MIT License - see LICENSE file for details.

## Acknowledgments
* Original project: anubhav1772/6-dof-robotic-arm
* Industrial hardening: sam-black007
* MoveIt configuration templates
* Gazebo ROS control tutorials

### Industrial Checklist
See `docs/ROS1_INDUSTRIAL_CHECKLIST.md`.
