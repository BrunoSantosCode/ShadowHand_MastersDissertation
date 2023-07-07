# Postural and Hand Control of a Robotic System for Detailed Tasks

This repository contains the code developed in **Master's Dissertation** context.

The `src` folder contains packages specifically implemented to run inside the Shadow Dexterous Hand docker container.

Packages description:
  - `openpose_mujoco` - includes the code for the kinematic retargeting algorithms **Direct Joints Mapping** and **DexPilot**;
  - `human_robot_map` - includes the code for the kinematic retargeting algorithm **BioIK** and for the **Automotive Painting Quality Control** task;
  - `teach_net_teleoperation` and `shadow_teleop` - includes the code from the end-to-end algorithm **TeachNET** (adapted from [(TeachNet_Teleoperation repository)]([https://github.com/jorgef1299/SERP](https://github.com/TAMS-Group/TeachNet_Teleoperation)));
  - `shadow_pinch_teleop` - includes the code for the **Wiring Fitting** task;

## References

 - [DexPilot](https://doi.org/10.1109/ICRA40945.2020.9197124)
 - [BioIK](https://doi.org/10.1109/ICRA.2018.8460799)
 - [TeachNET](https://doi.org/10.1109/ICRA.2019.8794277)

## How to run

### Direct Joints Mapping

1. Run OpenPose

2. Run Shadow Hand MuJoCo simulator
    ```bash
      roslaunch sr_robot_launch srhand_mujoco.launch
    ```

3. Run OpenPose ROS msg converter
    ```bash
      roslaunch openpose_mujoco openpose2dexpilot.launch
    ```

4. Run Direct Joints Mapping
    ```bash
      roslaunch openpose_mujoco hand_commander_Joints_vF.launch
    ```

### DexPilot

1. Run OpenPose

2. Run Shadow Hand MuJoCo simulator
    ```bash
      roslaunch sr_robot_launch srhand_mujoco.launch
    ```

3. Run OpenPose ROS msg converter
    ```bash
      roslaunch openpose_mujoco openpose2dexpilot.launch
    ```

4. Run DexPilot
    ```bash
      roslaunch openpose_mujoco hand_commander_DexPilot_v7.launch
    ```

### BioIK

1. Run OpenPose

2. Run Shadow Hand MuJoCo simulator
    ```bash
      roslaunch sr_robot_launch srhand_mujoco.launch
    ```

3. Run OpenPose ROS msg converter
    ```bash
      roslaunch human_robot_map openpose2dexpilot.launch
    ```

4. Run BioIK
    ```bash
      roslaunch human_robot_map bio_ik_v8.launch
    ```

### TeachNET

1. Run Kinect ROS Wrapper
    ```bash
      roslaunch kinect2_bridge kinect2_bridge.launch
    ```

2. Run TeachNET
    ```bash
      roslaunch teach_net_teleoperation teachnet_launch_unsafe.launch
    ```

### Wiring Fitting Task

1. Run UR5 + Shadow Hand MuJoCo simulator
    ```bash
      roslaunch sr_robot_launch sr_ur5_arm_hand_mujoco.launch
    ```

2. Run Clipping Task Script
    ```bash
      roslaunch shadow_pinch_teleop clipping_task.launch
    ```

### Automotive Painting Quality Control Task

1. Run UR5 + Shadow Hand MuJoCo simulator
    ```bash
      roslaunch sr_robot_launch sr_ur5_arm_hand_mujoco.launch
    ```

2. Run BioIK
    ```bash
      roslaunch human_robot_map bio_ik_v12.launch
    ```

3. Publish hand keypoints positions to replicate

## Demonstration

 - [Wiring Fitting](https://youtu.be/Tl3kcnhR5q0)
 - [Automotive Painting Quality Contro](https://youtu.be/lpxIPF_6WYc)


![GitHub](https://github.com/BrunoSantosCode/ShadowHand_MastersDissertation/assets/78873048/2d035f65-0977-4cc2-a0be-0c87cf1df7f3)

Bruno Martins Santos, 2023
