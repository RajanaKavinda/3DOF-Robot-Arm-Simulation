# Voice Controlled 3DOF Robot Arm Simulation 🤖🎙️

## Overview

This project is a 3 Degrees of Freedom (DOF) robot arm manipulator integrated with Amazon Alexa for voice-controlled motion. The goal was to bridge theoretical knowledge in robotics, kinematics, and AI-driven control interfaces with practical hands-on experience.

## Key Features

### 3DOF Articulated Manipulator
- **Base**, **Shoulder**, **Elbow**, and **Gripper** with parallel linkage mechanism for stability and efficient movement.

### ROS2 Simulation & Visualization
- **URDF (Unified Robot Description Format)** for robot modeling.
- **RViz** for visualization.
- **Gazebo Ignition** for realistic physics simulation.

### Robot Links
- Base Link
- Base Plate
- Forward Drive Arm
- Horizontal Arm
- Claw Support
- Gripper Right & Left

### Camera Integration
- Mounted on the Base Link for a robot-eye perspective.

### Gazebo ROS2 Control Interface
- Utilized `gazebo_ros2_control` for managing robot movements.

### MoveIt2 for Motion Planning
- **Forward Kinematics (FK)** – Computing end-effector position from joint angles.
- **Inverse Kinematics (IK)** – Finding joint angles for a given end-effector position.
- **MoveIt2 API** – Enabling external applications to control robotic movements.

### ROS2 Communication Framework
- **Publishers & Subscribers** – For real-time sensor-actuator interaction.
- **Actions & Services** – For executing predefined robot tasks.

### Voice-Controlled Robot with Alexa
- Integrated Amazon Alexa Developer Console (ASK SDK) for voice commands.
- **Workflow**:
  1. User gives a voice command → Alexa Skill processes it → Ngrok Web Server forwards it → ROS2 task_server executes the motion.
- Flask-based web service using **Ngrok** acts as a bridge between Alexa and ROS2, allowing seamless cloud-to-robot control.

## Final Outcome & Learnings

- Successfully controlled the robot arm with voice commands!
- Gained experience in robot kinematics, ROS2, AI voice interfaces, and real-world robotics applications.
- Explored ROS2-based control architectures, robot perception, and MoveIt2 motion planning.

## Demo Video 🎥  
Watch the live demonstration of the voice-controlled 3DOF robot arm in action:  
🔗 [LinkedIn Demo Video](https://www.linkedin.com/posts/rajana-kavinda_robotics-ros2-moveit2-activity-7296642820826705920-olkz?utm_source=share&utm_medium=member_desktop&rcm=ACoAADM93PkBbwmiL9zO8WEKQftM0nxN6BMWjco)

## Requirements

- **ROS2** (Humble)
- **Gazebo Sim** & **RViz**
- **MoveIt2**
- **Amazon Alexa SDK** (ASK SDK)
- **Flask** & **Ngrok** for web service bridge


