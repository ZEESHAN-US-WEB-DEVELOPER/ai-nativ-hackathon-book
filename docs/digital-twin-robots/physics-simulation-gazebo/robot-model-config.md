# Example Robot Model Configuration

This document provides an example configuration for a humanoid robot model in Gazebo. This configuration demonstrates the key aspects of creating realistic digital twin robot models for physics simulation.

## Complete Humanoid Robot URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0" />
      <origin xyz="0 0 0.2" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0" />
      <geometry>
        <box size="0.3 0.3 0.4" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0" />
      <geometry>
        <box size="0.3 0.3 0.4" />
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link" />
    <child link="head" />
    <origin xyz="0 0 0.4" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0" />
  </joint>

  <link name="head">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.15" />
      </geometry>
      <material name="white">
        <color rgba="1 1 1 0.8" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.15" />
      </geometry>
    </collision>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link" />
    <child link="left_upper_arm" />
    <origin xyz="0.2 0 0.1" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>

  <link name="left_upper_arm">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 -0.15" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm" />
    <child link="left_lower_arm" />
    <origin xyz="0 0 -0.3" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <mass value="0.8" />
      <origin xyz="0 0 -0.1" />
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.2" radius="0.04" />
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.2" radius="0.04" />
      </geometry>
    </collision>
  </link>

  <!-- Right Arm -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link" />
    <child link="right_upper_arm" />
    <origin xyz="-0.2 0 0.1" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>

  <link name="right_upper_arm">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 -0.15" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
    </collision>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm" />
    <child link="right_lower_arm" />
    <origin xyz="0 0 -0.3" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
  </joint>

  <link name="right_lower_arm">
    <inertial>
      <mass value="0.8" />
      <origin xyz="0 0 -0.1" />
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.2" radius="0.04" />
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.2" radius="0.04" />
      </geometry>
    </collision>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link" />
    <child link="left_thigh" />
    <origin xyz="0.1 0 0" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0" />
  </joint>

  <link name="left_thigh">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.06" />
      </geometry>
      <material name="green">
        <color rgba="0 1 0 0.8" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.06" />
      </geometry>
    </collision>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh" />
    <child link="left_shin" />
    <origin xyz="0 0 -0.4" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0" />
  </joint>

  <link name="left_shin">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
      <material name="green">
        <color rgba="0 1 0 0.8" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
    </collision>
  </link>

  <!-- Right Leg -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link" />
    <child link="right_thigh" />
    <origin xyz="-0.1 0 0" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0" />
  </joint>

  <link name="right_thigh">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.06" />
      </geometry>
      <material name="green">
        <color rgba="0 1 0 0.8" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.06" />
      </geometry>
    </collision>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_thigh" />
    <child link="right_shin" />
    <origin xyz="0 0 -0.4" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0" />
  </joint>

  <link name="right_shin">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
      <material name="green">
        <color rgba="0 1 0 0.8" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
    </collision>
  </link>

  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="left_upper_arm">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="left_lower_arm">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_upper_arm">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_lower_arm">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="left_thigh">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="left_shin">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="right_thigh">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="right_shin">
    <material>Gazebo/Green</material>
  </gazebo>

</robot>
```

## Gazebo World Configuration Example

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Physics parameters -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Include our humanoid robot -->
    <include>
      <uri>model://simple_humanoid</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>

    <!-- Additional objects for interaction -->
    <model name="table">
      <pose>2 0 0 0 0 0</pose>
      <link name="link">
        <pose>0 0 0.5 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.5 0.2 1</ambient>
            <diffuse>0.8 0.5 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

  </world>
</sdf>
```

## Model Installation Instructions

To use this robot model in Gazebo:

1. Create a directory for your robot model:
   ```
   ~/.gazebo/models/simple_humanoid
   ```

2. Create the following structure:
   ```
   simple_humanoid/
   ├── model.config
   └── meshes/
       └── (any mesh files if using custom geometry)
   └── model.sdf (or model.urdf)
   ```

3. The `model.config` file should contain:
   ```xml
   <?xml version="1.0"?>
   <model>
     <name>simple_humanoid</name>
     <version>1.0</version>
     <sdf version='1.7'>model.sdf</sdf>
     <author>
       <name>Your Name</name>
       <email>your.email@example.com</email>
     </author>
     <description>A simple humanoid robot model for digital twin simulation.</description>
   </model>
   ```

4. Place the URDF content in a `model.urdf` file in the same directory.

5. Launch Gazebo and your model will be available in the Insert tab.