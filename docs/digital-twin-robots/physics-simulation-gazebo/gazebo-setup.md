# Gazebo Setup and Configuration

This section provides instructions for setting up Gazebo for physics simulation of digital twin humanoid robots. Proper configuration is essential for achieving realistic simulation results.

## Installing Gazebo

### System Requirements
- Ubuntu 18.04, 20.04, or 22.04 (recommended)
- At least 8GB RAM (16GB+ recommended for complex simulations)
- GPU with OpenGL 3.3+ support
- 10GB+ free disk space

### Installation Methods
1. **APT Package Manager** (recommended for beginners):
   ```bash
   sudo apt install gazebo libgazebo-dev
   ```

2. **Docker Container** (recommended for consistent environments):
   ```bash
   docker pull gazebo:gz-latest
   ```

3. **Build from Source** (for development and customization):
   - Follow instructions at http://gazebosim.org/tutorials?tut=install_from_source

## Basic Configuration

### World File Structure
Gazebo simulations are defined in SDF (Simulation Description Format) files:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="digital_twin_world">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

### Physics Engine Configuration
- **Time Step**: Set to 0.001s for stable humanoid robot simulation
- **Real-time Factor**: 1.0 for real-time simulation, higher for faster-than-real-time
- **Update Rate**: Match to desired simulation frequency (typically 1000 Hz for control)

## Robot Model Configuration

### URDF to SDF Conversion
Gazebo can directly load URDF files, but SDF provides more features:

```xml
<model name="humanoid_robot">
  <static>false</static>
  <link name="base_link">
    <inertial>
      <mass>10.0</mass>
      <inertia>
        <ixx>0.1</ixx>
        <ixy>0.0</ixy>
        <ixz>0.0</ixz>
        <iyy>0.1</iyy>
        <iyz>0.0</iyz>
        <izz>0.1</izz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry>
        <box><size>0.5 0.5 0.5</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>0.5 0.5 0.5</size></box>
      </geometry>
    </visual>
  </link>
</model>
```

### Joint Configuration
Properly configure joints with realistic limits:

```xml
<joint name="hip_joint" type="revolute">
  <parent>base_link</parent>
  <child>thigh_link</child>
  <axis>
    <xyz>0 0 1</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
      <effort>100.0</effort>
      <velocity>3.0</velocity>
    </limit>
  </axis>
</joint>
```

## Plugin Integration

### ROS Integration
Use the libgazebo_ros_pkgs for ROS integration:

```xml
<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
  <robotNamespace>/humanoid_robot</robotNamespace>
</plugin>
```

### Sensor Plugins
Add sensor plugins to robot models:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <noise>
      <type>gaussian</type>
      <angular_velocity>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </angular_velocity>
    </noise>
  </imu>
</sensor>
```

## Performance Optimization

### Graphics Settings
- Reduce visual quality for headless simulations
- Use OGRE's multi-rendering context for multiple sensors
- Adjust shadow quality and anti-aliasing based on needs

### Physics Settings
- Use appropriate solver (ODE, Bullet) based on simulation requirements
- Adjust CFM and ERP values for stability
- Use thread-safe models when running multiple simulations