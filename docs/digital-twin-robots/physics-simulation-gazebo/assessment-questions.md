# Assessment Questions: Physics Simulation Concepts

This section provides assessment questions to test understanding of physics simulation concepts for digital twin humanoid robots.

## Multiple Choice Questions

1. What is the primary purpose of a physics engine in robotics simulation?
   a) To render realistic graphics
   b) To simulate the behavior of physical objects using mathematical models
   c) To control the robot's actuators
   d) To process sensor data

2. In a typical robotics simulation environment, how is Earth's gravity represented?
   a) As a positive value in the Z direction
   b) As a negative value in the Z direction
   c) As a positive value in the X direction
   d) As a negative value in the Y direction

3. Which of the following is NOT a common physics engine used in robotics simulation?
   a) ODE (Open Dynamics Engine)
   b) Bullet Physics
   c) PhysX
   d) TensorFlow

4. What does the acronym ERP stand for in physics simulation?
   a) Error Reduction Parameter
   b) Energy Recovery Protocol
   c) Extended Range Physics
   d) Elastic Response Property

5. Which collision detection phase quickly eliminates pairs of objects that are far apart?
   a) Narrow phase
   b) Broad phase
   c) Contact phase
   d) Integration phase

## Short Answer Questions

1. Explain the difference between discrete and continuous collision detection. When would you use each approach?

2. What is the significance of the center of mass (CoM) in humanoid robot stability? How does it affect balance control?

3. Describe the concept of restitution in collision physics. What does a restitution value of 0.0 versus 1.0 represent?

4. What are the main advantages and disadvantages of using mesh-based collision geometry versus primitive shapes?

5. Explain how gravity compensation works in robot control systems and why it's important for digital twin simulations.

## Problem-Solving Questions

1. A humanoid robot has a mass of 50kg and its center of mass is located 0.8m above the ground. If the robot's feet are positioned 0.4m apart (side to side), calculate the maximum horizontal force that can be applied at the center of mass before the robot tips over. Assume no sliding occurs.

2. You are designing a simulation for a humanoid robot walking at 0.5 m/s. The robot's control system updates at 100 Hz. What is the maximum time step you could use in your physics simulation to ensure at least 10 simulation steps occur between each control update? Why is this important?

3. A robot arm has a joint with a maximum torque of 50 Nm. The joint's velocity is limited to 2 rad/s, and its position range is -π/2 to π/2 radians. If simulating a task where the joint needs to move from -π/4 to π/4 radians against a constant load of 30 Nm, describe what could happen if the physics simulation parameters are not properly configured.

## Essay Questions

1. Discuss the trade-offs between simulation accuracy and computational performance in digital twin robotics. Provide specific examples of scenarios where you might prioritize accuracy over performance, and vice versa.

2. Explain the importance of validation in physics simulation for digital twin robots. Describe at least three different validation approaches and their respective advantages and limitations.

3. Describe how collision detection and response mechanisms impact the realism of a digital twin humanoid robot simulation. Include in your discussion the role of material properties, contact generation, and response models.

## Practical Application Questions

1. You are tasked with creating a physics simulation of a humanoid robot that needs to navigate through a room with furniture. Describe the steps you would take to set up the simulation environment, including how you would configure collision geometry for both the robot and the furniture.

2. A real humanoid robot is experiencing instability during walking in the laboratory. The engineering team wants to use the digital twin simulation to identify the cause. Describe how you would set up a simulation scenario to troubleshoot this issue, including what parameters you would vary and what metrics you would monitor.

## Answers to Multiple Choice Questions

1. b) To simulate the behavior of physical objects using mathematical models
2. b) As a negative value in the Z direction
3. d) TensorFlow
4. a) Error Reduction Parameter
5. b) Broad phase