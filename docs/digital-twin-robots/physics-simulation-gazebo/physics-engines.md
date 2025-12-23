# Physics Engines in Robotics Simulation

Physics engines are computational systems that simulate the behavior of physical objects using mathematical models. In robotics simulation, they are critical for creating realistic digital twins that accurately mirror real-world physics.

## Overview of Physics Engines

Physics engines in robotics simulation handle:
- Collision detection and response
- Rigid body dynamics
- Joint constraints
- Contact forces
- Friction and damping

## Common Physics Engines in Robotics

### ODE (Open Dynamics Engine)
- One of the oldest and most widely used physics engines
- Good for rigid body simulation
- Used in early versions of Gazebo

### Bullet Physics
- Modern physics engine with good performance
- Supports both rigid and soft body dynamics
- Used in Gazebo and other simulation environments

### PhysX
- NVIDIA's physics engine
- Optimized for GPU acceleration
- Used in Unity and other rendering engines

### DART (Dynamic Animation and Robotics Toolkit)
- Modern physics engine designed for robotics
- Supports complex kinematic trees
- Good for humanoid robot simulation

## Physics Engine Parameters

### Time Step
- Defines the granularity of the simulation
- Smaller time steps provide more accurate simulation but require more computation
- Typically ranges from 0.001s to 0.01s

### Solver Iterations
- Determines how many iterations the physics solver runs per time step
- More iterations provide more stable simulation
- Trade-off between stability and performance

### ERP (Error Reduction Parameter)
- Controls how quickly constraint errors are corrected
- Higher values correct errors faster but can cause instability

### CFM (Constraint Force Mixing)
- Adds regularization to constraint matrices
- Helps with numerical stability