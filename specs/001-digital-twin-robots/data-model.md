# Data Model: Digital Twin of Humanoid Robots

## Core Entities

### Digital Twin Robot Model
- **Name**: Unique identifier for the robot model
- **Physical Properties**: Mass, dimensions, center of mass, friction coefficients
- **Joints**: Types, ranges of motion, actuator properties
- **Sensors**: Types, positions, specifications, noise characteristics
- **Materials**: Surface properties for rendering and physics

### Simulation Environment
- **Physical Properties**: Gravity, friction, air resistance
- **Obstacles**: Static and dynamic objects in the environment
- **Terrain**: Surface properties, textures, collision geometry
- **Lighting**: Light sources, shadows, reflections

### Sensor Data Streams
- **LiDAR Data**: Point cloud coordinates, intensity values, timestamp
- **Camera Data**: RGB image frames, depth information, field of view
- **IMU Data**: Acceleration (x, y, z), angular velocity (x, y, z), orientation quaternion
- **Joint States**: Position, velocity, effort for each joint

### Physics State
- **Position**: X, Y, Z coordinates in 3D space
- **Orientation**: Quaternion representing rotation
- **Velocity**: Linear and angular velocity vectors
- **Forces**: Applied forces and torques on each component

## Relationships

### Robot Model ↔ Simulation Environment
- A robot model exists within a simulation environment
- The environment affects the robot through physics properties and obstacles
- The robot interacts with the environment through collisions and forces

### Robot Model ↔ Sensor Data Streams
- A robot model has multiple sensor data streams
- Each sensor stream originates from a specific sensor placement on the robot
- Sensor data reflects the robot's state and environment perception

### Simulation Environment ↔ Physics State
- The simulation environment determines how physics states evolve
- Physics states are updated based on environment properties and interactions
- Multiple physics states can exist simultaneously for multiple robots

## State Transitions

### Simulation States
- **Configuration**: Robot model and environment are being set up
- **Initialization**: Physics engine is initializing, sensors are calibrated
- **Running**: Simulation is actively computing physics and sensor outputs
- **Paused**: Simulation is temporarily stopped, state is preserved
- **Stopped**: Simulation has ended, final state is available

### Robot States
- **Idle**: Robot is in a stable configuration with no active movement
- **Moving**: Robot is executing planned movements or responding to forces
- **Interacting**: Robot is in contact with environment or other objects
- **Sensing**: Robot is actively collecting sensor data from environment

## Validation Rules

### From Functional Requirements
- FR-001: Physics simulation must maintain realistic gravity and collision responses
- FR-002: Rendering must maintain visual fidelity and responsiveness
- FR-003: Sensor simulation must include realistic noise models
- FR-004: User configuration must validate against physical constraints
- FR-005: Real-time visualization must maintain consistent frame rates
- FR-006: Human-robot interaction must respond appropriately to forces
- FR-007: Performance measurement tools must provide accurate metrics
- FR-008: Environment definition must validate physical properties

## Data Flow

### Simulation Data Pipeline
1. User configures robot model and environment parameters
2. Physics engine initializes state based on configuration
3. Sensors begin collecting data from environment
4. Physics simulation updates state based on forces and constraints
5. Sensor data streams are processed and made available
6. Rendering engine updates visual representation
7. User can observe, analyze, and modify simulation parameters