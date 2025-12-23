# Costmap Configuration in Nav2

Costmaps in Nav2 are critical for navigation, representing the environment with information about obstacles, clear space, and areas of inflation. Proper configuration is essential for safe and effective navigation.

## Costmap Layers

Nav2 uses layered costmaps that combine multiple sources of information:

### Static Layer

- Represents static obstacles from the map
- Loaded from a pre-built occupancy grid
- Updated only when map changes

### Obstacle Layer

- Processes sensor data to detect obstacles
- Updates in real-time as robot moves
- Combines data from multiple sensors

### Inflation Layer

- Creates safety margins around obstacles
- Prevents robot from getting too close to obstacles
- Configured based on robot size and safety requirements

## Configuration Parameters

### Resolution

- Defines the size of each cell in meters
- Affects navigation precision and computational cost
- Typically between 0.01m and 0.1m

### Update Frequency

- How often the costmap is updated
- Affects responsiveness to dynamic obstacles
- Must balance between reactivity and computational load

### Robot Footprint

- Defines the physical dimensions of the robot
- Critical for proper collision checking
- Can be circular, polygonal, or line-based

### Inflation Settings

- Defines how far obstacles are inflated
- Includes inscribed and circumscribed radius
- Affects path planning and safety

## Multiple Costmap Configuration

Nav2 typically uses two costmaps:

### Global Costmap

- Used for global path planning
- Covers a larger area
- Updated less frequently
- Focuses on static obstacles

### Local Costmap

- Used for local path planning and obstacle avoidance
- Covers a smaller area around the robot
- Updated more frequently
- Includes dynamic obstacles