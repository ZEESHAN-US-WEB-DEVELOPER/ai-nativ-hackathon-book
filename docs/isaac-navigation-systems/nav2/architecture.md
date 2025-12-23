# Nav2 Architecture and Components

Navigation2 (Nav2) is the ROS2 navigation stack that provides path planning, trajectory generation, and obstacle avoidance capabilities for mobile robots. It integrates with various perception systems to enable autonomous navigation.

## Core Architecture

Nav2 consists of several key components:

- **Navigation Server**: Main orchestrator that coordinates navigation tasks
- **Lifecycle Manager**: Manages the state of navigation components
- **Behavior Tree Engine**: Executes navigation behaviors using behavior trees
- **Planners Server**: Hosts global and local planners
- **Controller Server**: Manages trajectory following
- **Smoother Server**: Provides trajectory smoothing
- **Recovery Server**: Handles navigation recovery behaviors

## Navigation System Components

### Global Planner

- Computes a path from start to goal
- Considers static map information
- Uses algorithms like A*, Dijkstra, or NavFn

### Local Planner

- Follows the global path while avoiding obstacles
- Uses local sensor data
- Implements algorithms like DWA or Trajectory Rollout

### Costmap_2D

- Maintains obstacle information
- Combines static map, obstacles, and inflation
- Provides collision checking for planners

### Controller

- Converts planned path to velocity commands
- Implements trajectory following algorithms
- Maintains robot motion along the path

## Behavior Trees

Nav2 uses behavior trees to orchestrate navigation tasks:

- More flexible than traditional state machines
- Allows complex navigation behaviors
- Enables runtime reconfiguration
- Supports parallel execution of tasks