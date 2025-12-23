# Path Planning with Nav2

Path planning is a core capability of Nav2, enabling robots to compute safe and efficient paths from their current location to a goal. This section covers the path planning algorithms and configuration in Nav2.

## Global Path Planning

Global planners compute a path from start to goal considering the static map:

### A* (A-star) Planner

- Optimal path planning algorithm
- Guarantees shortest path in terms of cost
- Efficient for most navigation scenarios

### Dijkstra's Algorithm

- Variant of A* without heuristic
- Guarantees optimal path
- More computationally intensive than A*

### NavFn Planner

- Fast navigation function
- Uses wavefront propagation
- Good for large maps

## Local Path Planning

Local planners adjust the global path to avoid dynamic obstacles:

### Dynamic Window Approach (DWA)

- Considers robot kinodynamics
- Optimizes for velocity and acceleration limits
- Good for real-time obstacle avoidance

### Trajectory Rollout

- Evaluates multiple potential trajectories
- Selects best trajectory based on criteria
- Can handle complex robot constraints

## Configuration Considerations

### Planner Selection

Choose planners based on:

- Environment complexity
- Robot kinematics
- Real-time requirements
- Accuracy needs

### Parameter Tuning

Key parameters for path planning:

- Planner frequency
- Goal tolerance
- Compute path to goal tolerance
- Planning window size
- Velocity limits

## Integration with Isaac Tools

Nav2 integrates with Isaac tools by:

- Using Isaac Sim for testing path planning algorithms
- Leveraging Isaac ROS perception for obstacle detection
- Validating navigation performance in simulation
- Ensuring simulation-to-reality transfer