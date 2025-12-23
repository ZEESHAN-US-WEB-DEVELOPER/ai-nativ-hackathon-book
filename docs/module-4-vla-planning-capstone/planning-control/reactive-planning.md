# Reactive Planning

Reactive planning enables robots to respond in real-time to dynamic environments, moving obstacles, and changing goals. Unlike deliberative planning (which computes complete plans offline), reactive planning continuously adjusts actions based on current sensor observations.

## Overview

**Deliberative Planning**: Plan once, execute
- Example: RRT computes path, robot follows it
- Fails if environment changes during execution

**Reactive Planning**: Sense-plan-act loop
- Continuously update plan based on observations
- Responds to dynamic obstacles, unexpected events

## Dynamic Window Approach (DWA)

DWA selects velocity commands by simulating trajectories in velocity space and choosing the best according to a cost function.

### Algorithm

```python
def dynamic_window_approach(current_velocity, goal, obstacles):
    # 1. Compute dynamic window (feasible velocities)
    v_min, v_max = compute_velocity_limits(current_velocity, dt, a_max)
    ω_min, ω_max = compute_angular_velocity_limits(current_velocity, dt, α_max)

    # 2. Sample velocities in dynamic window
    velocities = []
    for v in np.linspace(v_min, v_max, num_v_samples):
        for ω in np.linspace(ω_min, ω_max, num_ω_samples):
            velocities.append((v, ω))

    # 3. Simulate trajectories and evaluate
    best_velocity = None
    best_score = -float('inf')

    for (v, ω) in velocities:
        trajectory = simulate_trajectory(v, ω, sim_time=2.0)

        # Check collision
        if collides(trajectory, obstacles):
            continue

        # Compute cost
        score = (α * heading_score(trajectory, goal) +
                 β * velocity_score(v) +
                 γ * clearance_score(trajectory, obstacles))

        if score > best_score:
            best_score = score
            best_velocity = (v, ω)

    return best_velocity
```

**Cost Components**:
1. **Heading**: Alignment with goal direction
   ```python
   def heading_score(trajectory, goal):
       final_heading = trajectory[-1].theta
       goal_heading = atan2(goal.y - trajectory[-1].y,
                           goal.x - trajectory[-1].x)
       return 1 - abs(angle_diff(final_heading, goal_heading)) / π
   ```

2. **Velocity**: Prefer higher speeds
   ```python
   def velocity_score(v):
       return v / v_max
   ```

3. **Clearance**: Distance to obstacles
   ```python
   def clearance_score(trajectory, obstacles):
       min_distance = min(distance(point, obstacles) for point in trajectory)
       return min_distance
   ```

**Properties**:
- Real-time: 10-30 Hz update rate
- Locally optimal (greedy)
- Collision-free within dynamic window

## Potential Fields

Treat goal as attractive force, obstacles as repulsive forces.

### Attractive Potential

```python
def attractive_potential(position, goal):
    distance = np.linalg.norm(goal - position)
    return 0.5 * k_att * distance**2

def attractive_force(position, goal):
    return -k_att * (position - goal)
```

### Repulsive Potential

```python
def repulsive_potential(position, obstacle, influence_radius):
    distance = np.linalg.norm(obstacle - position)

    if distance > influence_radius:
        return 0
    else:
        return 0.5 * k_rep * (1/distance - 1/influence_radius)**2

def repulsive_force(position, obstacle, influence_radius):
    distance = np.linalg.norm(obstacle - position)

    if distance > influence_radius:
        return np.zeros(2)
    else:
        direction = (position - obstacle) / distance
        return k_rep * (1/distance - 1/influence_radius) * (1/distance**2) * direction
```

### Total Force

```python
def compute_control(position, goal, obstacles):
    # Attractive force toward goal
    f_att = attractive_force(position, goal)

    # Repulsive forces from obstacles
    f_rep = np.zeros(2)
    for obstacle in obstacles:
        f_rep += repulsive_force(position, obstacle, influence_radius=2.0)

    # Total force
    f_total = f_att + f_rep

    # Convert to velocity command
    velocity = f_total / np.linalg.norm(f_total) * v_max

    return velocity
```

**Advantages**:
- Simple and intuitive
- Real-time computation
- Smooth trajectories

**Limitations**:
- Local minima (can get stuck)
- No global optimality guarantees
- Oscillations in narrow passages

## Model Predictive Control (MPC)

MPC solves an optimization problem at each time step over a finite horizon.

### Formulation

```
minimize    Σ_{t=0}^{N} cost(x_t, u_t)
subject to  x_{t+1} = f(x_t, u_t)      # dynamics
            x_t ∈ X_free                # collision-free
            u_t ∈ U                     # control limits
```

### Algorithm

```python
def mpc_control(current_state, goal, obstacles, horizon=10):
    # Initialize optimization variables
    x = [current_state]  # States over horizon
    u = []               # Controls over horizon

    # Setup optimization problem
    problem = OptimizationProblem()

    for t in range(horizon):
        # Add decision variables
        u_t = problem.add_variable(bounds=control_limits)
        u.append(u_t)

        # Dynamics constraint
        x_next = dynamics(x[t], u_t)
        x.append(x_next)

        # Collision constraint
        problem.add_constraint(collision_free(x_next, obstacles))

        # Add cost
        problem.add_cost(stage_cost(x_next, u_t, goal))

    # Terminal cost
    problem.add_cost(terminal_cost(x[-1], goal))

    # Solve optimization
    solution = problem.solve()

    # Return first control (receding horizon)
    return solution.u[0]
```

**Stage Cost**:
```python
def stage_cost(state, control, goal):
    # Tracking error
    c_tracking = ||state - goal||²

    # Control effort
    c_control = ||control||²

    return w1 * c_tracking + w2 * c_control
```

**Advantages**:
- Optimal over horizon
- Handles constraints explicitly
- Predictive (anticipates future)

**Disadvantages**:
- Computationally expensive
- Requires fast solver (IPOPT, OSQP)
- Model accuracy critical

## Behavior Trees

Modular, hierarchical reactive control structure.

### Structure

```
Root
├── Sequence
│   ├── Navigate to object
│   ├── Grasp object
│   └── Navigate to goal
└── Fallback (if sequence fails)
    └── Call for human help
```

### Node Types

**Action**: Executable behavior
```python
class ActionNode:
    def tick(self):
        result = self.execute()
        return SUCCESS if result else FAILURE
```

**Condition**: Check predicate
```python
class ConditionNode:
    def tick(self):
        return SUCCESS if self.condition() else FAILURE
```

**Sequence**: Execute children in order (AND)
```python
class SequenceNode:
    def tick(self):
        for child in self.children:
            result = child.tick()
            if result != SUCCESS:
                return result
        return SUCCESS
```

**Fallback**: Try children until one succeeds (OR)
```python
class FallbackNode:
    def tick(self):
        for child in self.children:
            result = child.tick()
            if result == SUCCESS:
                return SUCCESS
        return FAILURE
```

### Example: Fetch Object

```python
# Root
root = FallbackNode()

# Primary sequence
primary = SequenceNode()
primary.add_child(NavigateToObject())
primary.add_child(GraspObject())
primary.add_child(NavigateToGoal())
primary.add_child(ReleaseObject())

# Fallback: retry with alternative grasp
retry = SequenceNode()
retry.add_child(ReplanGrasp())
retry.add_child(GraspObject())

# Final fallback: ask for help
ask_help = AskForHelp()

root.add_child(primary)
root.add_child(retry)
root.add_child(ask_help)

# Execute behavior tree
while True:
    result = root.tick()
    if result == SUCCESS:
        break
```

**Advantages**:
- Modular and reusable
- Easy to understand and debug
- Reactive to changes

**Tools**: BehaviorTree.CPP, FlexBE (ROS)

## Reactive Navigation in ROS

### Nav2 with DWA Controller

```python
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

def reactive_navigation():
    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2 to be ready
    navigator.waitUntilNav2Active()

    # Set goal
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = 5.0
    goal_pose.pose.position.y = 5.0
    navigator.goToPose(goal_pose)

    # Monitor progress (reactive updates)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        print(f"Distance remaining: {feedback.distance_remaining}")

        # Navigator automatically replans if obstacles detected

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal reached!")
```

## Hybrid Deliberative-Reactive Architecture

Combine global planning with local reactive control:

```python
def hybrid_planning(start, goal, obstacles):
    # 1. Global planner: Compute high-level path
    global_path = astar(start, goal, static_obstacles)

    # 2. Follow path with reactive local planner
    current_pos = start
    for waypoint in global_path:
        while not reached(current_pos, waypoint):
            # Reactive planner responds to dynamic obstacles
            velocity = dwa(current_pos, waypoint, dynamic_obstacles)
            current_pos = move(current_pos, velocity, dt)

            # Replan globally if path blocked
            if path_blocked(global_path, dynamic_obstacles):
                global_path = astar(current_pos, goal, obstacles)
                break

    return global_path
```

**Benefits**:
- Global optimality (from deliberative planner)
- Real-time reactivity (from reactive planner)

## Comparison of Reactive Methods

| Method | Speed | Optimality | Completeness | Use Case |
|--------|-------|------------|--------------|----------|
| DWA | Fast (10-30 Hz) | Local | No | Mobile robot navigation |
| Potential Fields | Very fast | None | No | Simple environments |
| MPC | Slow (1-10 Hz) | Local optimal | No | Constrained systems |
| Behavior Trees | Fast | N/A | Depends | Task-level control |

## Summary

Reactive planning enables robots to adapt to dynamic environments in real-time. DWA and potential fields provide fast local control, MPC offers optimal control over a horizon, and behavior trees provide structured reactive behavior. Combining global planning with reactive local control creates robust navigation systems.

Next: [Manipulation Planning](manipulation-planning.md) - Learn planning for grasping and object manipulation.
