# Path Planning Algorithms

Path planning computes collision-free paths for robots to move from start to goal configurations. This section covers fundamental algorithms: graph search methods (A*, Dijkstra) and sampling-based planners (RRT, RRT*).

## Problem Formulation

**Given**:
- Start configuration q_start
- Goal configuration q_goal
- Configuration space C (robot's possible configurations)
- Obstacle space C_obs (configurations in collision)

**Find**: Path from q_start to q_goal in C_free = C \ C_obs

## Graph-Based Planning

### Dijkstra's Algorithm

Finds shortest path in weighted graphs by exploring nodes in order of distance from start.

**Algorithm**:
```python
def dijkstra(graph, start, goal):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while not open_set.empty():
        current_cost, current = open_set.get()

        if current == goal:
            return reconstruct_path(came_from, goal)

        for neighbor in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, neighbor)

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost
                open_set.put((priority, neighbor))
                came_from[neighbor] = current

    return None  # No path found
```

**Complexity**: O((V + E) log V) where V = vertices, E = edges

**Pros**: Guarantees optimal path

**Cons**: Explores uniformly in all directions, slow without heuristic

### A* Algorithm

Extends Dijkstra with heuristic function h(n) that estimates cost to goal.

**Cost Function**: f(n) = g(n) + h(n)
- g(n): Cost from start to n
- h(n): Estimated cost from n to goal

**Algorithm**:
```python
def astar(graph, start, goal, heuristic):
    open_set = PriorityQueue()
    open_set.put((heuristic(start, goal), start))
    came_from = {}
    g_score = {start: 0}

    while not open_set.empty():
        _, current = open_set.get()

        if current == goal:
            return reconstruct_path(came_from, goal)

        for neighbor in graph.neighbors(current):
            tentative_g = g_score[current] + graph.cost(current, neighbor)

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                open_set.put((f_score, neighbor))

    return None
```

**Heuristics for Robotics**:

1. **Euclidean Distance** (C-space):
   ```python
   h(q, q_goal) = ||q - q_goal||_2
   ```

2. **Workspace Distance** (Cartesian space):
   ```python
   h(q, q_goal) = ||forward_kinematics(q) - forward_kinematics(q_goal)||_2
   ```

3. **Manhattan Distance** (grid worlds):
   ```python
   h(q, q_goal) = sum(|q[i] - q_goal[i]| for i in range(n))
   ```

**Admissibility**: h(n) ≤ actual cost from n to goal → A* finds optimal path

**Consistency**: h(n) ≤ cost(n, n') + h(n') → A* is efficient

## Sampling-Based Planning

For high-dimensional robots (e.g., 7-DOF arms), discretizing C-space is intractable. Sampling-based methods explore C-space by random sampling.

### RRT (Rapidly-Exploring Random Tree)

Grows tree from start by sampling random configurations and extending toward them.

**Algorithm**:
```python
def rrt(start, goal, obstacles, max_iter=10000):
    tree = Tree(start)

    for i in range(max_iter):
        # Sample random configuration
        q_rand = sample_random_config()

        # Find nearest node in tree
        q_near = tree.nearest(q_rand)

        # Extend toward q_rand
        q_new = steer(q_near, q_rand, step_size=0.1)

        # Check collision
        if not in_collision(q_new, obstacles):
            tree.add_node(q_new, parent=q_near)

            # Check if close to goal
            if distance(q_new, goal) < threshold:
                return tree.extract_path(start, q_new)

    return None  # No path found
```

**Key Functions**:

**Steer**: Extend from q_near toward q_rand by step_size
```python
def steer(q_near, q_rand, step_size):
    direction = (q_rand - q_near) / ||q_rand - q_near||
    q_new = q_near + step_size * direction
    return q_new
```

**Collision Checking**:
```python
def in_collision(q, obstacles):
    robot_geometry = forward_kinematics(q)
    for obstacle in obstacles:
        if intersects(robot_geometry, obstacle):
            return True
    return False
```

**Properties**:
- Probabilistically complete (finds path if exists, given infinite time)
- Not optimal (path quality depends on sampling)
- Fast for high-dimensional spaces

### RRT* (Optimal RRT)

Extends RRT with rewiring step to improve path quality.

**Algorithm**:
```python
def rrt_star(start, goal, obstacles, max_iter=10000):
    tree = Tree(start)

    for i in range(max_iter):
        q_rand = sample_random_config()
        q_near = tree.nearest(q_rand)
        q_new = steer(q_near, q_rand, step_size=0.1)

        if not in_collision(q_new, obstacles):
            # Find neighbors within radius
            neighbors = tree.near(q_new, radius=1.0)

            # Choose best parent (minimum cost)
            q_min = min(neighbors, key=lambda q: tree.cost(start, q) + distance(q, q_new))
            tree.add_node(q_new, parent=q_min)

            # Rewire tree
            for q_neighbor in neighbors:
                new_cost = tree.cost(start, q_new) + distance(q_new, q_neighbor)
                if new_cost < tree.cost(start, q_neighbor):
                    tree.change_parent(q_neighbor, q_new)

        if distance(q_new, goal) < threshold:
            return tree.extract_path(start, q_new)

    return None
```

**Improvements Over RRT**:
- Asymptotically optimal (converges to optimal path)
- Rewiring reduces path cost over time
- Better path quality for same number of samples

### RRT-Connect

Bidirectional RRT that grows trees from both start and goal.

**Algorithm**:
```python
def rrt_connect(start, goal, obstacles, max_iter=10000):
    tree_start = Tree(start)
    tree_goal = Tree(goal)

    for i in range(max_iter):
        # Grow tree_start
        q_rand = sample_random_config()
        q_new_start = extend_tree(tree_start, q_rand, obstacles)

        if q_new_start:
            # Try to connect from tree_goal
            q_new_goal = connect_tree(tree_goal, q_new_start, obstacles)

            if q_new_goal == q_new_start:
                # Trees connected!
                return merge_paths(tree_start, tree_goal, q_new_start)

        # Swap trees
        tree_start, tree_goal = tree_goal, tree_start

    return None
```

**Benefits**:
- Faster convergence (explores from both ends)
- Good for narrow passages
- Used in MoveIt motion planning framework

## Advanced Sampling Strategies

### Informed RRT*

Uses heuristic to sample only in regions that could improve current best path.

```python
def informed_sampling(start, goal, c_best):
    """Sample in ellipse with foci at start and goal"""
    c_min = distance(start, goal)
    if c_best < float('inf'):
        # Sample in ellipse where f1 + f2 < c_best
        return sample_ellipse(start, goal, c_best)
    else:
        # No solution yet, sample uniformly
        return sample_uniform()
```

**Speedup**: 2-10x faster than RRT* for many problems

### Goal Biasing

Sample goal configuration with probability p_goal (typically 5-10%).

```python
def sample_config(goal, p_goal=0.05):
    if random.random() < p_goal:
        return goal
    else:
        return sample_uniform()
```

**Effect**: Faster convergence to goal region

## Practical Implementation

### Configuration Space for 7-DOF Arm

```python
class ArmConfigSpace:
    def __init__(self, joint_limits):
        self.joint_limits = joint_limits  # [(min, max) for each joint]
        self.dim = len(joint_limits)

    def sample_uniform(self):
        config = []
        for (min_val, max_val) in self.joint_limits:
            config.append(random.uniform(min_val, max_val))
        return np.array(config)

    def distance(self, q1, q2):
        return np.linalg.norm(q1 - q2)

    def interpolate(self, q1, q2, t):
        """Linear interpolation between q1 and q2"""
        return (1 - t) * q1 + t * q2

    def in_bounds(self, q):
        for i, (min_val, max_val) in enumerate(self.joint_limits):
            if not (min_val <= q[i] <= max_val):
                return False
        return True
```

### Collision Checking with PyBullet

```python
import pybullet as p

def check_collision(robot_id, q, obstacles):
    """Check if configuration q is in collision"""
    # Set robot to configuration q
    for i, joint_angle in enumerate(q):
        p.resetJointState(robot_id, i, joint_angle)

    # Check for collisions
    contact_points = p.getContactPoints(robot_id)

    return len(contact_points) > 0
```

## Comparison of Algorithms

| Algorithm | Optimality | Completeness | Speed | Best For |
|-----------|------------|--------------|-------|----------|
| Dijkstra | Optimal | Complete | Slow | Small graphs |
| A* | Optimal* | Complete | Fast | Known graphs, good heuristic |
| RRT | Not optimal | Prob. complete | Fast | High-D, single query |
| RRT* | Asymp. optimal | Prob. complete | Medium | High-D, path quality matters |
| RRT-Connect | Not optimal | Prob. complete | Very fast | High-D, fast solutions |

*A* is optimal if heuristic is admissible

## Best Practices

1. **Choose the right algorithm**:
   - Low-D (2D, 3D) → A*
   - High-D (6+ DOF) → RRT/RRT*/RRT-Connect

2. **Tune parameters**:
   - Step size: Balance exploration vs resolution
   - Goal threshold: Too small → slow, too large → poor solutions
   - Max iterations: Depends on problem difficulty

3. **Improve collision checking**:
   - Use bounding volumes (OBB, AABB) for fast rejection
   - Hierarchical collision checking
   - Cache collision checks

4. **Post-process paths**:
   - Smooth paths (remove unnecessary waypoints)
   - Shortcut paths (connect distant waypoints if collision-free)

## Summary

Path planning finds collision-free paths in configuration space. Graph search (A*) works for low-dimensional spaces, while sampling-based planners (RRT, RRT*) scale to high-dimensional robots. Understanding these algorithms is essential for implementing motion planning in humanoid robotics systems.

Next: [Trajectory Optimization](trajectory-optimization.md) - Learn to generate smooth, dynamically feasible trajectories.
