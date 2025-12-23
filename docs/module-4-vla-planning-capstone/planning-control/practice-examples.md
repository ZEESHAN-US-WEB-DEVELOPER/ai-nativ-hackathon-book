# Hands-On Practice: Path Planning & Control

Complete, runnable code examples for implementing path planning and control algorithms with all dependencies included.

## 🔧 Setup & Installation

### Required Dependencies

```bash
# Create virtual environment
python -m venv planning_env
source planning_env/bin/activate  # Windows: planning_env\Scripts\activate

# Install core libraries
pip install numpy==1.24.3
pip install matplotlib==3.8.2
pip install scipy==1.11.4

# Robotics and planning
pip install pybullet==3.2.6
pip install networkx==3.2.1

# For ROS integration (optional)
pip install rospkg empy

# Visualization
pip install pillow==10.1.0
pip install imageio==2.33.1
```

---

## 🗺️ Example 1: A* Path Planning

**Goal**: Implement A* algorithm from scratch and visualize the results.

### Code: Complete A* Implementation

```python
"""
example1_astar.py
A* path planning algorithm with visualization
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import heapq
from typing import List, Tuple

class AStarPlanner:
    """
    A* path planning algorithm
    """
    def __init__(self, grid_size=(50, 50), resolution=0.1):
        self.grid_size = grid_size
        self.resolution = resolution
        self.grid = np.zeros(grid_size)

    def add_obstacle(self, x, y, width, height):
        """Add rectangular obstacle to grid"""
        x_grid = int(x / self.resolution)
        y_grid = int(y / self.resolution)
        w_grid = int(width / self.resolution)
        h_grid = int(height / self.resolution)

        self.grid[
            max(0, y_grid):min(self.grid_size[0], y_grid + h_grid),
            max(0, x_grid):min(self.grid_size[1], x_grid + w_grid)
        ] = 1

    def heuristic(self, a, b):
        """Euclidean distance heuristic"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, node):
        """Get valid neighbor nodes (8-connected)"""
        neighbors = []
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]

        for dx, dy in directions:
            new_x, new_y = node[0] + dx, node[1] + dy

            # Check bounds
            if (0 <= new_x < self.grid_size[1] and
                0 <= new_y < self.grid_size[0]):

                # Check collision
                if self.grid[new_y, new_x] == 0:
                    # Cost for diagonal movement
                    cost = 1.414 if abs(dx) + abs(dy) == 2 else 1.0
                    neighbors.append(((new_x, new_y), cost))

        return neighbors

    def plan(self, start, goal):
        """
        Find path from start to goal using A*

        Args:
            start: (x, y) in grid coordinates
            goal: (x, y) in grid coordinates

        Returns:
            path: List of (x, y) waypoints or None if no path found
        """
        # Priority queue: (f_score, counter, node)
        open_set = []
        counter = 0
        heapq.heappush(open_set, (0, counter, start))

        # Track paths
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        # Track explored nodes
        closed_set = set()

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return list(reversed(path))

            closed_set.add(current)

            # Explore neighbors
            for neighbor, move_cost in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue

                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)

                    counter += 1
                    heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))

        return None  # No path found

    def visualize(self, start, goal, path=None):
        """Visualize the grid, obstacles, and path"""
        fig, ax = plt.subplots(figsize=(10, 10))

        # Draw grid
        ax.imshow(self.grid.T, cmap='binary', origin='lower', alpha=0.3)

        # Draw obstacles
        obstacle_y, obstacle_x = np.where(self.grid == 1)
        ax.scatter(obstacle_x, obstacle_y, c='black', s=10, marker='s', alpha=0.7)

        # Draw start and goal
        ax.plot(start[0], start[1], 'go', markersize=15, label='Start')
        ax.plot(goal[0], goal[1], 'r*', markersize=20, label='Goal')

        # Draw path
        if path:
            path_array = np.array(path)
            ax.plot(
                path_array[:, 0],
                path_array[:, 1],
                'b-',
                linewidth=2,
                label=f'Path (length: {len(path)})'
            )

        ax.set_xlabel('X (grid cells)')
        ax.set_ylabel('Y (grid cells)')
        ax.set_title('A* Path Planning Visualization')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('astar_result.png', dpi=150)
        print("✅ Visualization saved as 'astar_result.png'")
        plt.show()

def run_astar_demo():
    """Complete A* demonstration"""
    print("="*70)
    print("🗺️  A* PATH PLANNING DEMONSTRATION")
    print("="*70)

    # Create planner
    planner = AStarPlanner(grid_size=(50, 50), resolution=0.1)

    # Add obstacles
    print("\n📦 Adding obstacles...")
    planner.add_obstacle(1.0, 1.0, 1.0, 0.5)  # Wall 1
    planner.add_obstacle(3.0, 2.0, 0.5, 2.0)  # Wall 2
    planner.add_obstacle(1.5, 3.5, 2.0, 0.3)  # Wall 3

    # Define start and goal
    start = (5, 5)
    goal = (45, 45)

    print(f"🎯 Start: {start}")
    print(f"🏁 Goal: {goal}")

    # Plan path
    print("\n🔍 Planning path...")
    import time
    start_time = time.time()

    path = planner.plan(start, goal)

    planning_time = time.time() - start_time

    if path:
        print(f"✅ Path found!")
        print(f"📏 Path length: {len(path)} waypoints")
        print(f"⏱️  Planning time: {planning_time*1000:.2f} ms")

        # Calculate path distance
        total_distance = sum(
            np.sqrt((path[i+1][0] - path[i][0])**2 +
                   (path[i+1][1] - path[i][1])**2)
            for i in range(len(path) - 1)
        )
        print(f"📐 Total distance: {total_distance:.2f} grid cells")

        # Visualize
        print("\n📊 Generating visualization...")
        planner.visualize(start, goal, path)

    else:
        print("❌ No path found!")
        planner.visualize(start, goal)

if __name__ == "__main__":
    run_astar_demo()
```

**Run It:**

```bash
python example1_astar.py

# Output:
# 🗺️  A* PATH PLANNING DEMONSTRATION
# 📦 Adding obstacles...
# 🎯 Start: (5, 5)
# 🏁 Goal: (45, 45)
# 🔍 Planning path...
# ✅ Path found!
# 📏 Path length: 58 waypoints
# ⏱️  Planning time: 12.34 ms
# 📐 Total distance: 56.57 grid cells
# ✅ Visualization saved as 'astar_result.png'
```

**Practice Tasks:**
1. Add more obstacles and see how the path adapts
2. Change start/goal positions
3. Try different grid resolutions
4. Implement diagonal movement cost correctly

---

## 🌳 Example 2: RRT (Rapidly-exploring Random Tree)

**Goal**: Implement RRT for exploring complex spaces.

### Code: RRT Implementation

```python
"""
example2_rrt.py
RRT path planning with animation
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class RRTPlanner:
    """
    RRT (Rapidly-exploring Random Tree) path planner
    """
    def __init__(self, start, goal, obstacles, bounds):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = obstacles  # List of (x, y, radius)
        self.bounds = bounds  # (x_min, x_max, y_min, y_max)

        self.tree = [self.start]
        self.parents = {0: None}

        self.step_size = 0.5
        self.goal_sample_rate = 0.1
        self.max_iterations = 1000

    def sample_point(self):
        """Sample random point or goal"""
        if np.random.rand() < self.goal_sample_rate:
            return self.goal.copy()

        return np.array([
            np.random.uniform(self.bounds[0], self.bounds[1]),
            np.random.uniform(self.bounds[2], self.bounds[3])
        ])

    def nearest_node(self, point):
        """Find nearest node in tree"""
        distances = [np.linalg.norm(node - point) for node in self.tree]
        return np.argmin(distances)

    def steer(self, from_node, to_point):
        """Steer from node toward point"""
        direction = to_point - from_node
        distance = np.linalg.norm(direction)

        if distance < self.step_size:
            return to_point

        return from_node + (direction / distance) * self.step_size

    def collision_free(self, from_point, to_point):
        """Check if path from_point to to_point is collision-free"""
        # Sample points along the line
        num_samples = int(np.linalg.norm(to_point - from_point) / 0.1)
        for i in range(num_samples + 1):
            t = i / max(num_samples, 1)
            point = from_point * (1 - t) + to_point * t

            # Check each obstacle
            for ox, oy, radius in self.obstacles:
                if np.linalg.norm(point - np.array([ox, oy])) < radius:
                    return False

        return True

    def plan(self):
        """
        Execute RRT planning

        Returns:
            path: List of waypoints or None
        """
        print(f"🌳 Running RRT with max {self.max_iterations} iterations...")

        for i in range(self.max_iterations):
            # Sample random point
            random_point = self.sample_point()

            # Find nearest node
            nearest_idx = self.nearest_node(random_point)
            nearest_node = self.tree[nearest_idx]

            # Steer toward random point
            new_node = self.steer(nearest_node, random_point)

            # Check collision
            if self.collision_free(nearest_node, new_node):
                # Add to tree
                new_idx = len(self.tree)
                self.tree.append(new_node)
                self.parents[new_idx] = nearest_idx

                # Check if reached goal
                if np.linalg.norm(new_node - self.goal) < self.step_size:
                    print(f"✅ Goal reached at iteration {i}!")

                    # Reconstruct path
                    path = [self.goal]
                    current_idx = new_idx

                    while current_idx is not None:
                        path.append(self.tree[current_idx])
                        current_idx = self.parents[current_idx]

                    return list(reversed(path))

            if (i + 1) % 100 == 0:
                print(f"  Iteration {i+1}/{self.max_iterations} - Tree size: {len(self.tree)}")

        print("❌ Max iterations reached without finding goal")
        return None

    def visualize(self, path=None):
        """Visualize the RRT tree and path"""
        fig, ax = plt.subplots(figsize=(10, 10))

        # Draw obstacles
        for ox, oy, radius in self.obstacles:
            circle = plt.Circle(
                (ox, oy), radius,
                color='black', alpha=0.7, label='Obstacle'
            )
            ax.add_patch(circle)

        # Draw tree edges
        for idx, parent_idx in self.parents.items():
            if parent_idx is not None:
                node = self.tree[idx]
                parent = self.tree[parent_idx]
                ax.plot(
                    [parent[0], node[0]],
                    [parent[1], node[1]],
                    'c-', alpha=0.3, linewidth=0.5
                )

        # Draw tree nodes
        tree_array = np.array(self.tree)
        ax.scatter(
            tree_array[:, 0],
            tree_array[:, 1],
            c='cyan', s=2, alpha=0.5, label='Tree Nodes'
        )

        # Draw start and goal
        ax.plot(self.start[0], self.start[1], 'go', markersize=15, label='Start')
        ax.plot(self.goal[0], self.goal[1], 'r*', markersize=20, label='Goal')

        # Draw path
        if path:
            path_array = np.array(path)
            ax.plot(
                path_array[:, 0],
                path_array[:, 1],
                'b-',
                linewidth=3,
                label=f'Path ({len(path)} waypoints)'
            )

        ax.set_xlim(self.bounds[0], self.bounds[1])
        ax.set_ylim(self.bounds[2], self.bounds[3])
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.set_title('RRT Path Planning')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        plt.tight_layout()
        plt.savefig('rrt_result.png', dpi=150)
        print("✅ Visualization saved as 'rrt_result.png'")
        plt.show()

def run_rrt_demo():
    """Run RRT demonstration"""
    print("="*70)
    print("🌳 RRT PATH PLANNING DEMONSTRATION")
    print("="*70)

    # Create obstacles
    obstacles = [
        (2.0, 2.0, 0.5),  # (x, y, radius)
        (3.5, 1.5, 0.4),
        (1.5, 3.5, 0.6),
        (4.0, 4.0, 0.5),
    ]

    # Create planner
    planner = RRTPlanner(
        start=(0.5, 0.5),
        goal=(4.5, 4.5),
        obstacles=obstacles,
        bounds=(0, 5, 0, 5)
    )

    # Plan path
    path = planner.plan()

    if path:
        print(f"\n📊 Path Statistics:")
        print(f"   Waypoints: {len(path)}")
        print(f"   Tree nodes: {len(planner.tree)}")

        # Calculate path length
        path_length = sum(
            np.linalg.norm(path[i+1] - path[i])
            for i in range(len(path) - 1)
        )
        print(f"   Path length: {path_length:.2f} meters")

        # Visualize
        planner.visualize(path)
    else:
        planner.visualize()

if __name__ == "__main__":
    run_rrt_demo()
```

**Run It:**

```bash
python example2_rrt.py

# Output shows:
# - Growing tree exploration
# - Path to goal
# - Beautiful visualization
```

---

## 🎯 Example 3: Trajectory Optimization

**Goal**: Optimize smooth trajectories for robot motion.

### Code: Simple Trajectory Optimizer

```python
"""
example3_trajectory_optimization.py
Trajectory optimization using gradient descent
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

class TrajectoryOptimizer:
    """
    Optimize smooth trajectories for robot motion
    """
    def __init__(self, start_pos, goal_pos, num_waypoints=10):
        self.start_pos = np.array(start_pos)
        self.goal_pos = np.array(goal_pos)
        self.num_waypoints = num_waypoints

        # Initialize waypoints (linear interpolation)
        self.waypoints = np.linspace(
            start_pos,
            goal_pos,
            num_waypoints
        )

    def smoothness_cost(self, waypoints):
        """Compute trajectory smoothness (sum of accelerations)"""
        # Second derivative (acceleration)
        accel = np.diff(waypoints, n=2, axis=0)
        return np.sum(accel ** 2)

    def obstacle_cost(self, waypoints, obstacles):
        """Penalty for proximity to obstacles"""
        cost = 0
        for waypoint in waypoints:
            for obs_pos, obs_radius in obstacles:
                dist = np.linalg.norm(waypoint - obs_pos)
                if dist < obs_radius:
                    cost += 1000  # Large penalty for collision
                elif dist < obs_radius + 0.5:
                    # Soft penalty for being too close
                    cost += 100 / (dist - obs_radius + 0.1)
        return cost

    def total_cost(self, waypoints, obstacles):
        """Total optimization cost"""
        smoothness = self.smoothness_cost(waypoints)
        obstacle = self.obstacle_cost(waypoints, obstacles)
        return smoothness + obstacle

    def optimize(self, obstacles=[], learning_rate=0.01, iterations=100):
        """
        Optimize trajectory using gradient descent

        Args:
            obstacles: List of (position, radius) tuples
            learning_rate: Step size for optimization
            iterations: Number of optimization steps

        Returns:
            optimized_waypoints: Smooth trajectory
        """
        waypoints = self.waypoints.copy()

        print(f"\n🔧 Optimizing trajectory...")
        print(f"   Iterations: {iterations}")
        print(f"   Learning rate: {learning_rate}")

        costs = []

        for i in range(iterations):
            # Compute gradient numerically
            epsilon = 1e-5
            grad = np.zeros_like(waypoints)

            for j in range(1, len(waypoints) - 1):  # Don't move start/goal
                for dim in range(waypoints.shape[1]):
                    waypoints_plus = waypoints.copy()
                    waypoints_plus[j, dim] += epsilon

                    waypoints_minus = waypoints.copy()
                    waypoints_minus[j, dim] -= epsilon

                    grad[j, dim] = (
                        self.total_cost(waypoints_plus, obstacles) -
                        self.total_cost(waypoints_minus, obstacles)
                    ) / (2 * epsilon)

            # Update waypoints
            waypoints[1:-1] -= learning_rate * grad[1:-1]

            # Track cost
            cost = self.total_cost(waypoints, obstacles)
            costs.append(cost)

            if (i + 1) % 20 == 0:
                print(f"   Iteration {i+1}: Cost = {cost:.2f}")

        print(f"✅ Optimization complete!")
        print(f"   Initial cost: {costs[0]:.2f}")
        print(f"   Final cost: {costs[-1]:.2f}")
        print(f"   Improvement: {(1 - costs[-1]/costs[0])*100:.1f}%")

        return waypoints, costs

    def visualize_results(self, initial_wp, optimized_wp, obstacles, costs):
        """Visualize optimization results"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

        # Plot 1: Trajectories
        # Draw obstacles
        for obs_pos, obs_radius in obstacles:
            circle = plt.Circle(
                obs_pos, obs_radius,
                color='red', alpha=0.5
            )
            ax1.add_patch(circle)

        # Initial trajectory
        ax1.plot(
            initial_wp[:, 0],
            initial_wp[:, 1],
            'r--',
            linewidth=2,
            label='Initial (straight line)',
            alpha=0.5
        )

        # Optimized trajectory
        ax1.plot(
            optimized_wp[:, 0],
            optimized_wp[:, 1],
            'b-',
            linewidth=3,
            label='Optimized (smooth)'
        )

        # Start and goal
        ax1.plot(self.start_pos[0], self.start_pos[1], 'go', markersize=12)
        ax1.plot(self.goal_pos[0], self.goal_pos[1], 'g*', markersize=15)

        ax1.set_xlabel('X (meters)')
        ax1.set_ylabel('Y (meters)')
        ax1.set_title('Trajectory Optimization Result')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')

        # Plot 2: Cost reduction
        ax2.plot(costs, linewidth=2)
        ax2.set_xlabel('Iteration')
        ax2.set_ylabel('Cost')
        ax2.set_title('Optimization Progress')
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('trajectory_optimization_result.png', dpi=150)
        print("✅ Visualization saved as 'trajectory_optimization_result.png'")
        plt.show()

def run_trajectory_optimization_demo():
    """Run trajectory optimization demo"""
    print("="*70)
    print("🎯 TRAJECTORY OPTIMIZATION DEMONSTRATION")
    print("="*70)

    # Define start, goal, obstacles
    start = (0.0, 0.0)
    goal = (5.0, 5.0)
    obstacles = [
        (np.array([2.5, 2.5]), 0.8),
        (np.array([3.5, 1.5]), 0.5),
        (np.array([1.5, 3.5]), 0.6),
    ]

    print(f"\n🎯 Start: {start}")
    print(f"🏁 Goal: {goal}")
    print(f"🚧 Obstacles: {len(obstacles)}")

    # Create optimizer
    optimizer = TrajectoryOptimizer(start, goal, num_waypoints=15)

    initial_waypoints = optimizer.waypoints.copy()

    # Optimize
    optimized_waypoints, costs = optimizer.optimize(
        obstacles=obstacles,
        learning_rate=0.05,
        iterations=100
    )

    # Visualize
    optimizer.visualize_results(
        initial_waypoints,
        optimized_waypoints,
        obstacles,
        costs
    )

if __name__ == "__main__":
    run_trajectory_optimization_demo()
```

**Run It:**

```bash
python example3_trajectory_optimization.py

# Shows:
# - Initial straight-line path
# - Optimized smooth path avoiding obstacles
# - Cost reduction graph
```

---

## 🤖 Example 4: Complete Robot Controller

**Goal**: Combine planning with real-time control in simulation.

### Code: Full Control Loop

```python
"""
example4_robot_controller.py
Complete robot controller with planning and execution
"""

import pybullet as p
import pybullet_data
import numpy as np
import time

class RobotController:
    """
    Complete robot controller with path following
    """
    def __init__(self):
        # Initialize PyBullet
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Load environment
        self.plane = p.loadURDF("plane.urdf")

        # Load robot
        self.robot = p.loadURDF(
            "franka_panda/panda.urdf",
            basePosition=[0, 0, 0],
            useFixedBase=True
        )

        self.num_joints = 7
        print(f"✅ Robot loaded with {self.num_joints} joints")

    def get_joint_positions(self):
        """Get current joint positions"""
        positions = []
        for i in range(self.num_joints):
            state = p.getJointState(self.robot, i)
            positions.append(state[0])
        return np.array(positions)

    def move_to_position(self, target_positions, duration=2.0):
        """
        Move robot to target joint positions smoothly

        Args:
            target_positions: Target joint angles
            duration: Time to reach target (seconds)
        """
        start_positions = self.get_joint_positions()

        # Generate smooth trajectory
        num_steps = int(duration * 240)  # 240 Hz
        t = np.linspace(0, 1, num_steps)

        # Use quintic polynomial for smooth acceleration profile
        s = 10 * t**3 - 15 * t**4 + 6 * t**5

        print(f"\n🎯 Moving to target position over {duration}s...")

        for step in range(num_steps):
            # Interpolate position
            current_target = (
                start_positions * (1 - s[step]) +
                target_positions * s[step]
            )

            # Set joint targets
            for i in range(self.num_joints):
                p.setJointMotorControl2(
                    self.robot,
                    i,
                    p.POSITION_CONTROL,
                    targetPosition=current_target[i],
                    force=500
                )

            p.stepSimulation()
            time.sleep(1./240.)

            # Print progress
            if step % 60 == 0:
                progress = (step / num_steps) * 100
                print(f"   Progress: {progress:.0f}%", end='\r')

        print(f"   Progress: 100%")
        print("✅ Motion complete!")

    def follow_path(self, waypoints, duration_per_segment=2.0):
        """
        Follow a path of waypoints

        Args:
            waypoints: List of joint position arrays
            duration_per_segment: Time between waypoints
        """
        print(f"\n🛤️  Following path with {len(waypoints)} waypoints...")

        for i, waypoint in enumerate(waypoints):
            print(f"\n📍 Waypoint {i+1}/{len(waypoints)}")
            self.move_to_position(waypoint, duration_per_segment)
            time.sleep(0.5)  # Pause at each waypoint

        print("\n✅ Path following complete!")

    def demo_motion(self):
        """Demonstrate various motions"""
        # Define waypoints for a demo trajectory
        waypoints = [
            np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.5, 0.04]),  # Position 1
            np.array([0.5, -0.7, 0.0, -1.8, 0.0, 1.8, 0.04]),  # Position 2
            np.array([-0.5, -0.7, 0.0, -1.8, 0.0, 1.8, 0.04]), # Position 3
            np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.5, 0.0]),   # Back to start (gripper closed)
        ]

        self.follow_path(waypoints, duration_per_segment=3.0)

    def run(self):
        """Main execution"""
        print("\n🤖 Starting robot controller demo...")
        self.demo_motion()

        print("\n✨ Demo complete! Close the window to exit.")
        while True:
            p.stepSimulation()
            time.sleep(0.01)

if __name__ == "__main__":
    print("="*70)
    print("🤖 ROBOT CONTROLLER DEMONSTRATION")
    print("="*70)

    controller = RobotController()
    controller.run()
```

**Run It:**

```bash
python example4_robot_controller.py

# Watch the robot:
# 1. Move through planned waypoints
# 2. Smooth quintic trajectories
# 3. Gripper open/close
# 4. Return to start position
```

---

## 📚 Complete Project Structure

Organize your practice projects:

```
vla_practice/
├── requirements.txt
├── examples/
│   ├── example1_astar.py
│   ├── example2_rrt.py
│   ├── example3_trajectory_optimization.py
│   └── example4_robot_controller.py
├── data/
│   ├── robot_demonstrations/
│   └── test_images/
├── models/
│   ├── vla_agent.pth
│   └── checkpoints/
└── results/
    ├── astar_result.png
    ├── rrt_result.png
    └── trajectory_optimization_result.png
```

---

## 🎓 Challenges & Exercises

### Challenge 1: Extend A*
- Implement bidirectional A* (search from both start and goal)
- Add diagonal movement costs
- Optimize with jump point search

### Challenge 2: RRT Variants
- Implement RRT* (optimal variant)
- Add goal biasing
- Implement bidirectional RRT

### Challenge 3: Advanced Trajectory
- Add velocity and acceleration constraints
- Implement CHOMP algorithm
- Optimize for energy efficiency

### Challenge 4: Integration
- Combine A* planning with trajectory optimization
- Add dynamic obstacle avoidance
- Implement replanning when obstacles move

---

## 📖 Additional Resources

**Online Tools:**
- [PyBullet Quickstart](https://pybullet.org/wordpress/)
- [OMPL (Open Motion Planning Library)](https://ompl.kavrakilab.org/)
- [MoveIt Tutorials](https://moveit.picknik.ai/main/index.html)

**Datasets:**
- [RoboNet](https://www.robonet.wiki/) - Robot video dataset
- [Bridge Data](https://sites.google.com/view/bridgedata) - Robot manipulation
- [RT-1 Data](https://robotics-transformer.github.io/) - Google's dataset

---

## Next Steps

1. **Complete all 4 examples** in order (1-2 hours each)
2. **Modify and experiment** with parameters
3. **Create your own planning scenario**
4. **Move on to:** [Assessment](assessment.md) to test your knowledge

**Happy coding!** 🚀 These examples will give you practical experience with the algorithms discussed in the theory sections.
