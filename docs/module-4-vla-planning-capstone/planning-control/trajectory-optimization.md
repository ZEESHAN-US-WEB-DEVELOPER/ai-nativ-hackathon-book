# Trajectory Optimization

Trajectory optimization generates smooth, dynamically feasible paths that minimize cost while satisfying constraints. Unlike path planning (which finds geometric paths), trajectory optimization considers robot dynamics, smoothness, and time-optimality.

## Problem Formulation

**Find trajectory**: Position, velocity, and acceleration over time from start to goal

**Minimize cost**: Integral of cost function over trajectory (penalize acceleration, obstacles, time)

**Subject to constraints**:
- Dynamics: Robot dynamics equations
- Collision avoidance: Stay in collision-free space
- Joint limits: Respect joint angle bounds
- Velocity limits: Respect velocity bounds
- Boundary conditions: Start at initial config, end at goal config

## Trajectory Representation

### Waypoint Representation

Discretize trajectory into N waypoints:
```
τ = [q_0, q_1, ..., q_N] where q_i = q(t_i)
```

**Optimization variables**: Joint positions at each waypoint

**Interpolation**: Cubic splines, linear, or polynomial

### Basis Functions

Represent trajectory as weighted sum of basis functions:
```
q(t) = Σ w_i φ_i(t)
```

**Common bases**:
- Polynomial: φ_i(t) = t^i
- Fourier: φ_i(t) = sin(ωt), cos(ωt)
- B-splines: Piecewise polynomials

**Optimization variables**: Weights w_i

## CHOMP: Covariant Hamiltonian Optimization for Motion Planning

CHOMP optimizes trajectories by minimizing obstacle cost and smoothness cost.

### Cost Function

```
C(τ) = λ_smooth * C_smooth(τ) + λ_obs * C_obs(τ)
```

**Smoothness Cost**:
```
C_smooth = ∫ ||q̈(t)||² dt
```
Penalizes high accelerations (jerk)

**Obstacle Cost**:
```
C_obs = Σ_i cost_obs(q_i) where cost_obs(q) = max(0, ε - d(q))
```
d(q) = distance to nearest obstacle
ε = safety margin

### Algorithm

```python
def chomp(initial_trajectory, obstacles, max_iter=100):
    τ = initial_trajectory

    for iteration in range(max_iter):
        # Compute gradient of cost function
        ∇C_smooth = compute_smoothness_gradient(τ)
        ∇C_obs = compute_obstacle_gradient(τ, obstacles)

        ∇C = λ_smooth * ∇C_smooth + λ_obs * ∇C_obs

        # Update trajectory (gradient descent)
        τ = τ - α * ∇C

        # Check convergence
        if ||∇C|| < threshold:
            break

    return τ
```

**Advantages**:
- Smooth trajectories
- Handles narrow passages
- Fast convergence

**Limitations**:
- Local minima
- Requires good initial trajectory

## TrajOpt: Trajectory Optimization

TrajOpt formulates trajectory optimization as a large-scale nonlinear optimization problem.

### Formulation

```
minimize    Σ_t cost(q_t, q̇_t, q̈_t)
subject to  dynamics constraints
            collision avoidance constraints
            joint limit constraints
```

### Sequential Convex Optimization

TrajOpt iteratively approximates nonlinear constraints as convex constraints.

```python
def trajopt(initial_trajectory, constraints, max_iter=50):
    τ = initial_trajectory

    for iteration in range(max_iter):
        # Linearize constraints around current trajectory
        linearized_constraints = []
        for constraint in constraints:
            linearized = constraint.linearize(τ)
            linearized_constraints.append(linearized)

        # Solve convex subproblem
        τ_new = solve_convex_problem(τ, linearized_constraints)

        # Line search for step size
        α = line_search(τ, τ_new, constraints)
        τ = τ + α * (τ_new - τ)

        if converged(τ, τ_new):
            break

    return τ
```

**Advantages**:
- Handles complex constraints
- Reliable convergence
- Used in MoveIt

**Disadvantages**:
- Computationally expensive
- Requires good solver (IPOPT, SNOPT)

## STOMP: Stochastic Trajectory Optimization

STOMP uses stochastic optimization to explore trajectory space and avoid local minima.

### Algorithm

```python
def stomp(initial_trajectory, cost_function, K=10, max_iter=100):
    τ_best = initial_trajectory
    cost_best = cost_function(τ_best)

    for iteration in range(max_iter):
        # Generate noisy trajectories
        trajectories = []
        for k in range(K):
            noise = sample_gaussian_noise()
            τ_k = τ_best + noise
            trajectories.append(τ_k)

        # Evaluate costs
        costs = [cost_function(τ) for τ in trajectories]

        # Update using weighted average
        weights = compute_weights(costs)  # Lower cost → higher weight
        τ_best = weighted_average(trajectories, weights)

        cost_best = cost_function(τ_best)

    return τ_best
```

**Noise Generation**:
```python
def sample_gaussian_noise(N, covariance):
    """Generate smooth noise using Gaussian process"""
    noise = np.random.multivariate_normal(mean=0, cov=covariance, size=N)
    return noise
```

**Weight Computation** (softmax with temperature):
```python
def compute_weights(costs, temperature=1.0):
    exp_costs = np.exp(-costs / temperature)
    weights = exp_costs / np.sum(exp_costs)
    return weights
```

**Advantages**:
- Avoids local minima better than gradient descent
- No gradient computation needed
- Robust to discontinuous cost functions

**Disadvantages**:
- Slower than gradient-based methods
- Requires many trajectory samples

## Time-Optimal Trajectory Generation

Minimize execution time while respecting constraints.

### Problem Formulation

```
minimize    T (total execution time)
subject to  q(0) = q_start, q(T) = q_goal
            |q̇(t)| ≤ v_max
            |q̈(t)| ≤ a_max
```

### Time-Scaling Approach

1. Generate geometric path: s(t) ∈ [0, 1]
2. Optimize time-scaling: find s(t) to minimize T

```python
def time_optimal_trajectory(path, v_max, a_max):
    """
    path: geometric path parameterized by s ∈ [0, 1]
    v_max, a_max: velocity and acceleration limits
    """
    # Compute maximum velocity at each s
    v_limit = []
    for s in np.linspace(0, 1, 100):
        curvature = compute_curvature(path, s)
        v_s = min(v_max, sqrt(a_max / curvature))
        v_limit.append(v_s)

    # Forward pass: accelerate as much as possible
    v_forward = [0]
    for i in range(1, len(v_limit)):
        v_max_accel = sqrt(v_forward[-1]**2 + 2 * a_max * ds)
        v_forward.append(min(v_limit[i], v_max_accel))

    # Backward pass: decelerate to respect limits
    v_backward = [0] * len(v_limit)
    v_backward[-1] = 0
    for i in range(len(v_limit) - 2, -1, -1):
        v_max_decel = sqrt(v_backward[i+1]**2 + 2 * a_max * ds)
        v_backward[i] = min(v_limit[i], v_forward[i], v_max_decel)

    # Compute time stamps
    t = [0]
    for i in range(1, len(v_backward)):
        dt = ds / v_backward[i]
        t.append(t[-1] + dt)

    return t, v_backward
```

**Result**: Minimum-time trajectory respecting velocity and acceleration limits

## Differential Dynamic Programming (DDP)

DDP optimizes trajectories by iteratively improving control inputs.

### Algorithm (Simplified)

```python
def ddp(initial_trajectory, dynamics, cost, max_iter=50):
    x = initial_trajectory  # States
    u = initial_controls    # Controls

    for iteration in range(max_iter):
        # Forward pass: simulate dynamics
        x_new, cost_total = simulate(x, u, dynamics, cost)

        # Backward pass: compute optimal control updates
        Δu = backward_pass(x_new, u, dynamics, cost)

        # Line search
        α = line_search(x, u, Δu, dynamics, cost)
        u = u + α * Δu

        if converged(Δu):
            break

    return x, u
```

**Used in**:
- Model Predictive Control (MPC)
- Whole-body control for humanoids
- Legged locomotion

## Practical Considerations

### Initial Trajectory

Good initial trajectory is critical:
- Use RRT/RRT* to get feasible path
- Interpolate with cubic splines
- Shortcut and smooth

```python
def get_initial_trajectory(start, goal, obstacles):
    # Get geometric path
    path = rrt(start, goal, obstacles)

    # Interpolate to waypoints
    trajectory = interpolate_cubic_spline(path, num_waypoints=50)

    return trajectory
```

### Cost Function Design

Balance multiple objectives:
```python
def cost_function(trajectory):
    c_smooth = smoothness_cost(trajectory)
    c_obstacle = obstacle_cost(trajectory, obstacles)
    c_time = time_cost(trajectory)
    c_energy = energy_cost(trajectory)

    return w1*c_smooth + w2*c_obstacle + w3*c_time + w4*c_energy
```

**Tuning weights**: Start with obstacle avoidance, add smoothness, then optimize time/energy

### Collision Checking

Efficient collision checking is critical:
```python
def trajectory_collision_free(trajectory, obstacles, resolution=0.01):
    for i in range(len(trajectory) - 1):
        # Interpolate between waypoints
        q_start, q_end = trajectory[i], trajectory[i+1]
        num_checks = int(distance(q_start, q_end) / resolution)

        for t in np.linspace(0, 1, num_checks):
            q = interpolate(q_start, q_end, t)
            if in_collision(q, obstacles):
                return False

    return True
```

## Tools and Libraries

**MoveIt** (ROS):
- Integrates CHOMP, TrajOpt, STOMP
- Easy to use for manipulation planning

**OMPL** (Open Motion Planning Library):
- Path planning + trajectory optimization
- Python and C++ bindings

**Drake** (MIT):
- Advanced trajectory optimization
- Differential dynamic programming
- Whole-body planning

**CasADi** (Optimal Control):
- Symbolic differentiation
- Nonlinear optimization
- Fast trajectory optimization

## Example: MoveIt Integration

```python
import moveit_commander

# Initialize MoveIt
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")

# Set planner
group.set_planner_id("TrajOpt")  # or "CHOMP", "STOMP"

# Plan trajectory
group.set_pose_target(target_pose)
plan = group.plan()

# Execute
group.execute(plan, wait=True)
```

## Summary

Trajectory optimization generates smooth, feasible motions by minimizing cost functions subject to constraints. CHOMP optimizes using gradient descent, TrajOpt uses sequential convex optimization, and STOMP uses stochastic sampling. Choose the method based on problem requirements: CHOMP for speed, TrajOpt for reliability, STOMP for robustness.

Next: [Task Planning](task-planning.md) - Learn high-level symbolic planning for complex tasks.
