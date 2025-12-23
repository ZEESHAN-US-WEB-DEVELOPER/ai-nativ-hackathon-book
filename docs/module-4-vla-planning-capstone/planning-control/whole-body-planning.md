# Whole-Body Planning for Humanoid Robots

Whole-body planning coordinates all joints of a humanoid robot simultaneously, considering balance, contact constraints, and full-body kinematics. This section covers humanoid-specific planning challenges and techniques.

## Overview

Humanoid robots have unique challenges:
- **High Dimensionality**: 30-50 DOF (compared to 6-7 for manipulators)
- **Balance Constraints**: Must maintain stability
- **Multiple Contacts**: Feet, hands can make simultaneous contacts
- **Redundancy**: Multiple configurations achieve same end-effector pose

## Balance and Stability

### Center of Mass (CoM) Constraints

```python
def compute_center_of_mass(joint_config):
    """Compute robot's center of mass from joint configuration"""
    total_mass = 0
    com = np.zeros(3)

    for link in robot.links:
        link_pose = forward_kinematics(joint_config, link)
        com += link.mass * link_pose.position
        total_mass += link.mass

    return com / total_mass
```

### Zero Moment Point (ZMP)

ZMP must remain inside support polygon for static stability:

```python
def check_static_stability(joint_config, contact_points):
    """Check if configuration is statically stable"""
    # Compute center of mass
    com = compute_center_of_mass(joint_config)

    # Project CoM onto ground plane
    com_2d = com[:2]  # (x, y)

    # Compute support polygon (convex hull of contact points)
    support_polygon = convex_hull([pt[:2] for pt in contact_points])

    # Check if CoM projection is inside support polygon
    return point_in_polygon(com_2d, support_polygon)
```

### Dynamic Balance

For dynamic motion (walking, running):

```python
def compute_zmp(joint_config, joint_velocity, joint_accel):
    """Compute Zero Moment Point for dynamic stability"""
    com = compute_center_of_mass(joint_config)
    com_velocity = compute_com_velocity(joint_config, joint_velocity)
    com_accel = compute_com_acceleration(joint_config, joint_velocity, joint_accel)

    # ZMP formula
    g = 9.81  # gravity
    zmp_x = com.x - (com.z / (com_accel.z + g)) * com_accel.x
    zmp_y = com.y - (com.z / (com_accel.z + g)) * com_accel.y

    return np.array([zmp_x, zmp_y])
```

## Whole-Body Inverse Kinematics

Compute joint configuration to achieve desired end-effector poses while satisfying constraints.

### Problem Formulation

```
minimize    ||J(q)q̇ - v_desired||²     # Track desired velocities
            + ||q - q_nominal||²         # Stay close to nominal posture
subject to  q_min ≤ q ≤ q_max           # Joint limits
            CoM within support polygon   # Balance
            No self-collisions
```

### Quadratic Programming Formulation

```python
import cvxpy as cp

def whole_body_ik(desired_poses, current_config, support_polygon):
    """
    desired_poses: Dict of end-effector name -> desired pose
    current_config: Current joint configuration
    support_polygon: Support polygon for balance
    """
    n_joints = len(current_config)

    # Decision variable: joint velocities
    q_dot = cp.Variable(n_joints)

    # Objective: Track desired end-effector velocities + posture
    cost = 0

    for ee_name, desired_pose in desired_poses.items():
        # Compute Jacobian for end-effector
        J = compute_jacobian(current_config, ee_name)

        # Desired velocity
        current_pose = forward_kinematics(current_config, ee_name)
        v_desired = compute_velocity_error(current_pose, desired_pose)

        # Track end-effector velocity
        cost += cp.sum_squares(J @ q_dot - v_desired)

    # Posture cost (stay close to nominal)
    q_nominal = get_nominal_posture()
    cost += 0.1 * cp.sum_squares(current_config + q_dot * dt - q_nominal)

    # Constraints
    constraints = []

    # Joint limits
    q_next = current_config + q_dot * dt
    constraints.append(q_next >= joint_limits_min)
    constraints.append(q_next <= joint_limits_max)

    # Balance constraint (CoM must be in support polygon)
    com_jacobian = compute_com_jacobian(current_config)
    com_velocity = com_jacobian @ q_dot

    # Approximate: CoM projection must stay in polygon
    # (linearize around current CoM)
    constraints.append(com_inside_polygon_constraint(com_velocity, support_polygon))

    # Solve QP
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve()

    return q_dot.value
```

## Hierarchical Inverse Kinematics

Use task priorities to resolve conflicts:

```python
class HierarchicalIK:
    def __init__(self, robot):
        self.robot = robot
        self.tasks = []  # List of (task, priority)

    def add_task(self, task, priority):
        self.tasks.append((task, priority))

    def solve(self, current_config):
        # Sort tasks by priority
        self.tasks.sort(key=lambda x: x[1])

        q_dot = np.zeros(self.robot.n_joints)
        null_space_projector = np.eye(self.robot.n_joints)

        for task, priority in self.tasks:
            # Compute task Jacobian
            J = task.compute_jacobian(current_config)

            # Desired task velocity
            v_desired = task.compute_desired_velocity(current_config)

            # Project into null space of higher-priority tasks
            J_projected = J @ null_space_projector

            # Solve for joint velocities in null space
            q_dot_task = np.linalg.pinv(J_projected) @ (v_desired - J @ q_dot)

            # Update joint velocities
            q_dot += q_dot_task

            # Update null space projector
            null_space_projector = null_space_projector @ (np.eye(self.robot.n_joints) - np.linalg.pinv(J) @ J)

        return q_dot
```

**Example Usage**:
```python
ik_solver = HierarchicalIK(robot)

# Priority 1: Balance (highest priority)
ik_solver.add_task(BalanceTask(support_polygon), priority=1)

# Priority 2: Right hand end-effector
ik_solver.add_task(EndEffectorTask("right_hand", desired_pose), priority=2)

# Priority 3: Posture (lowest priority)
ik_solver.add_task(PostureTask(nominal_posture), priority=3)

q_dot = ik_solver.solve(current_config)
```

## Whole-Body Motion Planning

### Planning with Contact Constraints

```python
def plan_whole_body_motion(start_config, goal_config, contact_sequence):
    """
    contact_sequence: List of contact states (e.g., [(left_foot, right_foot), (left_foot, right_foot, right_hand), ...])
    """
    path = [start_config]
    current_config = start_config

    for i, contacts in enumerate(contact_sequence):
        # Plan motion within this contact mode
        next_config = goal_config if i == len(contact_sequence) - 1 else compute_intermediate_config(contacts)

        # Use contact-constrained planning
        segment = plan_contact_mode_transition(
            current_config,
            next_config,
            contacts
        )

        path.extend(segment)
        current_config = segment[-1]

    return path
```

### Contact Mode Transitions

```python
def plan_contact_mode_transition(start, goal, new_contacts, breaking_contacts):
    """Plan transition between contact modes"""

    # Phase 1: Break old contacts (lift foot/hand)
    phase1 = plan_motion_maintaining_contacts(
        start,
        goal_phase1,
        active_contacts=new_contacts.intersection(breaking_contacts)
    )

    # Phase 2: Move in free space
    phase2 = plan_motion_maintaining_contacts(
        phase1[-1],
        goal_phase2,
        active_contacts=new_contacts
    )

    # Phase 3: Make new contacts (place foot/hand)
    phase3 = plan_motion_maintaining_contacts(
        phase2[-1],
        goal,
        active_contacts=new_contacts
    )

    return phase1 + phase2 + phase3
```

## Humanoid Locomotion Planning

### Footstep Planning

```python
def plan_footsteps(start_pose, goal_pose, terrain):
    """Plan sequence of footsteps"""
    footsteps = [start_pose]
    current = start_pose

    while distance(current, goal_pose) > threshold:
        # Sample candidate footstep locations
        candidates = sample_footstep_candidates(current)

        # Evaluate candidates
        best_footstep = None
        best_score = -np.inf

        for candidate in candidates:
            # Check feasibility
            if not is_valid_footstep(candidate, terrain):
                continue

            # Compute score (distance to goal, stability, etc.)
            score = evaluate_footstep(candidate, goal_pose, current)

            if score > best_score:
                best_score = score
                best_footstep = candidate

        if best_footstep is None:
            return None  # No valid footstep found

        footsteps.append(best_footstep)
        current = best_footstep

    return footsteps
```

### Walking Motion Generation

```python
def generate_walking_motion(footsteps):
    """Generate full-body motion from footstep plan"""
    motion = []

    for i in range(len(footsteps) - 1):
        swing_foot = footsteps[i]
        landing_foot = footsteps[i + 1]

        # Phase 1: Shift weight to support leg
        weight_shift = generate_weight_shift_motion(swing_foot, landing_foot)
        motion.extend(weight_shift)

        # Phase 2: Swing foot trajectory (avoid ground)
        swing_trajectory = generate_swing_trajectory(swing_foot, landing_foot, clearance=0.05)

        # Phase 3: Whole-body motion tracking swing trajectory
        for swing_pose in swing_trajectory:
            # Solve whole-body IK
            config = whole_body_ik(
                desired_poses={"swing_foot": swing_pose},
                current_config=motion[-1],
                support_polygon=compute_support_polygon([landing_foot])
            )
            motion.append(config)

    return motion
```

## Tools and Libraries

### DART (Dynamic Animation and Robotics Toolkit)

```python
import dartpy as dart

# Load humanoid robot
world = dart.simulation.World()
robot_skeleton = dart.utils.DartLoader().parseSkeleton("atlas.urdf")
world.addSkeleton(robot_skeleton)

# Create whole-body IK
ik = dart.dynamics.InverseKinematics(robot_skeleton)

# Set target
end_effector = robot_skeleton.getBodyNode("right_hand")
target = dart.math.Isometry3()
target.set_translation([0.5, 0.3, 1.0])

ik.addTarget(end_effector, target)

# Solve
ik.solve()

# Get joint configuration
q = robot_skeleton.getPositions()
```

### Drake (MIT)

Drake provides advanced whole-body planning and control:

```python
from pydrake.all import (
    DiagramBuilder, MultibodyPlant, SceneGraph,
    InverseKinematics, Solve
)

# Build diagram
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

# Load robot
robot = Parser(plant).AddModelFromFile("atlas_v5.urdf")
plant.Finalize()

# Inverse kinematics
ik = InverseKinematics(plant)

# Add end-effector constraint
hand_frame = plant.GetFrameByName("r_hand")
ik.AddPositionConstraint(
    hand_frame,
    [0, 0, 0],  # Point in hand frame
    plant.world_frame(),
    [0.5, 0.5, 1.0],  # World position
    [0.5, 0.5, 1.0]   # (exact position)
)

# Add balance constraint
ik.AddPointToPointDistanceConstraint(
    plant.GetBodyByName("pelvis"),
    [0, 0, 0],  # CoM
    plant.world_frame(),
    [0.5, 0.5],  # Support polygon center
    0.0,  # min distance
    0.5   # max distance (must be close to support)
)

# Solve
result = Solve(ik.prog())
q_solution = result.GetSolution(ik.q())
```

### Pinocchio (Fast Rigid Body Dynamics)

```python
import pinocchio as pin

# Load model
model = pin.buildModelFromUrdf("humanoid.urdf")
data = model.createData()

# Compute forward kinematics
q = np.random.randn(model.nq)
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

# Compute Jacobian
frame_id = model.getFrameId("right_hand")
J = pin.computeFrameJacobian(model, data, q, frame_id)

# Compute center of mass
pin.centerOfMass(model, data, q)
com = data.com[0]
```

## Complete Example: Reaching Task

```python
def plan_humanoid_reach(robot, target_pose):
    """Plan whole-body motion for humanoid to reach target"""

    # 1. Footstep planning (if needed to get closer)
    if distance(robot.base_position, target_pose) > arm_reach:
        footsteps = plan_footsteps(robot.base_position, target_pose, terrain)
        for footstep in footsteps:
            walking_motion = generate_walking_motion([robot.feet_poses, footstep])
            execute(walking_motion)

    # 2. Whole-body reaching motion
    ik_solver = HierarchicalIK(robot)

    # Priority 1: Balance
    ik_solver.add_task(BalanceTask(support_polygon), priority=1)

    # Priority 2: Right hand reaches target
    ik_solver.add_task(EndEffectorTask("right_hand", target_pose), priority=2)

    # Priority 3: Look at target
    ik_solver.add_task(GazeTask("head", target_pose), priority=3)

    # Priority 4: Comfortable posture
    ik_solver.add_task(PostureTask(nominal_posture), priority=4)

    # Solve iteratively until converged
    motion = []
    current_config = robot.get_current_config()

    for i in range(max_iterations):
        q_dot = ik_solver.solve(current_config)
        current_config += q_dot * dt
        motion.append(current_config)

        if reached_target(current_config, target_pose):
            break

    return motion
```

## Summary

Whole-body planning for humanoid robots requires coordinating many degrees of freedom while maintaining balance and satisfying contact constraints. Key techniques include whole-body inverse kinematics with task hierarchies, contact-constrained planning, and footstep planning for locomotion. Tools like DART, Drake, and Pinocchio provide efficient implementations of these algorithms.

Next: [Assessment](assessment.md) - Test your understanding of motion planning and control.
