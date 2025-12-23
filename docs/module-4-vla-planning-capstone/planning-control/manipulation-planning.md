# Manipulation Planning

Manipulation planning focuses on planning for grasping, object interaction, and contact-rich tasks. This section covers grasp planning, contact reasoning, and manipulation-specific motion planning.

## Overview

Manipulation planning addresses:
- **Grasp Planning**: Where and how to grasp objects
- **Pre-grasp and Post-grasp Motion**: Approach and retreat trajectories
- **Contact Constraints**: Maintaining contact during manipulation
- **Object Manipulation**: Planning for pushing, pulling, placing

## Grasp Planning

### Grasp Representation

**6-DOF Grasp Pose**: Position (x, y, z) + Orientation (roll, pitch, yaw)

**Grasp Quality Metrics**:
1. **Force Closure**: Can resist arbitrary external forces
2. **Form Closure**: Geometric constraint prevents motion
3. **Grasp Stability**: Resistant to perturbations

### Analytical Grasp Planning

For simple geometric shapes, compute grasps analytically:

```python
def plan_cylinder_grasp(cylinder_center, cylinder_radius, cylinder_height):
    """Plan top-down grasp for cylinder"""
    grasp_pose = Pose()

    # Position: above cylinder center
    grasp_pose.position.x = cylinder_center.x
    grasp_pose.position.y = cylinder_center.y
    grasp_pose.position.z = cylinder_center.z + cylinder_height/2 + gripper_offset

    # Orientation: gripper pointing down
    grasp_pose.orientation = Quaternion(0, π/2, 0)  # Pitch 90 degrees

    return grasp_pose
```

### Sampling-Based Grasp Planning

For complex objects, sample candidate grasps and evaluate quality:

```python
def sample_grasps(object_mesh, num_samples=100):
    candidate_grasps = []

    for i in range(num_samples):
        # Sample point on object surface
        point = sample_surface_point(object_mesh)
        normal = compute_surface_normal(object_mesh, point)

        # Compute grasp pose
        grasp_pose = compute_grasp_from_surface(point, normal)

        # Evaluate grasp quality
        quality = evaluate_grasp_quality(grasp_pose, object_mesh)

        if quality > threshold:
            candidate_grasps.append((grasp_pose, quality))

    # Return best grasp
    return max(candidate_grasps, key=lambda x: x[1])
```

**Grasp Quality Evaluation**:
```python
def evaluate_grasp_quality(grasp_pose, object_mesh):
    # 1. Check collision with object
    if self_collision(grasp_pose, object_mesh):
        return 0.0

    # 2. Compute contact points
    contacts = compute_contacts(grasp_pose, object_mesh)

    if len(contacts) < 2:
        return 0.0  # Insufficient contacts

    # 3. Check force closure
    force_closure = check_force_closure(contacts, object_mesh)

    # 4. Grasp stability (antipodal grasp preferred)
    stability = compute_stability(contacts)

    return force_closure * stability
```

### Learning-Based Grasp Planning

Use deep learning to predict grasps:

```python
# GraspNet / DexNet approach
def learned_grasp_prediction(rgbd_image):
    """Predict grasp from RGB-D image using neural network"""
    # Input: RGB-D image (H x W x 4)
    # Output: Grasp pose + quality score

    features = grasp_network(rgbd_image)
    grasp_pose = decode_grasp_pose(features)
    quality_score = features['quality']

    return grasp_pose, quality_score
```

**Datasets**: DexNet-4.0, GraspNet-1Billion, Cornell Grasp Dataset

## Pre-grasp and Post-grasp Planning

### Pre-grasp Motion

Approach object from safe distance:

```python
def plan_pregrasp_motion(current_pose, grasp_pose, obstacles):
    # 1. Compute pre-grasp pose (offset from grasp)
    pregrasp_pose = compute_pregrasp_pose(grasp_pose, offset=-0.1)  # 10cm before grasp

    # 2. Plan motion: current → pre-grasp
    path_to_pregrasp = rrt(current_pose, pregrasp_pose, obstacles)

    # 3. Straight-line approach: pre-grasp → grasp
    approach_traj = linear_approach(pregrasp_pose, grasp_pose)

    return path_to_pregrasp + approach_traj
```

### Post-grasp Motion

Retreat after grasping:

```python
def plan_postgrasp_motion(grasp_pose, retreat_direction, obstacles):
    # Retreat along specified direction (usually opposite of approach)
    retreat_pose = grasp_pose.translate(retreat_direction * retreat_distance)

    # Check retreat is collision-free
    retreat_traj = linear_retreat(grasp_pose, retreat_pose)

    if not collision_free(retreat_traj, obstacles):
        # Try alternative retreat directions
        retreat_traj = plan_alternative_retreat(grasp_pose, obstacles)

    return retreat_traj
```

## Contact-Constrained Planning

Planning while maintaining contact with objects or environment.

### Problem Formulation

**Constraints**:
- Contact must be maintained: d(q) = 0 (distance to surface)
- Contact forces must be in friction cone
- No slipping: Tangential force < μ * Normal force

### Contact-Aware Planning

```python
def plan_with_contact_constraint(start, goal, contact_surface):
    # Project configurations onto contact manifold
    def project_to_contact(q):
        """Project configuration to maintain contact"""
        # Compute distance to surface
        distance = signed_distance(forward_kinematics(q), contact_surface)

        # Adjust configuration to satisfy contact
        q_projected = q.copy()
        while abs(distance) > tolerance:
            # Gradient descent on contact constraint
            gradient = compute_contact_gradient(q_projected, contact_surface)
            q_projected -= learning_rate * gradient
            distance = signed_distance(forward_kinematics(q_projected), contact_surface)

        return q_projected

    # RRT with projection
    tree = Tree(project_to_contact(start))

    for i in range(max_iterations):
        q_rand = sample_config()
        q_near = tree.nearest(q_rand)
        q_new = steer(q_near, q_rand)

        # Project onto contact manifold
        q_new = project_to_contact(q_new)

        if collision_free(q_new) and valid_contact_forces(q_new):
            tree.add(q_new, parent=q_near)

        if reached(q_new, project_to_contact(goal)):
            return extract_path(tree, q_new)

    return None
```

## MoveIt Manipulation Planning

MoveIt provides manipulation-specific planning capabilities:

```python
import moveit_commander

# Initialize
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")

# Add object to planning scene
object_pose = PoseStamped()
object_pose.header.frame_id = "world"
object_pose.pose.position = Point(0.5, 0.0, 0.5)
scene.add_box("target_object", object_pose, size=(0.05, 0.05, 0.1))

# Plan grasp
grasp_pose = compute_grasp_pose("target_object")
arm_group.set_pose_target(grasp_pose)
plan = arm_group.go(wait=True)

# Execute grasp
gripper_group.set_named_target("closed")
gripper_group.go(wait=True)

# Attach object (affects collision checking)
scene.attach_box("arm_link", "target_object")

# Plan motion with attached object
place_pose = PoseStamped()
place_pose.pose.position = Point(0.3, 0.3, 0.5)
arm_group.set_pose_target(place_pose)
arm_group.go(wait=True)

# Release object
gripper_group.set_named_target("open")
gripper_group.go(wait=True)
scene.remove_attached_object("arm_link", "target_object")
```

## Pick and Place Planning

Complete pick-and-place sequence:

```python
def pick_and_place(object_name, pick_location, place_location):
    # 1. Plan approach
    pregrasp_pose = compute_pregrasp_pose(pick_location)
    plan_to_pregrasp = plan_motion(current_pose(), pregrasp_pose)
    execute(plan_to_pregrasp)

    # 2. Approach and grasp
    approach_traj = linear_motion(pregrasp_pose, pick_location)
    execute(approach_traj)
    close_gripper()
    attach_object(object_name)

    # 3. Retreat
    retreat_pose = compute_retreat_pose(pick_location)
    retreat_traj = linear_motion(pick_location, retreat_pose)
    execute(retreat_traj)

    # 4. Move to place pre-pose
    preplace_pose = compute_preplace_pose(place_location)
    plan_to_preplace = plan_motion(retreat_pose, preplace_pose)
    execute(plan_to_preplace)

    # 5. Place
    place_traj = linear_motion(preplace_pose, place_location)
    execute(place_traj)
    open_gripper()
    detach_object(object_name)

    # 6. Retreat
    retreat_traj = linear_motion(place_location, preplace_pose)
    execute(retreat_traj)
```

## Pushing and Non-Prehensile Manipulation

Manipulate objects without grasping:

### Push Planning

```python
def plan_push(object_pose, goal_pose, obstacles):
    # Quasi-static assumption: Object moves in direction of push

    # 1. Compute push direction
    push_direction = (goal_pose.position - object_pose.position).normalize()

    # 2. Compute contact point (push from behind)
    contact_point = object_pose.position - push_direction * object_radius

    # 3. Plan end-effector motion
    start_ee_pose = compute_ee_pose_for_contact(contact_point, push_direction)
    end_ee_pose = start_ee_pose.translate(push_direction * push_distance)

    # 4. Plan trajectory
    push_traj = plan_straight_line(start_ee_pose, end_ee_pose)

    return push_traj
```

### Physics-Based Push Prediction

```python
def predict_object_motion(object_state, push_force, push_point):
    """Predict object motion from push using physics"""
    # Compute object acceleration (Newton's second law)
    acceleration = push_force / object_mass

    # Compute angular acceleration (torque = r × F)
    r = push_point - object_center_of_mass
    torque = cross(r, push_force)
    angular_accel = torque / object_inertia

    # Integrate to get velocity and position
    # (simplified; use physics engine for accurate simulation)
    velocity = object_state.velocity + acceleration * dt
    angular_velocity = object_state.angular_vel + angular_accel * dt

    new_position = object_state.position + velocity * dt
    new_orientation = object_state.orientation + angular_velocity * dt

    return ObjectState(new_position, new_orientation, velocity, angular_velocity)
```

## Dual-Arm Manipulation

Planning for robots with two arms:

```python
def plan_dual_arm_manipulation(left_arm, right_arm, object_pose, goal_pose):
    # 1. Compute coordinated grasps
    left_grasp, right_grasp = compute_dual_arm_grasps(object_pose)

    # 2. Plan coordinated approach
    left_traj = plan_motion(left_arm.current_pose(), left_grasp)
    right_traj = plan_motion(right_arm.current_pose(), right_grasp)

    # Synchronize trajectories
    synchronized_traj = synchronize_trajectories(left_traj, right_traj)

    # 3. Execute coordinated grasp
    execute_synchronized(left_arm, right_arm, synchronized_traj)
    close_grippers(left_arm, right_arm)

    # 4. Coordinated transport
    transport_traj = plan_coordinated_motion(
        (left_grasp, right_grasp),
        (goal_left, goal_right)
    )
    execute_synchronized(left_arm, right_arm, transport_traj)

    # 5. Release
    open_grippers(left_arm, right_arm)
```

## Summary

Manipulation planning encompasses grasp planning, approach/retreat planning, contact-constrained planning, and object interaction. Key techniques include sampling-based grasp planning, contact-aware motion planning, and MoveIt integration for practical robotic manipulation.

Next: [Whole-Body Planning](whole-body-planning.md) - Learn planning for humanoid robots with full-body coordination.
