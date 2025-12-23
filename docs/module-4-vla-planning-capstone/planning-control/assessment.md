# Chapter 2 Assessment: Motion Planning and Control

Test your understanding of path planning algorithms, trajectory optimization, task planning, reactive planning, manipulation planning, and whole-body planning.

## Path Planning Questions

###Question 1: Algorithm Selection

**Q**: You're planning for a 7-DOF robot arm in a cluttered environment. Compare A* and RRT* for this problem. Which would you choose and why?

<details>
<summary>Answer</summary>

**A* Analysis**:
- Requires discretized configuration space
- For 7-DOF arm with 10 samples per joint: 10^7 = 10 million states
- Memory and computation intractable

**RRT* Analysis**:
- Samples configuration space incrementally
- Handles high dimensions well
- Probabilistically complete and asymptotically optimal
- Typical solution in seconds for 7-DOF

**Choice**: RRT* for high-dimensional arm planning. A* only feasible for 2-3 DOF problems.

</details>

### Question 2: Heuristic Design

**Q**: Design an admissible heuristic for A* path planning for a mobile robot navigating in a 2D grid with obstacles.

<details>
<summary>Answer</summary>

**Euclidean Distance** (L2 norm):
```python
h(n, goal) = sqrt((n.x - goal.x)² + (n.y - goal.y)²)
```

**Admissibility**: Euclidean distance is the straight-line distance, which is always ≤ actual path length (since paths must go around obstacles).

**Alternative: Manhattan Distance** (L1 norm):
```python
h(n, goal) = |n.x - goal.x| + |n.y - goal.y|
```
Also admissible, but less tight than Euclidean (underestimates more).

**Consistency**: Both heuristics are consistent, ensuring A* efficiency.

</details>

## Trajectory Optimization Questions

### Question 3: CHOMP vs TrajOpt

**Q**: When would you prefer CHOMP over TrajOpt for trajectory optimization?

<details>
<summary>Answer</summary>

**Prefer CHOMP when**:
- Speed is critical (CHOMP is faster)
- Smooth trajectories are primary goal
- Good initial trajectory available (from RRT)
- Narrow passages in environment (CHOMP handles well)

**Prefer TrajOpt when**:
- Reliability is critical (TrajOpt more robust)
- Complex constraints (joint limits, dynamics)
- Need convergence guarantees
- Production systems (MoveIt default)

**Key Difference**: CHOMP uses gradient descent (fast, local minima). TrajOpt uses sequential convex optimization (slower, more reliable).

</details>

### Question 4: Time-Optimal Trajectory

**Q**: Explain the two-phase approach for time-optimal trajectory generation along a given path.

<details>
<summary>Answer</summary>

**Phase 1: Forward Pass** (Accelerate):
- Start from rest
- Accelerate as much as possible while respecting:
  - Velocity limits
  - Acceleration limits
  - Curvature constraints

**Phase 2: Backward Pass** (Decelerate):
- Start from goal (rest)
- Work backwards
- Decelerate as needed to reach each point at feasible velocity

**Final Velocity Profile**:
```python
v(s) = min(v_forward(s), v_backward(s))
```

This ensures we can stop at the goal while maximizing speed throughout.

</details>

## Task Planning Questions

### Question 5: PDDL Planning

**Q**: Write PDDL action for a robot opening a door.

<details>
<summary>Answer</summary>

```lisp
(:action open-door
  :parameters (?r - robot ?d - door ?loc1 - location ?loc2 - location)
  :precondition (and
    (at ?r ?loc1)           ; Robot at door location
    (connects ?d ?loc1 ?loc2) ; Door connects locations
    (closed ?d)              ; Door is closed
    (unlocked ?d)            ; Door is unlocked
    (empty-hand ?r)          ; Hand free to turn handle
  )
  :effect (and
    (open ?d)                ; Door becomes open
    (not (closed ?d))        ; Door no longer closed
    (can-traverse ?loc1 ?loc2) ; Can now move between locations
  )
)
```

</details>

### Question 6: HTN vs Classical Planning

**Q**: Give an example where HTN planning is superior to classical PDDL planning.

<details>
<summary>Answer</summary>

**Example: "Make Breakfast"**

**Classical PDDL**: Would search through action space
- grasp(pan), place(pan, stove), grasp(egg), crack(egg), ...
- Huge search space, no procedural knowledge

**HTN**: Encodes recipe knowledge
```python
task_make_breakfast = [
    "make_eggs",   # Compound task
    "make_toast",   # Compound task
    "serve"         # Primitive task
]

task_make_eggs = [
    "get_pan",
    "heat_stove",
    "crack_eggs",
    "cook",
    "plate_eggs"
]
```

**Why HTN is Better**:
- Encodes domain knowledge (recipes)
- Prunes infeasible action sequences
- Much faster planning
- Natural for procedural tasks

</details>

## Reactive Planning Questions

### Question 7: Dynamic Window Approach

**Q**: In DWA, why is the "dynamic window" necessary? What happens without it?

<details>
<summary>Answer</summary>

**Dynamic Window**: Set of velocities reachable given current velocity and acceleration limits.

**Without dynamic window**:
- Robot could command instantaneous velocity changes
- Violates physical constraints (motors have limits)
- Infeasible or dangerous motions

**With dynamic window**:
```
v_min = current_v - a_max * dt
v_max = current_v + a_max * dt
```
Only considers physically reachable velocities, ensuring smooth, feasible motion.

</details>

### Question 8: Local Minima in Potential Fields

**Q**: Describe a scenario where potential field planning gets stuck in a local minimum and propose a solution.

<details>
<summary>Answer</summary>

**Scenario: U-shaped Obstacle**
```
Goal: G
     ___
    |   |
    | R |  ← Robot stuck here
    |___|

Start: S
```

Robot enters U-shape, attractive force toward goal cancels repulsive forces from walls. Robot gets stuck.

**Solutions**:

1. **Random Walk**: Add random noise to escape
2. **Gradient Descent with Momentum**: Carry velocity across flat regions
3. **Hybrid Approach**: Use potential fields locally, global planner (A*, RRT) for overall path
4. **Navigation Function**: Carefully designed potential with guaranteed convergence

</details>

## Manipulation Planning Questions

### Question 9: Grasp Quality

**Q**: What makes a grasp "force-closure"? Why is it important?

<details>
<summary>Answer</summary>

**Force Closure**: Grasp can resist arbitrary external forces and torques.

**Requirements**:
- Sufficient contact points (≥4 for 3D force closure with point contacts)
- Contact normals span the wrench space
- Friction at contacts

**Example** (2D):
```
Two-finger parallel jaw grasp:
  |  ← Finger
  O  ← Object
  |  ← Finger
```
Force closure if fingers on opposite sides and friction sufficient.

**Importance**:
- Ensures object won't slip or fall
- Critical for robust manipulation
- Required for dynamic tasks (shaking, throwing)

</details>

### Question 10: Pre-grasp Planning

**Q**: Why do manipulation planners typically use a pre-grasp pose instead of planning directly to the grasp?

<details>
<summary>Answer</summary>

**Reasons for Pre-grasp**:

1. **Approach Direction**: Final approach along specific direction (usually along gripper axis)

2. **Collision Avoidance**: Direct planning may generate paths that cause collisions during final approach

3. **Grasp Execution Reliability**: Straight-line approach from known pre-grasp is more predictable

4. **Separation of Concerns**:
   - Free-space planning: Start → Pre-grasp
   - Constrained motion: Pre-grasp → Grasp (straight line)

**Example**:
```
Free RRT planning → Pre-grasp (10cm before object)
    ↓
Linear approach → Grasp
```

</details>

## Whole-Body Planning Questions

### Question 11: Balance Constraints

**Q**: Explain the Zero Moment Point (ZMP) criterion for humanoid balance.

<details>
<summary>Answer</summary>

**ZMP**: Point on the ground where net moment from gravity and inertial forces is zero.

**Static Stability**: ZMP must be inside support polygon (convex hull of contact points).

**Dynamic Stability**: ZMP must remain inside support polygon during motion.

**Computation**:
```python
# Simplified (flat ground)
zmp_x = com_x - (com_z / (com_accel_z + g)) * com_accel_x
zmp_y = com_y - (com_z / (com_accel_z + g)) * com_accel_y
```

**Why Important**: If ZMP leaves support polygon, robot will tip over.

**Application**: Walking controllers maintain ZMP within foot support.

</details>

### Question 12: Hierarchical IK

**Q**: In hierarchical IK for humanoids, what tasks should have highest priority and why?

<details>
<summary>Answer</summary>

**Priority Ordering**:

1. **Balance/Stability** (Highest)
   - CoM must stay over support
   - Robot must not fall
   - Safety-critical

2. **Contact Constraints**
   - Maintain foot/hand contacts
   - Required for task execution

3. **Primary Task**
   - End-effector goal (e.g., reach target)
   - Main objective

4. **Secondary Tasks**
   - Joint limit avoidance
   - Singularity avoidance

5. **Posture** (Lowest)
   - Comfort, aesthetics
   - Sacrificed if conflicts with higher priorities

**Reasoning**: Higher priorities are non-negotiable (falling is unacceptable), lower priorities are "nice to have".

</details>

## Integrated Problem

### Problem 1: Complete Planning Pipeline

**Scenario**: A humanoid robot must retrieve a cup from a high shelf.

**Q**: Design a complete planning pipeline integrating task planning, motion planning, and manipulation planning.

<details>
<summary>Sample Answer</summary>

**1. Task Planning** (High-level):
```
1. navigate_to(shelf)
2. reach(cup)
3. grasp(cup)
4. retract(arm)
5. navigate_to(user)
6. release(cup)
```

**2. Motion Planning** (Per task):

**navigate_to(shelf)**:
- Footstep planner: Plan footsteps avoiding obstacles
- Walking motion generator: Generate full-body walking motion

**reach(cup)**:
- Whole-body IK: Solve for configuration where hand reaches cup
- Consider balance: May need to shift weight, adjust stance
- Trajectory optimization: Smooth path to reach configuration

**grasp(cup)**:
- Grasp planner: Compute grasp pose for cup
- Pre-grasp approach: Plan straight-line approach
- Close gripper

**retract(arm)**:
- Whole-body planning: Retract while maintaining balance
- Avoid collisions with shelf

**3. Execution**:
```python
def execute_cup_retrieval():
    # Task 1: Navigate
    footsteps = plan_footsteps(current_pos, shelf_pos)
    walking_motion = generate_walking(footsteps)
    execute(walking_motion)

    # Task 2: Reach
    reach_config = whole_body_ik(hand_goal=cup_pos, maintain_balance=True)
    reach_trajectory = trajectory_optimizer(current_config, reach_config)
    execute(reach_trajectory)

    # Task 3: Grasp
    grasp_pose = plan_grasp(cup)
    approach = linear_approach(pregrasp, grasp_pose)
    execute(approach)
    close_gripper()

    # Task 4: Retract
    retract_config = whole_body_ik(hand_goal=retract_pos, maintain_balance=True)
    execute(trajectory_optimizer(current_config, retract_config))

    # Task 5: Navigate back
    footsteps = plan_footsteps(current_pos, user_pos)
    execute(generate_walking(footsteps))

    # Task 6: Release
    open_gripper()
```

**Key Integration Points**:
- Task planner provides high-level action sequence
- Motion planner handles geometry and collision avoidance
- Manipulation planner computes grasps
- Whole-body planner ensures balance throughout
- Trajectory optimizer ensures smooth execution

</details>

## Summary

This assessment covered all major aspects of motion planning and control:
- Path planning (A*, RRT, RRT*)
- Trajectory optimization (CHOMP, TrajOpt)
- Task planning (PDDL, HTN)
- Reactive planning (DWA, potential fields, MPC)
- Manipulation planning (grasps, pre-grasp, contact)
- Whole-body planning (balance, hierarchical IK)

If you can answer these questions, you're ready for the capstone project.

**Next**: Proceed to [Chapter 3: Capstone Project](../capstone/index.md) to integrate all concepts.
