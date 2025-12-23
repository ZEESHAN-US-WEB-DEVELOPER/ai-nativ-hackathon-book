# Integration Guide

This guide explains how to integrate concepts from all modules into a cohesive capstone system, providing system architecture, integration patterns, and code examples.

## System Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    User Interface Layer                       │
│              (Natural Language Commands, GUI)                 │
└────────────────────┬──────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│                  VLA Model Layer                             │
│        (Language Understanding, Visual Grounding)            │
└────────────────────┬──────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│                  Task Planning Layer                         │
│           (PDDL Planner, HTN, or LLM-based)                  │
└────────────────────┬──────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│               Motion Planning Layer                          │
│      (RRT*, A*, Trajectory Optimization, MoveIt)             │
└────────────────────┬──────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│                   Control Layer                              │
│         (Low-level Controllers, ROS Controllers)             │
└────────────────────┬──────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│              Simulation/Robot Layer                          │
│           (Isaac Sim, Gazebo, or Physical Robot)             │
└─────────────────────────────────────────────────────────────┘
```

## Module Integration

### Module 1: Digital Twin Integration

**What to integrate**:
- Simulation environment (Gazebo, Isaac Sim, Unity)
- Robot URDF model
- Physics simulation
- Sensor simulation (cameras, LiDAR)

**Integration Points**:
```python
# Initialize simulation
import pybullet as p
p.connect(p.GUI)
robot_id = p.loadURDF("humanoid.urdf")

# Or with Isaac Sim
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
from omni.isaac.core import World
world = World()
robot = world.scene.add(HumanoidRobot())
```

### Module 2: Navigation Integration

**What to integrate**:
- Nav2 for mobile base navigation
- Isaac ROS for perception
- Costmaps and path planning

**Integration Points**:
```python
# Nav2 integration
from nav2_simple_commander.robot_navigator import BasicNavigator

navigator = BasicNavigator()
navigator.waitUntilNav2Active()

goal_pose = PoseStamped()
goal_pose.pose.position.x = 5.0
navigator.goToPose(goal_pose)
```

### Module 3: Core Robotics Integration

**What to integrate**:
- Kinematics and dynamics
- Sensor processing
- Control loops

### Module 4 Chapter 1: VLA Integration

**What to integrate**:
- Language model for instruction parsing
- Vision model for object detection
- Action generation

**Integration Pattern**:
```python
# VLA Controller
class VLAController:
    def __init__(self):
        self.language_model = load_language_model()
        self.vision_model = load_vision_model()
        self.action_model = load_action_model()

    def process_instruction(self, text_instruction, camera_image):
        # Language understanding
        task_representation = self.language_model.encode(text_instruction)

        # Visual grounding
        detected_objects = self.vision_model.detect(camera_image)

        # Action generation
        action_sequence = self.action_model.predict(
            task_representation,
            detected_objects
        )

        return action_sequence
```

### Module 4 Chapter 2: Planning Integration

**What to integrate**:
- Task planner (high-level)
- Motion planner (geometric)
- Trajectory optimizer (smooth paths)

**Integration Pattern**:
```python
# Hierarchical Planning System
class PlanningSystem:
    def __init__(self):
        self.task_planner = PDDLPlanner()
        self.motion_planner = RRTStarPlanner()
        self.trajectory_optimizer = CHOMPOptimizer()

    def plan(self, instruction, current_state, goal):
        # High-level task planning
        task_plan = self.task_planner.plan(current_state, goal)

        # For each high-level action, plan motion
        motion_plan = []
        for action in task_plan:
            # Motion planning
            path = self.motion_planner.plan(
                action.start_config,
                action.goal_config
            )

            # Trajectory optimization
            trajectory = self.trajectory_optimizer.optimize(path)

            motion_plan.append(trajectory)

        return motion_plan
```

## ROS Integration

### ROS Node Structure

```
capstone_system/
├── vla_controller_node
│   ├── Subscribes: /camera/image, /task_instruction
│   └── Publishes: /high_level_actions
├── task_planner_node
│   ├── Subscribes: /high_level_actions
│   └── Publishes: /action_sequence
├── motion_planner_node
│   ├── Subscribes: /action_sequence
│   └── Publishes: /planned_trajectory
└── controller_node
    ├── Subscribes: /planned_trajectory
    └── Publishes: /joint_commands
```

### Launch File

```xml
<launch>
  <!-- Simulation -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find capstone)/worlds/environment.world"/>
  </include>

  <!-- Robot -->
  <param name="robot_description" command="$(find xacro)/xacro $(find capstone)/urdf/humanoid.urdf.xacro"/>
  <node name="spawn_robot" pkg="gazebo_ros" type="spawn_model"
        args="-urdf -param robot_description -model humanoid"/>

  <!-- MoveIt -->
  <include file="$(find humanoid_moveit_config)/launch/move_group.launch"/>

  <!-- Capstone Nodes -->
  <node name="vla_controller" pkg="capstone" type="vla_controller_node.py"/>
  <node name="task_planner" pkg="capstone" type="task_planner_node.py"/>
  <node name="motion_planner" pkg="capstone" type="motion_planner_node.py"/>

  <!-- RViz Visualization -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find capstone)/config/capstone.rviz"/>
</launch>
```

## Communication Patterns

### Synchronous Planning

```python
class SynchronousPlanner:
    def execute_task(self, instruction):
        # Step 1: VLA understanding
        high_level_actions = self.vla_controller.process(instruction)

        # Step 2: Task planning
        action_sequence = self.task_planner.plan(high_level_actions)

        # Step 3: Motion planning for each action
        for action in action_sequence:
            trajectory = self.motion_planner.plan(action)

            # Step 4: Execute
            self.controller.execute(trajectory)

            # Step 5: Check success
            if not self.check_success(action):
                self.replan(action)
```

### Asynchronous Pipeline

```python
class AsynchronousPipeline:
    def __init__(self):
        self.action_queue = queue.Queue()
        self.trajectory_queue = queue.Queue()

        # Start background threads
        threading.Thread(target=self.planning_loop, daemon=True).start()
        threading.Thread(target=self.execution_loop, daemon=True).start()

    def planning_loop(self):
        while True:
            action = self.action_queue.get()
            trajectory = self.motion_planner.plan(action)
            self.trajectory_queue.put(trajectory)

    def execution_loop(self):
        while True:
            trajectory = self.trajectory_queue.get()
            self.controller.execute(trajectory)
```

## Data Flow

### Perception Pipeline

```
Camera → Image Processing → Object Detection → World Model Update
  ↓
RGB-D → Point Cloud → Segmentation → Object Poses
  ↓
Depth → Obstacle Map → Collision Checking
```

### Control Pipeline

```
Instruction → VLA → Task Plan → Motion Plan → Trajectory → Joint Commands → Robot
                                              ↑
                                         Feedback (Joint States, Camera)
```

## Common Integration Challenges

### Challenge 1: Coordinate Frame Transforms

**Problem**: Different modules use different coordinate frames

**Solution**: Use tf2 for transforms
```python
import tf2_ros
import tf2_geometry_msgs

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Transform pose from camera frame to robot base frame
transform = tf_buffer.lookup_transform('base_link', 'camera_link', rospy.Time(0))
pose_in_base = tf2_geometry_msgs.do_transform_pose(pose_in_camera, transform)
```

### Challenge 2: Timing and Synchronization

**Problem**: VLA model slow, control loop must be fast

**Solution**: Hierarchical control with different update rates
```python
# VLA runs at 1 Hz (slow)
rospy.Timer(rospy.Duration(1.0), vla_callback)

# Motion planner runs at 5 Hz (medium)
rospy.Timer(rospy.Duration(0.2), motion_planner_callback)

# Controller runs at 50 Hz (fast)
rospy.Timer(rospy.Duration(0.02), controller_callback)
```

### Challenge 3: State Management

**Problem**: Multiple modules need consistent world state

**Solution**: Centralized world model
```python
class WorldModel:
    def __init__(self):
        self.objects = {}
        self.robot_state = None
        self.lock = threading.Lock()

    def update_object(self, obj_id, pose):
        with self.lock:
            self.objects[obj_id] = pose

    def get_objects(self):
        with self.lock:
            return copy.deepcopy(self.objects)
```

## Testing Integration

### Unit Tests

Test each component independently:
```python
def test_vla_controller():
    controller = VLAController()
    instruction = "pick up the red cup"
    image = load_test_image()

    actions = controller.process_instruction(instruction, image)

    assert len(actions) > 0
    assert "grasp" in actions
```

### Integration Tests

Test component interactions:
```python
def test_planning_pipeline():
    planner = PlanningSystem()
    instruction = "move the cup to the table"

    plan = planner.plan(instruction, current_state, goal_state)

    assert plan is not None
    assert len(plan) > 0
    assert all(is_collision_free(p) for p in plan)
```

### End-to-End Tests

Test complete system:
```python
def test_complete_task():
    system = CapstoneSystem()
    instruction = "navigate to the kitchen and pick up the cup"

    success = system.execute_task(instruction)

    assert success == True
    assert system.robot_holding_object("cup")
```

## Best Practices

1. **Modularity**: Keep components loosely coupled
2. **Error Handling**: Gracefully handle failures at each level
3. **Logging**: Comprehensive logging for debugging
4. **Visualization**: Use RViz for real-time visualization
5. **Configuration**: Externalize parameters (YAML files)
6. **Version Control**: Commit frequently with clear messages

## Next Steps

1. Set up development environment
2. Implement each module independently
3. Integration test incrementally (2 modules at a time)
4. End-to-end integration
5. System testing and debugging

Next: [Implementation Phases](implementation-phases.md) - Step-by-step implementation guide.
