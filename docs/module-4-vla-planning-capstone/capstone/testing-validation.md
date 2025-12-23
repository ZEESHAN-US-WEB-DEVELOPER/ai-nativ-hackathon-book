# Testing and Validation

Comprehensive testing ensures your capstone project works correctly. This guide covers unit testing, integration testing, end-to-end validation, and performance evaluation.

## Testing Strategy

### Testing Pyramid

```
        /\
       /E2E\        End-to-End Tests (Few, Slow, High-level)
      /------\
     /Integration\  Integration Tests (Some, Medium)
    /------------\
   /  Unit Tests  \ Unit Tests (Many, Fast, Focused)
  /----------------\
```

## Unit Testing

Test individual components in isolation.

### Example: Motion Planner Test

```python
import unittest
from motion_planner import RRTStarPlanner

class TestMotionPlanner(unittest.TestCase):
    def setUp(self):
        self.planner = RRTStarPlanner()
        self.start = [0, 0, 0, 0, 0, 0, 0]  # 7-DOF configuration
        self.goal = [1, 0.5, 0.3, 0, 0, 0, 0]

    def test_finds_path(self):
        path = self.planner.plan(self.start, self.goal)
        self.assertIsNotNone(path)
        self.assertTrue(len(path) > 2)

    def test_path_collision_free(self):
        path = self.planner.plan(self.start, self.goal)
        for config in path:
            self.assertTrue(self.planner.is_collision_free(config))

    def test_path_connects_start_goal(self):
        path = self.planner.plan(self.start, self.goal)
        self.assertAlmostEqual(path[0], self.start)
        self.assertAlmostEqual(path[-1], self.goal, delta=0.1)
```

### Example: VLA Controller Test

```python
class TestVLAController(unittest.TestCase):
    def setUp(self):
        self.controller = VLAController()
        self.test_image = load_test_image("kitchen.jpg")

    def test_instruction_parsing(self):
        instruction = "pick up the red cup"
        parsed = self.controller.parse_instruction(instruction)

        self.assertEqual(parsed['action'], 'pick')
        self.assertEqual(parsed['object'], 'cup')
        self.assertEqual(parsed['property'], 'red')

    def test_object_detection(self):
        objects = self.controller.detect_objects(self.test_image)
        self.assertGreater(len(objects), 0)
        self.assertIn('cup', [obj.label for obj in objects])
```

## Integration Testing

Test interactions between components.

### Example: Planning Pipeline Test

```python
class TestPlanningPipeline(unittest.TestCase):
    def setUp(self):
        self.vla = VLAController()
        self.task_planner = TaskPlanner()
        self.motion_planner = MotionPlanner()

    def test_instruction_to_plan(self):
        instruction = "move the cup to the table"
        image = load_test_image()

        # VLA extracts task
        task = self.vla.process_instruction(instruction, image)

        # Task planner generates action sequence
        actions = self.task_planner.plan(task)
        self.assertGreater(len(actions), 0)

        # Motion planner generates trajectories
        for action in actions:
            trajectory = self.motion_planner.plan(action)
            self.assertIsNotNone(trajectory)
```

## End-to-End Testing

Test complete system execution.

### Test Scenario Structure

```python
class EndToEndTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Start simulation
        cls.sim = Simulation()
        cls.robot = cls.sim.load_robot("humanoid.urdf")
        cls.system = CapstoneSystem(cls.robot)

    def test_pick_and_place(self):
        # Setup: Place cup on table
        cup_id = self.sim.spawn_object("cup.urdf", position=[0.5, 0, 0.8])

        # Execute task
        instruction = "pick up the cup and place it at x=0.3, y=0.3"
        success = self.system.execute_task(instruction)

        # Verify
        self.assertTrue(success)
        cup_pose = self.sim.get_object_pose(cup_id)
        self.assertAlmostEqual(cup_pose.position.x, 0.3, delta=0.05)
        self.assertAlmostEqual(cup_pose.position.y, 0.3, delta=0.05)
```

## Performance Testing

Measure system performance against requirements.

### Metrics Collection

```python
class PerformanceMetrics:
    def __init__(self):
        self.trials = []

    def record_trial(self, task_name, result):
        self.trials.append({
            'task': task_name,
            'success': result['success'],
            'planning_time': result['planning_time'],
            'execution_time': result['execution_time'],
            'total_time': result['total_time'],
            'num_replans': result['replans']
        })

    def compute_statistics(self):
        df = pd.DataFrame(self.trials)

        stats = {
            'success_rate': df['success'].mean(),
            'avg_planning_time': df['planning_time'].mean(),
            'std_planning_time': df['planning_time'].std(),
            'avg_execution_time': df['execution_time'].mean(),
            'avg_total_time': df['total_time'].mean(),
            'avg_replans': df['num_replans'].mean()
        }

        return stats
```

### Performance Test Example

```python
def test_performance():
    metrics = PerformanceMetrics()

    for trial in range(10):
        # Randomize initial conditions
        start_config = randomize_config()
        object_pose = randomize_pose()

        # Execute task
        start_time = time.time()
        result = system.execute_task("pick up the object")
        total_time = time.time() - start_time

        metrics.record_trial("pick_and_place", {
            'success': result.success,
            'planning_time': result.planning_time,
            'execution_time': result.execution_time,
            'total_time': total_time,
            'replans': result.num_replans
        })

    stats = metrics.compute_statistics()

    # Assert performance requirements
    assert stats['success_rate'] >= 0.8  # 80% success rate
    assert stats['avg_total_time'] < 30.0  # < 30 seconds per task
```

## Robustness Testing

Test system under variations and perturbations.

### Object Pose Variation

```python
def test_pose_robustness():
    success_count = 0
    num_trials = 20

    for trial in range(num_trials):
        # Vary object pose
        x = 0.5 + random.uniform(-0.1, 0.1)  # ±10cm
        y = 0.0 + random.uniform(-0.1, 0.1)
        theta = random.uniform(-15, 15)  # ±15 degrees

        result = system.execute_grasp(position=[x, y, 0.8], orientation=theta)

        if result.success:
            success_count += 1

    success_rate = success_count / num_trials
    assert success_rate >= 0.7  # 70% success under variation
```

### Sensor Noise

```python
def test_sensor_noise():
    # Add noise to camera image
    clean_image = get_camera_image()
    noisy_image = add_gaussian_noise(clean_image, std=0.1)

    # System should still detect objects
    objects = system.detect_objects(noisy_image)
    assert len(objects) > 0
```

## Failure Case Testing

Test system behavior on failures.

### Grasp Failure Recovery

```python
def test_grasp_failure_recovery():
    # Force grasp to fail
    system.gripper.set_failure_mode(True)

    instruction = "pick up the cup"
    result = system.execute_task(instruction)

    # System should detect failure and replan
    assert result.replanned == True
    # Should try alternative grasp or ask for help
```

## Validation Checklist

### Functional Requirements

- [ ] Language instructions parsed correctly
- [ ] Objects detected from camera
- [ ] Task plans generated
- [ ] Motion plans collision-free
- [ ] Trajectories smooth and executable
- [ ] Tasks complete successfully

### Performance Requirements

- [ ] Planning time < 5 seconds
- [ ] Control loop rate ≥ 10 Hz
- [ ] Success rate > 70%
- [ ] Handles minor variations (±5cm, ±15°)

### Integration Requirements

- [ ] All ROS nodes communicate correctly
- [ ] Coordinate transforms correct
- [ ] Timing and synchronization working
- [ ] State management consistent

## Test Automation

### Continuous Integration

```yaml
# .github/workflows/test.yml
name: Run Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Set up Python
        uses: actions/setup-python@v2
        with:
          python-version: 3.8
      - name: Install dependencies
        run: |
          pip install -r requirements.txt
      - name: Run unit tests
        run: |
          pytest tests/unit/
      - name: Run integration tests
        run: |
          pytest tests/integration/
```

## Logging and Debugging

### Comprehensive Logging

```python
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('capstone.log'),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger(__name__)

def execute_task(instruction):
    logger.info(f"Executing task: {instruction}")

    try:
        plan = planner.plan(instruction)
        logger.debug(f"Generated plan with {len(plan)} actions")

        for i, action in enumerate(plan):
            logger.info(f"Executing action {i+1}/{len(plan)}: {action}")
            result = execute_action(action)

            if not result.success:
                logger.warning(f"Action {i+1} failed, replanning...")

    except Exception as e:
        logger.error(f"Task execution failed: {e}", exc_info=True)
```

## Visualization for Debugging

### RViz Markers

```python
from visualization_msgs.msg import Marker

def visualize_planned_path(path):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = Marker.LINE_STRIP
    marker.scale.x = 0.01

    for config in path:
        point = forward_kinematics(config)
        marker.points.append(point)

    marker_pub.publish(marker)
```

## Summary

Comprehensive testing includes:
- **Unit tests**: Individual components
- **Integration tests**: Component interactions
- **End-to-end tests**: Complete system
- **Performance tests**: Metrics and benchmarks
- **Robustness tests**: Variations and noise
- **Failure tests**: Error handling

Aim for:
- 80%+ code coverage (unit tests)
- 10+ end-to-end test scenarios
- Quantitative performance data

Next: [Presentation Guidelines](presentation-guidelines.md) - Learn how to present your work.
