# Project Requirements

This document specifies the requirements for the capstone project, including functional requirements, technical specifications, constraints, and deliverables.

## Functional Requirements

### FR1: Natural Language Task Specification

**Requirement**: System must accept task instructions in natural language.

**Examples**:
- "Navigate to the kitchen and bring me a cup"
- "Pick up the red block and place it on the blue plate"
- "Clean the table by moving all objects to the bin"

**Acceptance Criteria**:
- [ ] Parse natural language instructions
- [ ] Extract task goals and constraints
- [ ] Handle ambiguous instructions (ask clarifying questions)
- [ ] Support at least 5 different task types

### FR2: Task Planning

**Requirement**: Decompose high-level tasks into executable action sequences.

**Acceptance Criteria**:
- [ ] Generate valid action sequences
- [ ] Handle task dependencies
- [ ] Detect infeasible tasks
- [ ] Support re-planning on failure

### FR3: Motion Planning

**Requirement**: Generate collision-free paths for navigation and manipulation.

**Acceptance Criteria**:
- [ ] Collision-free paths in cluttered environments
- [ ] Respect joint limits and kinematic constraints
- [ ] Smooth trajectories (no discontinuities)
- [ ] Planning success rate > 90%

### FR4: Object Manipulation

**Requirement**: Grasp and manipulate objects.

**Acceptance Criteria**:
- [ ] Grasp planning for multiple object types
- [ ] Pick-and-place execution
- [ ] Handle object variations (size, shape, pose)
- [ ] Grasp success rate > 80%

### FR5: Simulation Integration

**Requirement**: Test system in simulation before deployment.

**Acceptance Criteria**:
- [ ] Simulation environment setup (Isaac Sim, Gazebo, or PyBullet)
- [ ] Robot model accurately represents physical robot
- [ ] Sensor simulation (cameras, LiDAR, IMU)
- [ ] Physics simulation (contacts, friction)

## Technical Specifications

### TS1: Robot Platform

**Options**:
1. **Simulated Humanoid**: Atlas, Pepper, NAO, or custom URDF
2. **Physical Robot** (if available): Must have arms, grippers, cameras

**Requirements**:
- Minimum 6-DOF arms
- RGB-D camera or stereo vision
- Gripper or end-effector

### TS2: Software Stack

**Required**:
- ROS2 (recommended) or ROS1
- MoveIt for motion planning
- Simulation environment (Isaac Sim, Gazebo, or PyBullet)
- Python 3.8+

**Recommended**:
- OpenCV for vision processing
- PyTorch for VLA models
- Nav2 for navigation (if mobile base)

### TS3: VLA Model Integration

**Requirement**: Integrate a vision-language-action model for task understanding or control.

**Options**:
1. Use pre-trained model (RT-2, CLIP + LLM)
2. Fine-tune on custom dataset
3. Implement simplified VLA architecture

**Minimum Requirement**:
- Language understanding (instruction parsing)
- Visual grounding (object detection/recognition)
- Action generation (at least high-level actions)

### TS4: Planning Algorithms

**Requirements**:
- **Path Planning**: RRT, RRT*, or A*
- **Trajectory Optimization**: CHOMP, TrajOpt, or STOMP
- **Task Planning**: PDDL planner, HTN, or LLM-based

**At least 2 of 3 categories must be implemented**.

## Performance Requirements

### PR1: Real-Time Performance

- Planning latency: < 5 seconds for complex tasks
- Control loop: 10-30 Hz
- VLA model inference: < 500ms per decision

### PR2: Success Rate

- Overall task completion: > 70%
- Motion planning: > 90% success
- Grasp execution: > 80% success
- Navigation (if applicable): > 85% success

### PR3: Robustness

- Handle minor object pose variations (±5cm, ±15°)
- Recover from single action failures
- Gracefully degrade on sensor noise

## Project Scope

### In Scope

- Task planning and decomposition
- Motion planning for manipulation
- VLA model integration (basic level acceptable)
- Simulation-based testing
- At least 3 distinct tasks (e.g., pick-place, navigation, cleaning)
- Documentation and presentation

### Out of Scope (Unless Explicitly Chosen)

- Real hardware deployment (simulation is sufficient)
- Learning from demonstrations (unless part of your approach)
- Multi-robot coordination
- Long-term autonomy (> 30 minutes)
- Advanced computer vision (object detection APIs acceptable)

## Deliverables

### D1: Source Code

**Requirements**:
- Well-organized repository (GitHub/GitLab)
- README with setup instructions
- Requirements.txt or environment.yml
- Clear code structure and comments
- Version control history

**Structure**:
```
capstone-project/
├── README.md
├── requirements.txt
├── src/
│   ├── planning/
│   ├── control/
│   ├── perception/
│   └── integration/
├── config/
├── launch/
├── tests/
├── docs/
└── videos/
```

### D2: Technical Report

**Requirements**:
- 10-15 pages (excluding appendices)
- Sections: Introduction, System Architecture, Implementation, Results, Discussion, Conclusion
- Figures and diagrams
- References to course material and external sources

**Template**: LaTeX or Markdown

### D3: Demonstration Video

**Requirements**:
- 3-5 minutes
- Show system executing at least 3 tasks
- Include narration explaining what's happening
- Show both successes and handling of failures
- High-quality recording (720p minimum)

### D4: Presentation

**Requirements**:
- 15-20 minute presentation
- Slides covering: motivation, approach, results, challenges, future work
- Live demonstration or video
- Q&A readiness

### D5: Test Results

**Requirements**:
- Quantitative metrics (success rates, timing, etc.)
- Qualitative analysis (failure modes, limitations)
- At least 10 test runs per task
- Statistical analysis (mean, std dev, confidence intervals)

## Evaluation Criteria

Projects will be evaluated on:

1. **Technical Complexity** (30%): Difficulty and sophistication of implementation
2. **Integration** (25%): How well modules are integrated
3. **Performance** (20%): Success rates and robustness
4. **Innovation** (10%): Novel approaches or creative solutions
5. **Documentation** (10%): Code quality, report, and presentation
6. **Demonstration** (5%): Quality of video and live demo

See [Grading Rubric](grading-rubric.md) for detailed breakdown.

## Constraints

### Time Constraints

- Total project duration: 7-11 weeks
- Intermediate milestones (see [Implementation Phases](implementation-phases.md))
- Final submission deadline: [To be specified by instructor]

### Resource Constraints

- Computational: Laptop/workstation with GPU (recommended but not required)
- Simulation: Free tools (Isaac Sim, Gazebo, PyBullet)
- Hardware: Optional (simulation sufficient)

### Complexity Constraints

- Focus on breadth (integration) over depth (algorithmic novelty)
- Use existing libraries and frameworks (don't reimplement everything)
- Minimum viable product first, enhancements later

## Optional Extensions

For students seeking additional challenge:

- **Sim-to-Real Transfer**: Deploy on physical robot
- **Learning-Based Components**: Train VLA model on custom data
- **Advanced Planning**: Implement novel planning algorithm
- **Multi-Modal Interaction**: Add speech recognition, gesture control
- **Long-Horizon Tasks**: Tasks requiring > 10 actions
- **Human-Robot Collaboration**: Tasks involving human in the loop

## Support and Resources

- **Code Examples**: Course repository with starter code
- **Documentation**: Links to ROS, MoveIt, Isaac Sim docs
- **Office Hours**: Weekly sessions with instructor/TAs
- **Peer Review**: Mid-project peer feedback sessions

## Getting Started Checklist

- [ ] Read all capstone documentation
- [ ] Choose robot platform (simulated or physical)
- [ ] Select 3-5 tasks to implement
- [ ] Set up development environment
- [ ] Create project repository
- [ ] Review [Integration Guide](integration-guide.md)
- [ ] Begin [Implementation Phase 1](implementation-phases.md)

Next: [Integration Guide](integration-guide.md) - Learn how to integrate all course concepts.
