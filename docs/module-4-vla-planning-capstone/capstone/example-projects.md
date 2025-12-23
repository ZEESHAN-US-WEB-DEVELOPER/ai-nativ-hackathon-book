# Example Capstone Projects

This section provides concrete examples of capstone projects with varying complexity levels, from basic manipulation to advanced multi-task scenarios.

## Project 1: Kitchen Assistant Robot (Medium Complexity)

### Overview
Humanoid robot assists in kitchen by fetching objects and placing them on countertop.

### Tasks
1. Navigate to refrigerator
2. Open refrigerator door
3. Identify and grasp specific item (e.g., "milk bottle")
4. Close refrigerator door
5. Navigate to counter
6. Place item on counter

### Technical Approach
- **VLA**: CLIP for object recognition, GPT for instruction parsing
- **Planning**: PDDL for task planning, RRT* for motion planning
- **Control**: MoveIt for manipulation, Nav2 for navigation
- **Simulation**: Gazebo with custom kitchen environment

### Key Challenges
- Door opening requires contact planning
- Object identification in cluttered refrigerator
- Stable grasp of different object shapes (bottles, boxes)

### Expected Results
- Success rate: 75-85%
- Task completion time: 30-45 seconds
- Demonstrates: Navigation, manipulation, contact interaction

## Project 2: Table Clearing Robot (Basic-Medium)

### Overview
Robot clears dining table by moving objects to designated locations.

### Tasks
1. Identify objects on table
2. For each object:
   - Plan grasp
   - Pick up object
   - Navigate to bin/dishwasher
   - Place object
3. Repeat until table clear

### Technical Approach
- **VLA**: Object detection (YOLO or Mask R-CNN)
- **Planning**: HTN planning for task decomposition, CHOMP for trajectory optimization
- **Control**: Impedance control for gentle object handling
- **Simulation**: Isaac Sim with photorealistic table scene

### Key Challenges
- Multiple objects with different properties
- Avoid collisions with remaining objects
- Handle fragile items (cups, plates)

### Expected Results
- Success rate: 80-90% per object
- Complete table clear: 70-80%
- Demonstrates: Sequential manipulation, grasp planning

## Project 3: Warehouse Sorting (Medium-Advanced)

### Overview
Robot sorts packages based on labels/properties into different bins.

### Tasks
1. Identify packages on conveyor or shelf
2. Read label (OCR or barcode)
3. Determine destination bin
4. Pick package
5. Navigate to appropriate bin
6. Place package
7. Return to start position

### Technical Approach
- **VLA**: Vision transformer for label reading, LLM for routing logic
- **Planning**: Prioritized planning (handle multiple packages)
- **Control**: Whole-body planning for stability with heavy objects
- **Simulation**: PyBullet with warehouse environment

### Key Challenges
- Variable package sizes and weights
- Real-time sorting decision-making
- Efficient routing to minimize travel

### Expected Results
- Sorting accuracy: 90-95%
- Throughput: 5-8 packages/minute
- Demonstrates: Classification, efficient planning, whole-body control

## Project 4: Collaborative Assembly (Advanced)

### Overview
Robot assists human in furniture assembly by holding parts, passing tools, and following instructions.

### Tasks
1. Receive verbal instruction from human
2. Identify requested object
3. Grasp object
4. Hand object to human (coordinated handover)
5. Hold object in place while human works
6. Respond to dynamic instructions ("move it left", "hold it steady")

### Technical Approach
- **VLA**: RT-2 style model for natural language understanding and visual reasoning
- **Planning**: Reactive planning for real-time responsiveness
- **Control**: Force control for gentle handovers, compliant holding
- **Simulation**: Isaac Sim with human avatar

### Key Challenges
- Human-robot safety (force limits, collision avoidance)
- Natural language understanding (ambiguous instructions)
- Real-time reactivity to human motion

### Expected Results
- Task completion with human: 70-80%
- Safety: 100% (no collisions)
- Demonstrates: HRI, reactive planning, force control

## Project 5: Laundry Folding Robot (Advanced)

### Overview
Robot picks clothing items from basket and folds them.

### Tasks
1. Grasp clothing item from basket
2. Identify clothing type (shirt, pants, towel)
3. Manipulate to spread out flat
4. Execute folding sequence
5. Place folded item in stack

### Technical Approach
- **VLA**: Deformable object recognition, manipulation policy learned from demonstrations
- **Planning**: Contact-rich manipulation planning
- **Control**: Dual-arm coordination, tactile feedback
- **Simulation**: Isaac Sim with cloth simulation (FleX or PhysX)

### Key Challenges
- Deformable object manipulation (cloth dynamics)
- Grasp planning for limp objects
- Folding coordination (complex multi-contact)

### Expected Results
- Success rate: 60-75% (deformable manipulation is hard!)
- Demonstrates: Advanced manipulation, dual-arm, deformables

## Project Template

### Use this template for your own project:

**Project Title**: [Your Project Name]

**Overview**: [1-2 sentence description]

**Tasks**:
1. [High-level task 1]
2. [High-level task 2]
3. ...

**Technical Approach**:
- **VLA**: [Model/approach for language and vision]
- **Planning**: [Algorithms for task and motion planning]
- **Control**: [Control strategies]
- **Simulation**: [Environment]

**Key Challenges**:
- [Challenge 1 and how you'll address it]
- [Challenge 2]
- [Challenge 3]

**Expected Results**:
- [Success rate goal]
- [Timing goal]
- [What it demonstrates]

**Timeline**:
- Week 1-2: [Phase]
- Week 3-4: [Phase]
- ...

## Project Selection Guide

### Choose based on:

**Your Interests**:
- Manipulation-focused: Projects 1, 2, 5
- Navigation-focused: Project 3
- HRI-focused: Project 4

**Available Resources**:
- Limited compute: Projects 1, 2 (simpler VLA)
- GPU available: Projects 3, 4, 5 (advanced VLA)

**Difficulty Level**:
- Basic: Project 2
- Medium: Projects 1, 3
- Advanced: Projects 4, 5

### Scope Adjustment

**To Simplify**:
- Reduce number of tasks (e.g., only picking, no placing)
- Use rule-based instead of learned VLA
- Fewer object types
- Static environment only

**To Increase Complexity**:
- Add dynamic obstacles
- Multiple robots (coordination)
- Learning from demonstrations
- Sim-to-real transfer

## Common Pitfalls to Avoid

1. **Over-ambitious Scope**: Start with MVP, add features incrementally
2. **Integration Hell**: Test components independently first
3. **Ignoring Failure Cases**: Plan for error handling from start
4. **Late Testing**: Test continuously, not just at the end
5. **Poor Documentation**: Document as you go, not at the end

## Project Resources

### Datasets
- **Objects**: YCB dataset, ShapeNet
- **Scenes**: Replica dataset, AI2-THOR
- **Instructions**: CORe49 (human-robot interaction), ALFRED

### Models
- **VLA**: CLIP, RT-2 (if available), OWL-ViT
- **Planning**: OMPL, MoveIt, Fast-Downward

### Environments
- **Simulation**: Isaac Sim, Gazebo, PyBullet, AI2-THOR
- **Worlds**: Pre-built environments from communities

## Next Steps

1. Choose a project (or design your own)
2. Write a 1-page project proposal
3. Get feedback from instructor
4. Begin implementation following [Implementation Phases](implementation-phases.md)
5. Use [Testing and Validation](testing-validation.md) strategies
6. Document using [Presentation Guidelines](presentation-guidelines.md)

Next: [Grading Rubric](grading-rubric.md) - Understand evaluation criteria.
