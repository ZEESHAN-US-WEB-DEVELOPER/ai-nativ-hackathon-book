# Presentation Guidelines

This guide covers how to document and present your capstone project, including technical reports, demonstration videos, and presentations.

## Technical Report

### Structure (10-15 pages)

1. **Introduction** (1-2 pages)
   - Motivation
   - Problem statement
   - Objectives
   - Scope

2. **Background** (1-2 pages)
   - Related work
   - Course concepts used
   - Technologies and tools

3. **System Architecture** (2-3 pages)
   - High-level design
   - Component diagram
   - Integration architecture
   - Data flow

4. **Implementation** (3-4 pages)
   - Key algorithms
   - VLA integration
   - Planning implementation
   - Control system
   - Challenges and solutions

5. **Results** (2-3 pages)
   - Test scenarios
   - Performance metrics
   - Success rates
   - Timing analysis
   - Failure analysis

6. **Discussion** (1-2 pages)
   - Lessons learned
   - Limitations
   - Future improvements

7. **Conclusion** (0.5-1 page)
   - Summary
   - Achievements

8. **References**
   - Course materials
   - Papers
   - Libraries/tools

### Writing Tips

- **Be Concise**: Every sentence should add value
- **Use Figures**: Architecture diagrams, flowcharts, results plots
- **Quantitative**: Include numbers (success rates, timing)
- **Honest**: Discuss failures and limitations
- **Professional**: Proofread, consistent formatting

### LaTeX Template

```latex
\documentclass[conference]{IEEEtran}
\usepackage{graphicx}
\usepackage{amsmath}

\title{Capstone Project: [Your Title]}
\author{\IEEEauthorblockN{Your Name}
\IEEEauthorblockA{Course: AI Humanoids\\
Institution}}

\begin{document}
\maketitle

\begin{abstract}
[100-150 word summary]
\end{abstract}

\section{Introduction}
[Content]

\section{System Architecture}
[Content]

% More sections...

\bibliographystyle{IEEEtran}
\bibliography{references}

\end{document}
```

## Demonstration Video

### Requirements

- **Duration**: 3-5 minutes
- **Quality**: 720p minimum, 1080p preferred
- **Audio**: Clear narration explaining what's happening
- **Content**: Show system executing multiple tasks

### Structure

1. **Introduction** (30 seconds)
   - Title slide
   - Your name
   - Project overview

2. **System Overview** (30 seconds)
   - Show architecture diagram
   - Explain components

3. **Demonstrations** (2-3 minutes)
   - Task 1: Simple (e.g., pick and place)
   - Task 2: Medium complexity (e.g., navigation + manipulation)
   - Task 3: Complex (e.g., multi-step task)
   - Show both successes and recoveries

4. **Results Summary** (30 seconds)
   - Key metrics
   - Success rates
   - Timing

5. **Conclusion** (30 seconds)
   - Achievements
   - Future work

### Recording Tips

- **Screen Recording**: OBS Studio, QuickTime, or similar
- **Simulation View**: Show both RViz and Gazebo/Isaac Sim
- **Slow Motion**: For complex actions, show in slow-mo
- **Annotations**: Add text overlays explaining steps
- **Background Music**: Optional, keep it subtle

### Narration Script Example

```
"Hello, I'm [Name], and this is my capstone project: [Title].

The system integrates VLA models, motion planning, and simulation
to enable a humanoid robot to perform manipulation tasks from
natural language instructions.

Let me demonstrate three tasks:

First, a simple pick-and-place. I give the instruction:
'Pick up the red cup.' The system detects the cup using CLIP,
plans a grasp, generates a collision-free trajectory with RRT*,
and executes the motion.

Second, navigation and manipulation...

[Continue narration matching visual demonstration]

In testing, the system achieved 85% success rate over 30 trials,
with average task completion time of 18 seconds.

Future work includes deploying on physical hardware and handling
deformable objects. Thank you."
```

## Presentation Slides

### Slide Deck (15-20 slides for 15-20 minute talk)

1. **Title Slide**
2. **Outline**
3. **Motivation** (1-2 slides)
4. **Problem Statement** (1 slide)
5. **Approach Overview** (1 slide)
6. **System Architecture** (1-2 slides)
7. **Key Techniques** (3-4 slides)
   - VLA integration
   - Planning algorithms
   - Control system
8. **Implementation Highlights** (2-3 slides)
9. **Demonstration** (video or live)
10. **Results** (2-3 slides)
11. **Discussion** (1-2 slides)
12. **Conclusion** (1 slide)
13. **Q&A**

### Slide Design Tips

- **Minimal Text**: Bullet points, not paragraphs
- **Visual**: Use diagrams, images, videos
- **Consistent**: Same template, fonts, colors
- **Readable**: Large fonts (≥24pt for body text)
- **High Contrast**: Dark text on light background or vice versa

### Example Slides

**Slide: System Architecture**
```
┌─────────────────────────────────────┐
│     System Architecture             │
├─────────────────────────────────────┤
│                                     │
│  [Architecture Diagram]             │
│                                     │
│  • VLA Model: CLIP + GPT-4          │
│  • Planning: PDDL + RRT*            │
│  • Control: MoveIt + ROS2           │
│  • Simulation: Isaac Sim            │
│                                     │
└─────────────────────────────────────┘
```

**Slide: Results**
```
┌─────────────────────────────────────┐
│     Performance Results             │
├─────────────────────────────────────┤
│                                     │
│  Task Success Rates:                │
│  • Pick & Place: 90% (27/30)        │
│  • Navigation: 87% (26/30)          │
│  • Multi-step: 80% (24/30)          │
│                                     │
│  Timing:                            │
│  • Avg. Planning: 2.3s              │
│  • Avg. Execution: 15.7s            │
│                                     │
│  [Bar chart of results]             │
│                                     │
└─────────────────────────────────────┘
```

## Live Demonstration

### Preparation

- **Test Everything**: Run through demo multiple times
- **Backup Plan**: Have video ready if live demo fails
- **Network**: Ensure stable connection if remote
- **Checkpoints**: Start from saved states if possible
- **Explain as You Go**: Narrate what's happening

### Demo Script

```
1. Show system idle state
2. Give instruction: "Pick up the cup"
3. Point out: "Notice the camera feed shows object detection"
4. Point out: "VLA model parsed the instruction"
5. Point out: "Task planner generated action sequence"
6. Point out: "Motion planner found collision-free path" (show RViz)
7. Execute motion
8. Point out: "Robot successfully grasped and lifted cup"
9. Repeat for 1-2 more tasks
```

## Q&A Preparation

### Anticipated Questions

**Technical**:
- "Why did you choose RRT* over A*?"
- "How does your VLA model handle ambiguous instructions?"
- "What's your worst-case planning time?"

**Implementation**:
- "How long did it take to implement?"
- "What was the hardest part?"
- "How did you debug X?"

**Results**:
- "Why is the success rate not 100%?"
- "What causes failures?"
- "How would you improve performance?"

**Future Work**:
- "Would this work on a real robot?"
- "How would you handle moving obstacles?"
- "What about learning from experience?"

### Answering Tips

- **Be Honest**: If you don't know, say so
- **Be Specific**: Give concrete examples
- **Be Reflective**: Show you've thought about limitations
- **Be Positive**: Frame challenges as learning opportunities

## Documentation Repository

### README Structure

```markdown
# Capstone Project: [Title]

## Overview
[Brief description]

## Demo Video
[Link to video]

## Installation
\`\`\`bash
# Clone repository
git clone ...

# Install dependencies
pip install -r requirements.txt

# Build ROS workspace
colcon build
\`\`\`

## Usage
\`\`\`bash
# Launch simulation
ros2 launch capstone simulation.launch.py

# Run capstone system
ros2 run capstone main_node
\`\`\`

## Architecture
[Diagram or description]

## Results
[Summary of performance]

## Documentation
- [Technical Report](docs/report.pdf)
- [Presentation Slides](docs/slides.pdf)

## License
MIT
```

## Submission Checklist

- [ ] Technical report (PDF)
- [ ] Source code (GitHub/GitLab)
- [ ] README with setup instructions
- [ ] Demonstration video (uploaded, link in README)
- [ ] Presentation slides (PDF)
- [ ] Test results (CSV or plots)
- [ ] All code committed and pushed
- [ ] Repository public or shared with instructor

## Grading Considerations

Presentation quality matters:
- **Clarity**: Easy to understand
- **Organization**: Logical flow
- **Professionalism**: Polished, proofread
- **Completeness**: All requirements addressed
- **Honesty**: Acknowledges limitations

## Summary

A strong presentation includes:
- Clear, concise technical report
- Engaging demonstration video
- Professional presentation slides
- Thorough documentation
- Honest discussion of results and limitations

Next: [Example Projects](example-projects.md) - See concrete project examples.
