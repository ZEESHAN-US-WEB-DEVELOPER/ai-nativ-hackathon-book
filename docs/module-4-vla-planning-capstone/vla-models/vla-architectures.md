# VLA Architectures

This section explores specific Vision-Language-Action architectures that have demonstrated success in real-world robotics applications. We focus on RT-1, RT-2, and PaLM-E—three influential systems that represent the state-of-the-art in VLA research.

## RT-1: Robotics Transformer 1

**Paper**: "RT-1: Robotics Transformer for Real-World Control at Scale" (Google Research, 2022)

### Overview

RT-1 is a transformer-based model that directly outputs robot actions from visual observations and language instructions. It was trained on a large-scale dataset of real robot demonstrations.

### Architecture

```
Image (300x300x3) → EfficientNet → Visual Tokens (81 tokens, 512-dim)
Text Instruction → Universal Sentence Encoder → Language Embedding (512-dim)
    ↓
Token Transformer (8 layers, 8 attention heads)
    ↓
Action Tokens → Discretized Actions (7-DOF arm + gripper)
```

**Key Components**:

1. **Visual Tokenization**: EfficientNet-B3 extracts features, produces 81 spatial tokens
2. **Language Encoding**: USE (Universal Sentence Encoder) embeds instruction
3. **Transformer Backbone**: Token Transformer processes visual and language tokens
4. **Action Discretization**: Continuous actions discretized into 256 bins per dimension

### Training

- **Dataset**: 130,000 robot episodes from 700+ tasks
- **Tasks**: Pick-and-place, drawer opening, object manipulation
- **Training Time**: Several days on TPUv4 pods
- **Data Augmentation**: Color jittering, random cropping, temporal dropout

### Key Innovations

1. **Large-scale robot data**: Trained on diverse real-world demonstrations
2. **Action tokenization**: Discrete actions enable transformer processing
3. **History context**: Model observes 6 recent frames for temporal reasoning
4. **Success prediction**: Auxiliary head predicts task success probability

### Performance

- **Success Rate**: 97% on trained tasks, 70% on novel but similar tasks
- **Generalization**: Handles object variations, lighting changes, background clutter
- **Failure Modes**: Struggles with precise alignment, long-horizon tasks

## RT-2: Robotics Transformer 2

**Paper**: "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control" (Google DeepMind, 2023)

### Overview

RT-2 extends RT-1 by co-training on both web-scale vision-language data and robot demonstration data. This enables stronger generalization and emergent capabilities.

### Architecture

```
Image → ViT (Vision Transformer) → Visual Tokens
Text → T5/PaLI Encoder → Language Tokens
    ↓
[Visual | Language | Action] Tokens → Co-Transformer
    ↓
Action String Tokens (e.g., "1 128 91 245 ... ")
```

**Key Differences from RT-1**:

1. **Pre-trained backbone**: Uses PaLI or PaLM-E vision-language models
2. **Action as text**: Actions represented as text strings, not separate tokens
3. **Web-scale pre-training**: Leverages internet-scale image-text data
4. **Unified architecture**: Vision, language, and action in single transformer

### Training Strategy

**Phase 1: Vision-Language Pre-training**
- Dataset: WebLI (10B image-text pairs)
- Task: Visual question answering, captioning, object detection
- Duration: Weeks on large TPU clusters

**Phase 2: Robot Fine-tuning**
- Dataset: RT-1 dataset + new demonstrations
- Task: Action prediction from visual observations and instructions
- Duration: Days on TPUs

### Key Innovations

1. **Transfer from web data**: Internet knowledge transfers to robot control
2. **Emergent capabilities**: Zero-shot reasoning about novel objects
3. **Chain-of-thought**: Can explain reasoning before acting
4. **Improved generalization**: 3x better performance on unseen tasks vs RT-1

### Performance

- **Success Rate**: 90% on trained tasks, 62% on novel tasks
- **Reasoning**: Can handle abstract concepts ("pick up the extinct animal")
- **Limitations**: Still struggles with complex manipulation, long-horizon planning

## PaLM-E: Embodied Multimodal Language Model

**Paper**: "PaLM-E: An Embodied Multimodal Language Model" (Google Research, 2023)

### Overview

PaLM-E integrates continuous sensor observations directly into a large language model (PaLM), enabling the model to reason about the physical world and generate robot actions.

### Architecture

```
Camera Image → ViT → Visual Tokens → Input Embeddings
Robot State → MLP → State Tokens → Input Embeddings
Text → PaLM Tokenizer → Text Tokens
    ↓
PaLM Language Model (540B parameters)
    ↓
Generated Text (includes action commands)
```

**Key Components**:

1. **Vision Encoder**: ViT-22B processes images into token sequences
2. **State Encoder**: MLP encodes robot joint positions, end-effector pose
3. **PaLM Backbone**: 540B parameter language model
4. **Action Decoding**: Parse generated text for action commands

### Training

- **Multi-task Training**: Vision-language tasks + robotics tasks simultaneously
- **Datasets**:
  - OSCAR (image captions)
  - Language-Table (simulated tabletop manipulation)
  - Mobile manipulation (TidyBot dataset)
- **Scale**: Trained on thousands of TPUv4 chips

### Key Innovations

1. **Embodied language model**: LLM directly processes sensory input
2. **Massive scale**: 562B total parameters
3. **Multi-embodiment**: Same model controls different robot platforms
4. **Affordance reasoning**: Understands what actions are possible

### Capabilities

- **Long-horizon planning**: Breaks complex tasks into sequences
- **Common-sense reasoning**: Uses world knowledge for task execution
- **Multi-step tasks**: "Make me breakfast" → plans and executes multiple subtasks
- **Failure recovery**: Can replan when actions fail

### Performance

- **Success Rate**: 85% on long-horizon tasks (vs 35% for baseline)
- **Generalization**: Transfers across robot morphologies
- **Reasoning**: Explains decisions in natural language

## Architectural Comparison

| Model | Visual Encoder | Language Encoder | Action Representation | Parameters | Key Strength |
|-------|----------------|------------------|-----------------------|------------|--------------|
| RT-1 | EfficientNet | USE | Discrete tokens (256 bins) | ~35M | Real-world robustness |
| RT-2 | ViT | PaLI/T5 | Text strings | ~55B | Transfer from web data |
| PaLM-E | ViT-22B | PaLM | Text commands | 562B | Reasoning and planning |

## Design Principles Across VLA Models

### 1. Transformer Backbone

All modern VLA models use transformers:
- **Why**: Attention enables flexible information flow
- **Benefit**: Handles variable-length inputs (images, text, action sequences)
- **Challenge**: Computational cost scales quadratically

### 2. Pre-training + Fine-tuning

Leveraging large-scale pre-training:
- **RT-1**: Trained from scratch on robot data
- **RT-2**: Pre-trained on web data, fine-tuned on robot data
- **PaLM-E**: Pre-trained LLM, fine-tuned with embodied data

**Trend**: More pre-training data → better generalization

### 3. Action Representation

How actions are encoded matters:

**Discrete Bins (RT-1)**:
```
Continuous: [x=0.523, y=-0.187, z=0.845]
Discretized: [bin_134, bin_67, bin_217]
```
- Pros: Compatible with classification, enables probability distributions
- Cons: Resolution limits precision

**Text Strings (RT-2)**:
```
Action: "1 128 91 245 12 200 0"
```
- Pros: Native to language models, flexible
- Cons: Parsing overhead, requires careful formatting

**Language Commands (PaLM-E)**:
```
Action: "move_to(x=0.5, y=-0.2, z=0.8); grasp()"
```
- Pros: Interpretable, composable
- Cons: Requires symbolic execution layer

### 4. Multi-Task Learning

Training on diverse tasks improves robustness:
- RT-1: 700+ tasks in real-world settings
- RT-2: Vision-language tasks + robotics
- PaLM-E: Language, vision, and embodied tasks

**Key Insight**: Diversity in training data drives generalization

## Emerging Architectures

### Octo (Open X-Embodiment)

Open-source VLA trained on data from multiple robot platforms:
- **Goal**: Generalist policy for diverse robots
- **Approach**: Transformer trained on 1M+ trajectories from 20+ robots
- **Innovation**: Cross-embodiment transfer learning

### RoboCat (DeepMind)

Self-improving robot learning agent:
- **Architecture**: Vision-language transformer with action head
- **Innovation**: Generates synthetic training data through self-play
- **Performance**: Rapid adaptation to new tasks (< 100 demonstrations)

## Implementation Considerations

### Model Selection

**Choose RT-1 if**:
- You have real robot data
- Need fast inference (smaller model)
- Tasks are manipulation-focused

**Choose RT-2 if**:
- You want to leverage web pre-training
- Need better generalization to novel tasks
- Have access to vision-language models

**Choose PaLM-E if**:
- Tasks require complex reasoning
- Long-horizon planning is critical
- Computational resources are available

### Computational Requirements

| Model | Parameters | Inference Time | Hardware |
|-------|------------|----------------|----------|
| RT-1 | 35M | ~50ms | Single GPU (V100) |
| RT-2 | 55B | ~500ms | Multi-GPU (A100) |
| PaLM-E | 562B | ~2s | TPU pod or multi-GPU |

### Open-Source Resources

- **RT-1**: Code and checkpoints available (TensorFlow)
- **RT-2**: Limited release, pre-trained models available
- **PaLM-E**: Architecture described, no official release
- **Octo**: Fully open-source (PyTorch)

## Summary

VLA architectures represent different design trade-offs:

- **RT-1**: Practical, real-world robustness, moderate compute
- **RT-2**: Strong transfer learning, better generalization
- **PaLM-E**: Reasoning and planning, massive scale

All share common principles: transformer backbones, vision-language-action integration, and multi-task learning. The field is rapidly evolving toward models that combine web-scale pre-training with robot-specific fine-tuning.

Next: [Implementation](implementation.md) - Learn how to implement VLA models for your robotic system.
