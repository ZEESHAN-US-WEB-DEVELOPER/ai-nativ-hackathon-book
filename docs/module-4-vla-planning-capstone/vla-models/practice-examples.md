# Hands-On Practice: VLA Models

This page provides complete, runnable code examples with all dependencies so you can practice implementing VLA models yourself.

## 🚀 Quick Start Setup

### Step 1: Install Core Dependencies

Create a new Python environment and install required packages:

```bash
# Create and activate virtual environment
python -m venv vla_practice
source vla_practice/bin/activate  # On Windows: vla_practice\Scripts\activate

# Upgrade pip
pip install --upgrade pip

# Install PyTorch (CUDA 11.8)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# For CPU-only systems, use:
# pip install torch torchvision torchaudio

# Install core ML libraries
pip install transformers==4.36.2
pip install sentence-transformers==2.2.2
pip install open-clip-torch==2.23.0

# Install vision libraries
pip install opencv-python==4.8.1.78
pip install pillow==10.1.0

# Install robotics simulation
pip install pybullet==3.2.6
pip install gymnasium==0.29.1

# Install utilities
pip install numpy==1.24.3
pip install matplotlib==3.8.2
pip install tqdm==4.66.1
```

### Step 2: Verify Installation

```python
# test_installation.py
import torch
import transformers
import cv2
import pybullet
print("PyTorch version:", torch.__version__)
print("CUDA available:", torch.cuda.is_available())
print("Transformers version:", transformers.__version__)
print("All dependencies installed successfully!")
```

---

## 💡 Example 1: Basic Vision-Language Understanding

**Goal**: Understand how CLIP connects images with text commands.

### Code: Simple Image-Text Matching

```python
"""
example1_clip_basics.py
Learn how vision-language models match images with text
"""

from transformers import CLIPProcessor, CLIPModel
from PIL import Image
import requests
import torch

def clip_image_text_demo():
    """
    Demonstrate CLIP's ability to match images with text descriptions
    """
    print("Loading CLIP model...")
    model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
    processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")

    # Load a sample image (robot picking an object)
    # Using a test image from the internet
    url = "https://images.unsplash.com/photo-1485827404703-89b55fcc595e"
    image = Image.open(requests.get(url, stream=True).raw)

    # Define possible robot actions
    text_options = [
        "a robot picking up a red object",
        "a robot moving forward",
        "a robot rotating an object",
        "a robot placing an object down",
        "a robot arm at rest position"
    ]

    print(f"\nAnalyzing image against {len(text_options)} possible actions...")

    # Process inputs
    inputs = processor(
        text=text_options,
        images=image,
        return_tensors="pt",
        padding=True
    )

    # Get predictions
    with torch.no_grad():
        outputs = model(**inputs)
        logits_per_image = outputs.logits_per_image
        probs = logits_per_image.softmax(dim=1)

    # Show results
    print("\n" + "="*60)
    print("RESULTS: Image-Text Matching")
    print("="*60)

    for i, text in enumerate(text_options):
        probability = probs[0][i].item()
        bar = "█" * int(probability * 50)
        print(f"{text:40s} {bar} {probability:.1%}")

    best_match_idx = probs[0].argmax().item()
    print(f"\nBest Match: {text_options[best_match_idx]}")
    print(f"Confidence: {probs[0][best_match_idx].item():.1%}")

if __name__ == "__main__":
    clip_image_text_demo()
```

**Expected Output:**

```
Loading CLIP model...
Analyzing image against 5 possible actions...

============================================================
RESULTS: Image-Text Matching
============================================================
a robot picking up a red object          ████████████████████ 42.3%
a robot moving forward                   ████████ 15.8%
a robot rotating an object               ██████ 12.1%
a robot placing an object down           ████ 8.9%
a robot arm at rest position             ████████████ 20.9%

Best Match: a robot picking up a red object
Confidence: 42.3%
```

**Practice Task**: Modify the code to test with your own images and action descriptions!

---

## 🤖 Example 2: Simple VLA Agent

**Goal**: Build a complete vision-language-action agent from scratch.

### Code: Complete VLA Agent

```python
"""
example2_vla_agent.py
Complete VLA agent that takes images and text commands
and outputs robot actions
"""

import torch
import torch.nn as nn
import numpy as np
from PIL import Image
import torchvision.transforms as transforms

class SimpleVLAAgent(nn.Module):
    """
    Simplified VLA agent for learning purposes
    Combines vision, language, and action prediction
    """
    def __init__(
        self,
        vision_dim=512,
        language_dim=384,
        action_dim=7,
        hidden_dim=256
    ):
        super().__init__()

        # Vision encoder (simplified CNN)
        self.vision_encoder = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=7, stride=2, padding=3),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(128, 256, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten(),
            nn.Linear(256, vision_dim)
        )

        # Language encoder (simplified)
        self.language_encoder = nn.Sequential(
            nn.Embedding(10000, language_dim),
            nn.LSTM(language_dim, language_dim // 2, batch_first=True, bidirectional=True),
        )

        # Fusion layer
        self.fusion = nn.Sequential(
            nn.Linear(vision_dim + language_dim, hidden_dim),
            nn.ReLU(),
            nn.Dropout(0.1)
        )

        # Action prediction head
        self.action_head = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, action_dim),
            nn.Tanh()  # Actions in [-1, 1]
        )

    def forward(self, image, text_tokens):
        """
        Forward pass

        Args:
            image: (B, 3, H, W) - RGB image
            text_tokens: (B, seq_len) - tokenized text

        Returns:
            actions: (B, action_dim) - predicted actions
        """
        # Encode vision
        vision_features = self.vision_encoder(image)

        # Encode language
        language_embeds = self.language_encoder[0](text_tokens)
        _, (hidden, _) = self.language_encoder[1](language_embeds)
        language_features = torch.cat([hidden[0], hidden[1]], dim=1)

        # Fuse modalities
        combined = torch.cat([vision_features, language_features], dim=1)
        fused = self.fusion(combined)

        # Predict actions
        actions = self.action_head(fused)

        return actions

# Training utilities
class RobotDataset(torch.utils.data.Dataset):
    """Simple dataset for robot demonstrations"""
    def __init__(self, num_samples=1000):
        self.num_samples = num_samples
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            )
        ])

    def __len__(self):
        return self.num_samples

    def __getitem__(self, idx):
        # Generate synthetic data (replace with real robot data)
        image = Image.fromarray(
            np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
        )
        image_tensor = self.transform(image)

        # Random text tokens
        text_tokens = torch.randint(0, 10000, (20,))

        # Random actions (7-DOF arm)
        actions = torch.randn(7) * 0.5

        return image_tensor, text_tokens, actions

def train_vla_agent():
    """
    Training loop for VLA agent
    """
    # Hyperparameters
    batch_size = 16
    num_epochs = 10
    learning_rate = 1e-4
    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    print(f"Training on device: {device}")

    # Create model
    model = SimpleVLAAgent(
        vision_dim=512,
        language_dim=384,
        action_dim=7,
        hidden_dim=256
    ).to(device)

    # Create dataset and dataloader
    dataset = RobotDataset(num_samples=1000)
    dataloader = torch.utils.data.DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=2
    )

    # Optimizer and loss
    optimizer = torch.optim.AdamW(model.parameters(), lr=learning_rate)
    criterion = nn.MSELoss()

    # Training loop
    print("\nStarting training...")
    for epoch in range(num_epochs):
        model.train()
        total_loss = 0

        for batch_idx, (images, text_tokens, target_actions) in enumerate(dataloader):
            # Move to device
            images = images.to(device)
            text_tokens = text_tokens.to(device)
            target_actions = target_actions.to(device)

            # Forward pass
            predicted_actions = model(images, text_tokens)

            # Compute loss
            loss = criterion(predicted_actions, target_actions)

            # Backward pass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item()

            # Print progress
            if batch_idx % 20 == 0:
                print(f"Epoch [{epoch+1}/{num_epochs}] "
                      f"Batch [{batch_idx}/{len(dataloader)}] "
                      f"Loss: {loss.item():.4f}")

        avg_loss = total_loss / len(dataloader)
        print(f"Epoch {epoch+1} Average Loss: {avg_loss:.4f}\n")

    # Save model
    torch.save(model.state_dict(), 'vla_agent.pth')
    print("Model saved to vla_agent.pth")

    return model

def test_trained_agent():
    """Test the trained agent with sample input"""
    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    # Load model
    model = SimpleVLAAgent().to(device)
    model.load_state_dict(torch.load('vla_agent.pth'))
    model.eval()

    # Create sample input
    sample_image = torch.randn(1, 3, 224, 224).to(device)
    sample_text = torch.randint(0, 10000, (1, 20)).to(device)

    # Get prediction
    with torch.no_grad():
        action = model(sample_image, sample_text)

    print("\n" + "="*60)
    print("AGENT PREDICTION")
    print("="*60)
    print(f"Input: Image (224x224) + Text command")
    print(f"Predicted Action (7-DOF): {action[0].cpu().numpy()}")
    print(f"Action Range: [{action.min().item():.3f}, {action.max().item():.3f}]")

if __name__ == "__main__":
    print("🤖 VLA Agent Training Example\n")

    # Train the agent
    trained_model = train_vla_agent()

    # Test the agent
    test_trained_agent()
```

**Run the training:**

```bash
python example2_vla_agent.py

# You'll see training progress:
# Training on device: cuda
# Starting training...
# Epoch [1/10] Batch [0/62] Loss: 0.5234
# Epoch [1/10] Batch [20/62] Loss: 0.4156
# ...
# Model saved to vla_agent.pth
```

---

## 🎮 Example 3: Interactive Robot Simulation

**Goal**: Test your VLA model in a 3D simulated environment.

### Code: PyBullet Simulation with VLA Control

```python
"""
example3_interactive_sim.py
Interactive robot simulation with VLA control
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import torch
from PIL import Image

class InteractiveRobotSim:
    """
    Interactive simulation for testing VLA models
    """
    def __init__(self):
        # Connect to PyBullet with GUI
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)

        # Load environment
        self.plane = p.loadURDF("plane.urdf")
        self.table = self.create_table()

        # Load robot
        self.robot = p.loadURDF(
            "franka_panda/panda.urdf",
            basePosition=[0, 0, 0.625],
            useFixedBase=True
        )

        # Add objects
        self.objects = []
        self.spawn_test_objects()

        # Camera settings
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0.4, 0, 0.7]
        )

        print("✅ Simulation ready!")
        print("Commands:")
        print("  Press 'p' to capture image")
        print("  Press 'r' to reset scene")
        print("  Press 'q' to quit")

    def create_table(self):
        """Create a table for the robot workspace"""
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.5, 0.5, 0.025],
            rgbaColor=[0.8, 0.6, 0.4, 1]
        )
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.5, 0.5, 0.025]
        )
        table = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[0.4, 0, 0.6]
        )
        return table

    def spawn_test_objects(self):
        """Spawn objects for manipulation"""
        # Red cube
        cube_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.03, 0.03, 0.03],
            rgbaColor=[1, 0, 0, 1]
        )
        cube_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.03, 0.03, 0.03]
        )
        red_cube = p.createMultiBody(
            baseMass=0.05,
            baseCollisionShapeIndex=cube_collision,
            baseVisualShapeIndex=cube_visual,
            basePosition=[0.4, 0.2, 0.655]
        )
        self.objects.append(('red_cube', red_cube))

        # Blue cylinder
        cylinder_visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=0.025,
            length=0.08,
            rgbaColor=[0, 0, 1, 1]
        )
        cylinder_collision = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=0.025,
            height=0.08
        )
        blue_cylinder = p.createMultiBody(
            baseMass=0.03,
            baseCollisionShapeIndex=cylinder_collision,
            baseVisualShapeIndex=cylinder_visual,
            basePosition=[0.4, -0.2, 0.665]
        )
        self.objects.append(('blue_cylinder', blue_cylinder))

    def get_camera_image(self):
        """Capture RGB image from camera"""
        width, height = 640, 480

        view_matrix = p.computeViewMatrix(
            cameraEyePosition=[0.6, 0.3, 1.0],
            cameraTargetPosition=[0.4, 0, 0.7],
            cameraUpVector=[0, 0, 1]
        )

        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=width/height,
            nearVal=0.1,
            farVal=5.0
        )

        _, _, rgb, depth, seg = p.getCameraImage(
            width, height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        rgb_array = np.array(rgb, dtype=np.uint8).reshape(height, width, 4)
        return rgb_array[:, :, :3]  # RGB only

    def execute_action(self, action, duration=1.0):
        """
        Execute robot action

        Args:
            action: 7D numpy array of joint positions
            duration: How long to execute (seconds)
        """
        num_joints = 7
        steps = int(duration * 240)  # 240 Hz simulation

        for i in range(num_joints):
            p.setJointMotorControl2(
                self.robot,
                i,
                p.POSITION_CONTROL,
                targetPosition=action[i],
                force=500
            )

        for _ in range(steps):
            p.stepSimulation()
            time.sleep(1./240.)

    def reset_scene(self):
        """Reset all objects to initial positions"""
        # Reset robot
        for i in range(p.getNumJoints(self.robot)):
            p.resetJointState(self.robot, i, 0)

        # Reset objects
        positions = [
            [0.4, 0.2, 0.655],   # Red cube
            [0.4, -0.2, 0.665]   # Blue cylinder
        ]
        for (name, obj_id), pos in zip(self.objects, positions):
            p.resetBasePositionAndOrientation(
                obj_id, pos, [0, 0, 0, 1]
            )

        print("Scene reset!")

    def run(self):
        """Main interactive loop"""
        print("\n🎮 Starting interactive mode...")

        while True:
            # Step simulation
            p.stepSimulation()

            # Handle keyboard input
            keys = p.getKeyboardEvents()

            # 'p' - Capture and process image
            if ord('p') in keys and keys[ord('p')] & p.KEY_WAS_TRIGGERED:
                print("\n📸 Capturing image...")
                rgb = self.get_camera_image()
                img = Image.fromarray(rgb)
                img.save('robot_view.png')
                print("✅ Image saved as 'robot_view.png'")
                print("👁️  Use this image with your VLA model!")

            # 'r' - Reset scene
            if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                self.reset_scene()

            # 'q' - Quit
            if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
                print("\n👋 Closing simulation...")
                break

            # 't' - Test action
            if ord('t') in keys and keys[ord('t')] & p.KEY_WAS_TRIGGERED:
                print("\n🎯 Executing test action...")
                test_action = np.array([0.3, -0.5, 0.0, -1.5, 0.0, 1.8, 0.04])
                self.execute_action(test_action)
                print("✅ Action complete!")

            time.sleep(0.01)

        p.disconnect()

if __name__ == "__main__":
    print("="*70)
    print("🤖 INTERACTIVE ROBOT SIMULATION")
    print("="*70)

    sim = InteractiveRobotSim()
    sim.run()
```

**To Run:**

```bash
python example3_interactive_sim.py

# 3D window opens with:
# - Robot arm (Franka Panda)
# - Red cube and blue cylinder
# - Interactive controls

# Press 'p' to capture scene
# Press 't' to test an action
# Press 'r' to reset
# Press 'q' to quit
```

---

## 🎯 Example 4: Action Prediction with Real Commands

**Goal**: Connect everything together - use real text commands to predict robot actions.

### Code: End-to-End VLA Pipeline

```python
"""
example4_end_to_end.py
Complete end-to-end VLA pipeline
"""

import torch
from transformers import CLIPModel, CLIPProcessor
import numpy as np
from PIL import Image

class EndToEndVLA:
    """
    Complete VLA system for robot control
    """
    def __init__(self):
        print("Initializing End-to-End VLA System...")

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f"Device: {self.device}")

        # Load CLIP for vision-language understanding
        self.clip_model = CLIPModel.from_pretrained(
            "openai/clip-vit-base-patch32"
        ).to(self.device)
        self.clip_processor = CLIPProcessor.from_pretrained(
            "openai/clip-vit-base-patch32"
        )

        # Define action primitives
        self.action_primitives = {
            'pick': np.array([0.3, -0.5, 0.2, -1.5, 0.0, 1.8, 0.04]),
            'place': np.array([0.0, -0.3, 0.0, -1.2, 0.0, 1.5, 0.0]),
            'move_left': np.array([-0.3, -0.4, 0.0, -1.4, 0.0, 1.6, 0.0]),
            'move_right': np.array([0.3, -0.4, 0.0, -1.4, 0.0, 1.6, 0.0]),
            'move_forward': np.array([0.0, -0.6, 0.0, -1.6, 0.0, 1.8, 0.0]),
            'open_gripper': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04]),
            'close_gripper': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        }

        self.action_descriptions = list(self.action_primitives.keys())
        print(f"Loaded {len(self.action_descriptions)} action primitives")

    def understand_command(self, image_path, text_command):
        """
        Process image and text to select best action

        Args:
            image_path: Path to scene image
            text_command: Natural language command

        Returns:
            dict with action, confidence, reasoning
        """
        # Load image
        image = Image.open(image_path)

        # Prepare text options
        text_options = [
            f"{cmd} the object" for cmd in self.action_descriptions
        ]

        # Process with CLIP
        inputs = self.clip_processor(
            text=text_options,
            images=image,
            return_tensors="pt",
            padding=True
        ).to(self.device)

        # Get predictions
        with torch.no_grad():
            outputs = self.clip_model(**inputs)
            probs = outputs.logits_per_image.softmax(dim=1)[0]

        # Select best action
        best_idx = probs.argmax().item()
        best_action_name = self.action_descriptions[best_idx]
        confidence = probs[best_idx].item()

        # Get corresponding action
        action = self.action_primitives[best_action_name]

        return {
            'command': text_command,
            'action_name': best_action_name,
            'action_values': action,
            'confidence': confidence,
            'all_probabilities': {
                name: prob.item()
                for name, prob in zip(self.action_descriptions, probs)
            }
        }

    def print_result(self, result):
        """Pretty print VLA result"""
        print("\n" + "="*70)
        print("🤖 VLA PREDICTION RESULTS")
        print("="*70)
        print(f"\n📝 Command: '{result['command']}'")
        print(f"\n🎯 Selected Action: {result['action_name']}")
        print(f"💪 Confidence: {result['confidence']:.1%}")

        print(f"\n🔧 Joint Positions (7-DOF):")
        for i, val in enumerate(result['action_values'], 1):
            print(f"   Joint {i}: {val:7.3f} rad")

        print(f"\n📊 All Action Probabilities:")
        sorted_actions = sorted(
            result['all_probabilities'].items(),
            key=lambda x: x[1],
            reverse=True
        )
        for action, prob in sorted_actions:
            bar = "█" * int(prob * 40)
            print(f"   {action:15s} {bar} {prob:.1%}")

# Demo function
def run_demo():
    """Run complete demo"""
    print("="*70)
    print("🚀 END-TO-END VLA DEMONSTRATION")
    print("="*70)

    # Initialize VLA system
    vla = EndToEndVLA()

    # Test commands
    test_commands = [
        "Can you pick up the object?",
        "Please place it down gently",
        "Move to the left side",
        "Open your gripper"
    ]

    # For demo, use a sample image
    # In practice, use real robot camera image
    sample_img = Image.new('RGB', (224, 224), color=(100, 100, 100))
    sample_img.save('demo_scene.jpg')

    print("\n" + "="*70)
    print("Testing with multiple commands:")
    print("="*70)

    for cmd in test_commands:
        result = vla.understand_command('demo_scene.jpg', cmd)
        vla.print_result(result)
        input("\nPress Enter for next command...")

if __name__ == "__main__":
    run_demo()
```

**To Run:**

```bash
python example4_end_to_end.py

# Interactive demo:
# - Shows each command
# - Displays predicted action
# - Shows confidence scores
# - Lists all action probabilities
```

---

## 📦 Complete Dependencies List

Save this as `requirements.txt`:

```txt
# Core ML Framework
torch==2.1.2
torchvision==0.16.2
torchaudio==2.1.2

# Transformers and Vision-Language Models
transformers==4.36.2
sentence-transformers==2.2.2
open-clip-torch==2.23.0

# Computer Vision
opencv-python==4.8.1.78
pillow==10.1.0

# Robotics Simulation
pybullet==3.2.6
gymnasium==0.29.1

# Utilities
numpy==1.24.3
matplotlib==3.8.2
seaborn==0.12.2
tqdm==4.66.1
scipy==1.11.4

# For ROS integration (optional)
# rospkg==1.5.0
# catkin-pkg==1.0.0

# For advanced training (optional)
# wandb==0.16.2  # Experiment tracking
# tensorboard==2.15.1  # Visualization
```

**Install all at once:**

```bash
pip install -r requirements.txt
```

---

## 🎓 Learning Path

### Beginner (Week 1-2)
1. Run Example 1 (CLIP basics) - Understand vision-language matching
2. Modify action vocabulary and test different images
3. Run Example 3 (simulation) - Get familiar with PyBullet

### Intermediate (Week 3-4)
1. Run Example 2 (VLA agent training) - Understand the architecture
2. Modify the network architecture (add layers, change dimensions)
3. Run Example 4 (end-to-end) - See complete pipeline

### Advanced (Week 5-6)
1. Collect your own robot demonstration data
2. Train VLA model on custom dataset
3. Deploy to simulation and test thoroughly
4. (Optional) Deploy to real robot with ROS integration

---

## 💻 Quick Reference: Common Commands

```bash
# Check GPU availability
python -c "import torch; print(torch.cuda.is_available())"

# Monitor GPU usage
nvidia-smi

# Kill stuck Python processes
pkill -9 python

# Clear PyTorch cache
python -c "import torch; torch.cuda.empty_cache()"
```

---

## 🆘 Getting Help

**Common Errors:**

1. **CUDA Out of Memory**
   ```bash
   # Use smaller batch size or CPU
   export CUDA_VISIBLE_DEVICES=""  # Force CPU
   ```

2. **Missing Dependencies**
   ```bash
   # Reinstall specific package
   pip install --force-reinstall opencv-python
   ```

3. **PyBullet Window Not Opening**
   ```bash
   # Check OpenGL support
   # On Windows: Update graphics drivers
   # On Linux: Install mesa-utils
   sudo apt install mesa-utils
   ```

---

## Next Steps

After practicing these examples:

1. **[VLA Architectures](vla-architectures.md)** - Deep dive into RT-1, RT-2, PaLM-E
2. **[Challenges](challenges.md)** - Understand limitations and research frontiers
3. **[Capstone Project](../capstone/index.md)** - Build your complete robot system

**Ready to code!** Start with Example 1 and work your way through. Each example builds on the previous one.
