# 🌉 Omniverse Physics Playground

A playground for physics simulations using NVIDIA Omniverse, focusing on structural mechanics, bridge simulation, and reinforcement learning.

## 🎯 Goals

- **Structural Simulation**: Create structures (bridges, buildings) and analyze forces, stress, displacement
- **Physics Engine**: Leverage PhysX 5 for realistic rigid body and deformable body simulation
- **Machine Learning**: Train agents to optimize structural designs or control dynamic systems
- **Visualization**: Real-time 3D visualization of simulation results

## 🛠️ Tech Stack

| Component | Technology |
|-----------|------------|
| Platform | NVIDIA Omniverse / Isaac Sim |
| Physics | PhysX 5 SDK |
| Scene Format | USD (Universal Scene Description) |
| Scripting | Python 3.10+ |
| ML Framework | Isaac Lab / Stable Baselines3 |

## 📁 Project Structure

```
omniverse/
├── README.md
├── requirements.txt
├── setup/                 # Installation scripts
│   └── install.sh
├── assets/               # USD assets, materials
│   └── bridges/
├── src/                  # Python source code
│   ├── __init__.py
│   ├── bridge_builder.py   # Bridge construction utilities
│   ├── physics_sim.py      # Physics simulation wrapper
│   └── rl_training.py      # RL training scripts
├── scenes/               # USD scene files
│   └── simple_bridge.usda
├── configs/              # Configuration files
│   └── physics_config.yaml
└── notebooks/            # Jupyter notebooks for experiments
    └── 01_getting_started.ipynb
```

## 🚀 Quick Start

### Prerequisites

1. NVIDIA GPU (RTX series recommended)
2. NVIDIA Omniverse Launcher
3. Isaac Sim 4.0+ or Omniverse Kit
4. Python 3.10+

### Installation

```bash
# Clone this repo
git clone https://github.com/darcywudc/antigravity.git
cd antigravity
git checkout omniverse

# Install dependencies (use Isaac Sim's Python)
~/.local/share/ov/pkg/isaac-sim-*/python.sh -m pip install -r requirements.txt
```

### Run First Simulation

```bash
# Launch Isaac Sim with our scene
~/.local/share/ov/pkg/isaac-sim-*/python.sh src/physics_sim.py
```

## 📚 Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/)
- [PhysX Documentation](https://nvidia-omniverse.github.io/PhysX/)
- [USD Python API](https://openusd.org/docs/api/index.html)
- [Isaac Lab (RL)](https://isaac-sim.github.io/IsaacLab/)

## 🔬 Experiments

### 1. Simple Beam Bending
Apply forces to a beam and measure deflection.

### 2. Bridge Load Test
Build a truss bridge and test maximum load capacity.

### 3. RL Bridge Optimizer
Train an agent to optimize bridge design for given constraints.

---

**Note**: Omniverse requires local installation. Cloud/headless options available via Isaac Sim Docker.
