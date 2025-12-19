#!/bin/bash
# Omniverse Physics Playground Setup Script

echo "🚀 Setting up Omniverse Physics Playground..."

# Check for NVIDIA GPU
if ! command -v nvidia-smi &> /dev/null; then
    echo "⚠️  Warning: NVIDIA GPU not detected. Omniverse requires an NVIDIA GPU."
fi

# Check for Omniverse Launcher
OMNIVERSE_PATH="$HOME/.local/share/ov"
if [ ! -d "$OMNIVERSE_PATH" ]; then
    echo "❌ Omniverse not found at $OMNIVERSE_PATH"
    echo ""
    echo "Please install Omniverse Launcher from:"
    echo "  https://www.nvidia.com/en-us/omniverse/"
    echo ""
    echo "Then install Isaac Sim from the Omniverse Launcher."
    exit 1
fi

# Find Isaac Sim installation
ISAAC_SIM_PATH=$(ls -d $OMNIVERSE_PATH/pkg/isaac-sim-* 2>/dev/null | head -1)

if [ -z "$ISAAC_SIM_PATH" ]; then
    echo "❌ Isaac Sim not found."
    echo "Please install Isaac Sim from Omniverse Launcher."
    exit 1
fi

echo "✅ Found Isaac Sim at: $ISAAC_SIM_PATH"

# Create Python alias
ISAAC_PYTHON="$ISAAC_SIM_PATH/python.sh"

echo ""
echo "📦 Installing additional dependencies..."
$ISAAC_PYTHON -m pip install pyyaml omegaconf tqdm pandas --quiet

echo ""
echo "✅ Setup complete!"
echo ""
echo "Usage:"
echo "  # Run bridge builder"
echo "  $ISAAC_PYTHON src/bridge_builder.py"
echo ""
echo "  # Run physics simulation"
echo "  $ISAAC_PYTHON src/physics_sim.py"
echo ""
echo "  # Start Isaac Sim with our scene"
echo "  $ISAAC_SIM_PATH/isaac-sim.sh --scene scenes/sample_truss_bridge.usda"
echo ""
echo "🎉 Happy simulating!"
