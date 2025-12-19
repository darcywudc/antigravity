# NVIDIA Isaac Sim base image with Python
FROM nvcr.io/nvidia/isaac-sim:2023.1.1

# Set working directory
WORKDIR /workspace

# Copy project files
COPY requirements.txt .
COPY src/ ./src/
COPY configs/ ./configs/
COPY scenes/ ./scenes/

# Install additional Python dependencies
RUN ./python.sh -m pip install --no-cache-dir \
    pyyaml \
    omegaconf \
    tqdm \
    pandas \
    gymnasium \
    stable-baselines3

# Set environment variables
ENV OMNI_KIT_ACCEPT_EULA=yes
ENV ISAAC_SIM_PATH=/isaac-sim

# Default command: run headless simulation
CMD ["./python.sh", "src/run_simulation.py", "--mode", "simulate", "--output", "/workspace/results.json"]
