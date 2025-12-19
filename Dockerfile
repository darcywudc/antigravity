# NVIDIA Isaac Sim base image with Python
FROM nvcr.io/nvidia/isaac-sim:2023.1.1

# Set working directory
WORKDIR /workspace

# Set Isaac Sim path
ENV ISAAC_SIM_PATH=/isaac-sim
ENV OMNI_KIT_ACCEPT_EULA=yes

# Copy project files
COPY requirements.txt .
COPY src/ ./src/
COPY configs/ ./configs/
COPY scenes/ ./scenes/

# Install additional Python dependencies using Isaac Sim's Python
RUN ${ISAAC_SIM_PATH}/python.sh -m pip install --no-cache-dir \
    pyyaml \
    omegaconf \
    tqdm \
    pandas

# Note: gymnasium and stable-baselines3 may have conflicts, install separately if needed
# RUN ${ISAAC_SIM_PATH}/python.sh -m pip install --no-cache-dir gymnasium stable-baselines3

# Default command: run headless simulation
ENTRYPOINT ["/isaac-sim/python.sh"]
CMD ["src/run_simulation.py", "--mode", "simulate", "--output", "/workspace/results.json"]
