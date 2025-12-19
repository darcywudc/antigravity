# Google Cloud Deployment for Isaac Sim

## Prerequisites
- Google Cloud account with billing enabled
- `gcloud` CLI installed locally
- GPU quota approved (may need to request)

## Quick Start

### 1. Create GPU VM

```bash
# Set your project
gcloud config set project YOUR_PROJECT_ID

# Create VM with NVIDIA L4 GPU (recommended for Isaac Sim)
gcloud compute instances create isaac-sim-vm \
    --zone=us-central1-a \
    --machine-type=g2-standard-8 \
    --accelerator=type=nvidia-l4,count=1 \
    --image-family=ubuntu-2204-lts \
    --image-project=ubuntu-os-cloud \
    --boot-disk-size=200GB \
    --boot-disk-type=pd-ssd \
    --maintenance-policy=TERMINATE \
    --metadata=startup-script='#!/bin/bash
        # Install NVIDIA drivers
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
        curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
            sed "s#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g" | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
        sudo apt-get update
        sudo apt-get install -y nvidia-driver-535 docker.io nvidia-container-toolkit
        sudo systemctl enable docker
        sudo systemctl start docker
        sudo nvidia-ctk runtime configure --runtime=docker
        sudo systemctl restart docker
    '
```

### 2. SSH into VM

```bash
gcloud compute ssh isaac-sim-vm --zone=us-central1-a
```

### 3. Run Isaac Sim Container

```bash
# Pull Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Run with GPU support
docker run --gpus all -it \
    -v ~/omniverse:/workspace \
    -p 8211:8211 \
    nvcr.io/nvidia/isaac-sim:2023.1.1 \
    ./python.sh /workspace/src/bridge_builder.py
```

### 4. Headless Mode (for training)

For RL training without display:

```bash
docker run --gpus all -it \
    -v ~/omniverse:/workspace \
    nvcr.io/nvidia/isaac-sim:2023.1.1 \
    ./python.sh -c "
from omni.isaac.kit import SimulationApp
config = {'headless': True}
app = SimulationApp(config)
# Your simulation code here
"
```

## Cost Optimization

### Use Preemptible VMs (up to 80% cheaper)
```bash
gcloud compute instances create isaac-sim-vm \
    --preemptible \
    # ... other flags
```

### Stop when not in use
```bash
gcloud compute instances stop isaac-sim-vm --zone=us-central1-a
gcloud compute instances start isaac-sim-vm --zone=us-central1-a
```

## Alternative: Use Vertex AI Workbench
For Jupyter-based experiments, consider Vertex AI Workbench with GPU support.

## Files to Upload

After VM is ready:
```bash
# From your local machine
gcloud compute scp --recurse ./omniverse/* isaac-sim-vm:~/omniverse/ --zone=us-central1-a
```
