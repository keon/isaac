# Installation and Environment Setup

## Overview

Isaac Sim installation isn't just downloading an application. It's establishing an ecosystem:

- **NVIDIA drivers**: Must match GPU and CUDA requirements
- **Isaac Sim application**: The simulation core
- **Extensions**: Add-on functionality (Isaac Lab, ROS bridge, custom code)
- **Assets**: Robot models, environments, materials
- **Python environment**: Dependencies for your scripts
- **External tools**: ROS 2, ML frameworks, visualization

Getting this right initially prevents compounding problems later.

## System Requirements

Before installation, verify your system meets requirements. Isaac Sim is demanding.

### Hardware Requirements

**Minimum (for development):**
- **CPU**: Intel Core i7 (7th gen) or AMD Ryzen 5
- **RAM**: 32 GB
- **GPU**: NVIDIA RTX 3070 with 8GB VRAM
- **Storage**: 50 GB SSD

**Recommended (for production/training):**
- **CPU**: Intel Core i9 or AMD Ryzen 9
- **RAM**: 64 GB
- **GPU**: NVIDIA RTX 4080 (16GB VRAM) or better
- **Storage**: 1 TB NVMe SSD

**For RL training at scale:**
- Multiple RTX 4090s or A6000s
- 128+ GB RAM
- Fast storage for checkpoints

### GPU Constraints

Critical limitations:
- **RT Cores required**: GPUs without RT cores (A100, H100) cannot run Isaac Sim's rendering
- **VRAM matters**: Complex scenes and parallel environments consume VRAM rapidly
- **Multi-GPU**: Supported but requires careful configuration

If you're planning cloud deployment, this rules out many GPU instance types. RTX instances are less common than A100/H100 compute instances.

### Operating System

Officially supported:
- Ubuntu 22.04 LTS
- Ubuntu 24.04 LTS

Windows support exists but is less thoroughly tested for robotics workflows. This book assumes Linux.

## Installation Paths

Isaac Sim can be installed several ways, each with tradeoffs.

### Option 1: Omniverse Launcher (Interactive)

The graphical approach:

1. Install NVIDIA Omniverse Launcher from nvidia.com
2. Sign in with NVIDIA account
3. Navigate to Exchange → Isaac Sim
4. Install desired version

**Pros:**
- Easy initial setup
- Manages multiple versions
- GUI for configuration

**Cons:**
- Not reproducible (manual steps)
- Harder to automate in CI
- Version management is GUI-based

**Use for**: Initial exploration, learning

### Option 2: Python pip Install (Lightweight)

For Isaac Sim 4.0+, pip installation is available:

```bash
# Create virtual environment
python -m venv isaac_env
source isaac_env/bin/activate

# Install Isaac Sim
pip install isaacsim
```

**Pros:**
- Familiar Python workflow
- Easy virtual environment management
- Good for scripts and notebooks

**Cons:**
- Larger download (downloads on first run)
- May have PATH/environment issues
- Extension management different from launcher

**Use for**: Python-centric development, Jupyter workflows

### Option 3: Docker Container (Recommended for Production)

Container-based installation from NGC:

```bash
# Login to NGC
docker login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key>

# Pull Isaac Sim container
docker pull nvcr.io/nvidia/isaac-sim:4.2.0

# Run container
docker run --name isaac-sim --gpus all -it \
    -e "ACCEPT_EULA=Y" \
    --network=host \
    -v ~/docker/isaac-sim/cache:/root/.cache/ov \
    -v ~/docker/isaac-sim/data:/root/Documents \
    nvcr.io/nvidia/isaac-sim:4.2.0
```

**Pros:**
- Reproducible environment
- Easy CI/CD integration
- Version pinning built-in
- Isolates dependencies

**Cons:**
- Docker complexity
- GPU passthrough setup required
- Larger disk footprint

**Use for**: Production, CI/CD, team collaboration

### Option 4: Workstation Install (Bare Metal)

Direct installation on the host system:

```bash
# Download from developer.nvidia.com
chmod +x isaac-sim-*.run
./isaac-sim-*.run

# Follow installer prompts
```

**Pros:**
- Maximum performance
- Direct GPU access
- No container overhead

**Cons:**
- Pollutes host system
- Version switching is painful
- Dependency conflicts possible

**Use for**: Dedicated simulation workstations

## Driver Installation

Isaac Sim requires specific NVIDIA driver versions. Getting this wrong causes cryptic failures.

### Checking Current Driver

```bash
nvidia-smi
# Look for "Driver Version" in output
```

### Installing/Updating Drivers

For Isaac Sim 4.x, driver 535+ is typically required. Check the release notes for your specific version.

```bash
# Ubuntu - using NVIDIA's repository
sudo apt-get update
sudo apt-get install -y nvidia-driver-535

# Or download from NVIDIA
wget https://us.download.nvidia.com/XFree86/Linux-x86_64/535.xx/NVIDIA-Linux-x86_64-535.xx.run
chmod +x NVIDIA-Linux-x86_64-535.xx.run
sudo ./NVIDIA-Linux-x86_64-535.xx.run
```

After installation, reboot and verify:

```bash
nvidia-smi
```

### Driver Compatibility

Always check the [Isaac Sim release notes](https://docs.omniverse.nvidia.com/isaacsim/latest/release_notes.html) for your specific version. As a general rule:

- Isaac Sim 4.x requires driver 535+
- Newer drivers are generally recommended for stability and performance
- Match the driver version specified in the release notes for production deployments

## Container Setup Deep Dive

For production use, containers are the recommended approach. Here's a thorough setup.

### Prerequisites

Install Docker and NVIDIA Container Toolkit:

```bash
# Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
# Log out and back in for group membership

# NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

Verify GPU access in Docker:

```bash
docker run --rm --gpus all nvidia/cuda:12.0-base-ubuntu22.04 nvidia-smi
```

### NGC Authentication

Generate an API key at ngc.nvidia.com:

1. Sign in to NGC
2. Go to Setup → API Key
3. Generate and save the key

```bash
docker login nvcr.io
# Username: $oauthtoken
# Password: <your API key>
```

### Volume Mounts

Isaac Sim uses several directories. Mount them for persistence:

```bash
# Create directories
mkdir -p ~/isaac-sim/{cache,data,logs,config}

# Run with mounts
docker run --name isaac-sim --gpus all -it \
    -e "ACCEPT_EULA=Y" \
    --network=host \
    -v ~/isaac-sim/cache:/root/.cache/ov \
    -v ~/isaac-sim/data:/root/Documents \
    -v ~/isaac-sim/logs:/root/.nvidia-omniverse/logs \
    -v ~/isaac-sim/config:/root/.config/ov \
    -v ~/my-project:/workspace \
    nvcr.io/nvidia/isaac-sim:4.2.0 bash
```

Key mounts:
- `/root/.cache/ov`: Asset cache (large, reusable across runs)
- `/root/Documents`: User data
- `/root/.nvidia-omniverse/logs`: Logs for debugging
- `/workspace`: Your project code

### Display Forwarding

For GUI access from containers:

```bash
# Allow X connections
xhost +local:docker

# Run with display
docker run --gpus all -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    # ... other options ...
    nvcr.io/nvidia/isaac-sim:4.2.0
```

For headless operation, no display forwarding is needed.

## Python Environment Management

Isaac Sim includes its own Python environment. Managing dependencies requires care.

### Isaac Sim's Python

Isaac Sim bundles Python with pre-installed packages. Location varies by installation method:

```bash
# Launcher install
~/.local/share/ov/pkg/isaac-sim-*/python.sh

# Container
/isaac-sim/python.sh
```

Use this Python for Isaac Sim scripts:

```bash
# Instead of
python my_script.py

# Use
/path/to/isaac-sim/python.sh my_script.py
```

### Additional Dependencies

To install additional packages into Isaac Sim's environment:

```bash
# Using Isaac Sim's pip
/path/to/isaac-sim/python.sh -m pip install numpy-quaternion

# Or from within a script
import subprocess
subprocess.check_call([sys.executable, "-m", "pip", "install", "package"])
```

### Virtual Environments with Isaac Sim

For pip-installed Isaac Sim, use standard virtual environments:

```bash
python -m venv isaac_venv
source isaac_venv/bin/activate
pip install isaacsim
pip install your-other-dependencies
```

Create a `requirements.txt` for reproducibility:

```
isaacsim==4.2.0
numpy==1.24.0
torch==2.0.0
# ... other dependencies
```

## Extension Management

Isaac Sim functionality comes through extensions. Managing them is part of environment setup.

### Core Extensions

Isaac Sim includes essential extensions:
- `omni.isaac.core`: Base simulation APIs
- `omni.isaac.sensor`: Sensor simulation
- `omni.physx`: Physics engine interface
- `omni.isaac.ros2_bridge`: ROS 2 integration

### Installing Additional Extensions

From the GUI: Window → Extensions → search and install

Programmatically:

```python
import omni.kit.app
manager = omni.kit.app.get_app().get_extension_manager()
manager.enable_extension("omni.isaac.ros2_bridge")
```

### Isaac Lab

Isaac Lab (formerly Isaac Gym) is a separate extension for RL:

```bash
# Clone Isaac Lab
git clone https://github.com/isaac-sim/IsaacLab.git

# Install into Isaac Sim's Python
cd IsaacLab
/path/to/isaac-sim/python.sh -m pip install -e .
```

## Project Structure

Organize projects for reproducibility and collaboration.

### Recommended Structure

```
my_isaac_project/
├── docker/
│   ├── Dockerfile
│   └── docker-compose.yml
├── assets/
│   ├── robots/
│   │   └── my_robot.usd
│   └── environments/
│       └── warehouse.usd
├── src/
│   ├── controllers/
│   ├── tasks/
│   └── utils/
├── configs/
│   └── simulation.yaml
├── scripts/
│   ├── train.py
│   └── evaluate.py
├── tests/
│   └── test_simulation.py
├── requirements.txt
├── setup.py
└── README.md
```

### Configuration Files

Externalize configuration:

```yaml
# configs/simulation.yaml
physics:
  dt: 0.00833  # 120 Hz
  substeps: 2
  gpu_dynamics: true

robot:
  urdf_path: assets/robots/my_robot.urdf
  initial_position: [0, 0, 0.5]

environment:
  usd_path: assets/environments/warehouse.usd
```

Load in code:

```python
import yaml

with open("configs/simulation.yaml") as f:
    config = yaml.safe_load(f)

physics_dt = config["physics"]["dt"]
```

### Dockerfile for Reproducibility

```dockerfile
FROM nvcr.io/nvidia/isaac-sim:4.2.0

# Install additional dependencies
RUN /isaac-sim/python.sh -m pip install \
    pyyaml \
    tensorboard \
    wandb

# Copy project
COPY . /workspace
WORKDIR /workspace

# Install project package
RUN /isaac-sim/python.sh -m pip install -e .

# Default command
CMD ["/isaac-sim/python.sh", "scripts/train.py"]
```

Build and run:

```bash
docker build -t my-isaac-project:latest .
docker run --gpus all -it my-isaac-project:latest
```

## Version Management

As Isaac Sim evolves, managing versions becomes critical.

### Pinning Versions

Always pin your Isaac Sim version:

```yaml
# docker-compose.yml
services:
  isaac-sim:
    image: nvcr.io/nvidia/isaac-sim:4.2.0  # Explicit version
    # ...
```

```python
# requirements.txt
isaacsim==4.2.0  # Explicit version
```

### Upgrade Strategy

When upgrading Isaac Sim:

1. **Read release notes**: Check for breaking changes
2. **Test in isolation**: Run your test suite on new version
3. **Check physics behavior**: Dynamics may differ between versions
4. **Verify extension compatibility**: Custom extensions may need updates
5. **Update configurations**: API changes may require config updates

### Multi-Version Development

For testing across versions, use container tags:

```bash
# Run tests on multiple versions
for version in 4.0.0 4.1.0 4.2.0; do
    docker run --gpus all \
        nvcr.io/nvidia/isaac-sim:$version \
        /isaac-sim/python.sh -m pytest tests/
done
```

## Troubleshooting Installation

Common installation issues and solutions.

### "Failed to initialize PhysX"

**Cause**: Driver mismatch or GPU not properly detected.

**Fix**:
```bash
# Verify driver
nvidia-smi

# Check PhysX library
ldd /path/to/isaac-sim/exts/omni.physx/bin/linux-x86_64/release/*.so | grep nvidia
```

### "Extension failed to load"

**Cause**: Missing dependencies or incompatible versions.

**Fix**:
```bash
# Check extension logs
cat ~/.nvidia-omniverse/logs/Kit/*/omni.kit.app.*.log | grep -i error
```

### "Out of GPU memory"

**Cause**: Scene too complex or VRAM insufficient.

**Fix**:
- Reduce parallel environment count
- Lower render resolution
- Use `render=False` when possible
- Close other GPU applications

### "Nucleus connection failed"

**Cause**: Network issues or authentication problems.

**Fix**:
```bash
# Test connectivity
curl -I https://omniverse-nucleus.nvidia.com

# Check credentials
cat ~/.nvidia-omniverse/config/omniverse.toml
```

For offline work, use local assets instead of Nucleus references.

## Running Example

For our warehouse mobile manipulator project, here's the environment setup.

### Project Dockerfile

```dockerfile
FROM nvcr.io/nvidia/isaac-sim:4.2.0

# System dependencies
RUN apt-get update && apt-get install -y \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
COPY requirements.txt /tmp/
RUN /isaac-sim/python.sh -m pip install -r /tmp/requirements.txt

# Project code
COPY . /workspace/mobile_manipulator
WORKDIR /workspace/mobile_manipulator

# Install project
RUN /isaac-sim/python.sh -m pip install -e .

ENV PYTHONPATH="/workspace/mobile_manipulator:$PYTHONPATH"
```

### Development Workflow

```bash
# Build container
docker build -t warehouse-sim:dev .

# Run interactive development
docker run --gpus all -it \
    -v $(pwd):/workspace/mobile_manipulator \
    -v ~/isaac-cache:/root/.cache/ov \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    warehouse-sim:dev bash

# Inside container
/isaac-sim/python.sh scripts/test_environment.py
```

### CI Configuration

```yaml
# .github/workflows/test.yml
name: Simulation Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: [self-hosted, gpu]  # Requires self-hosted GPU runner
    container:
      image: nvcr.io/nvidia/isaac-sim:4.2.0
      options: --gpus all
    
    steps:
      - uses: actions/checkout@v4
      
      - name: Install dependencies
        run: /isaac-sim/python.sh -m pip install -e .
      
      - name: Run tests
        run: /isaac-sim/python.sh -m pytest tests/ -v
```

Note: GPU CI requires self-hosted runners with NVIDIA GPUs—standard GitHub Actions runners don't have GPUs.

## Chapter Summary

Key takeaways:

- **RT-core GPUs required**: A100/H100 won't work for rendering
- **Use containers for production**: Pin versions, isolate dependencies
- **Use Isaac Sim's Python**: Or manage virtual environments carefully
- **Externalize configuration**: Enable version-independent code
- **Test before upgrading**: Physics behavior can change between versions

Next: [Debugging Guide](./diagnostic-foundations.md)—troubleshooting when things go wrong.

## References

- [Isaac Sim Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/index.html) — Official installation guides
- [Isaac Sim Container Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html) — Docker setup and NGC authentication
- [Isaac Sim Release Notes](https://docs.omniverse.nvidia.com/isaacsim/latest/release_notes.html) — Version compatibility and driver requirements
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) — GPU Docker setup
