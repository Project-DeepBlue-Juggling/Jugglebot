FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3.11 python3.11-venv python3.11-dev python3-pip \
    libgl1-mesa-glx libglew-dev libosmesa6-dev libglfw3 libglfw3-dev \
    libhidapi-dev \
    && rm -rf /var/lib/apt/lists/*

# Make python3.11 the default python3
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1

COPY requirements.txt .
RUN python3 -m pip install --no-cache-dir -r requirements.txt

COPY . /app
WORKDIR /app

# Also make the parent repo available for imports from motion/
# The repo root is mounted at /repo in compose.yaml
ENV PYTHONPATH="/app:/repo/ros_ws/src/jugglebot:/repo/config/generated"
