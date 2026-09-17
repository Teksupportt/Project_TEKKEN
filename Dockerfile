FROM ros:humble-ros-base

# Basic build tooling
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    wget \
    && rm -rf /var/lib/apt/lists/*

# --- ONNX Runtime (CPU build) ---
ARG ONNXRUNTIME_VERSION=1.18.0
RUN wget -q \
    "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz" \
    -O /tmp/onnxruntime.tgz && \
    mkdir -p /opt/onnxruntime && \
    tar -xzf /tmp/onnxruntime.tgz -C /opt/onnxruntime --strip-components=1 && \
    rm /tmp/onnxruntime.tgz

# Initialize rosdep (skip if already initialized in base image)
RUN rosdep init || true && rosdep update

WORKDIR /combat_ws

# Copy workspace source
COPY src ./src

# Resolve and install package dependencies declared in package.xml files.
# apt-get update is required again here — the apt index was cleared by
# the earlier RUN's "rm -rf /var/lib/apt/lists/*" cleanup step.
RUN apt-get update && \
    . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source both underlay and overlay on container start
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /combat_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]