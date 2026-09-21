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

# Copy only package.xml files first, preserving each package's directory
# structure. This means the dependency-resolution layer below only
# invalidates when a package.xml actually changes — not on every source
# edit — so `apt-get update` + `rosdep install` won't rerun on every build.
COPY src/common_msgs/package.xml ./src/common_msgs/package.xml
COPY src/perception_msgs/package.xml ./src/perception_msgs/package.xml
COPY src/perception/package.xml ./src/perception/package.xml
COPY src/fusion/package.xml ./src/fusion/package.xml
COPY src/heading_controller/package.xml ./src/heading_controller/package.xml
COPY src/bringup/package.xml ./src/bringup/package.xml
COPY src/combat_sim/package.xml ./src/combat_sim/package.xml

# Resolve and install package dependencies declared in package.xml files.
# This layer is cached across source-only changes, and now also pulls in
# combat_sim's deps (gazebo_ros, robot_state_publisher, xacro,
# teleop_twist_keyboard) via rosdep.
RUN apt-get update && \
    . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Now copy the full workspace source — this invalidates on every code
# change, but no longer forces a re-download of the package index above.
COPY src ./src

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source both underlay and overlay on container start
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /combat_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
