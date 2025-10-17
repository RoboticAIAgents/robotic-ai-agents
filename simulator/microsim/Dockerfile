# Optional custom Dockerfile if you want to pre-build the workspace
# Use this if you want a ready-to-run image instead of building in container

FROM osrf/ros:humble-desktop-full

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install numpy pyyaml

# Create workspace
WORKDIR /workspace

# Copy package files (if building image with code baked in)
# COPY . /workspace/

# Build the package (uncomment if baking into image)
# RUN source /opt/ros/humble/setup.bash && \
#     colcon build --symlink-install

# Source ROS in bashrc for convenience
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc

# Default command
CMD ["bash"]
