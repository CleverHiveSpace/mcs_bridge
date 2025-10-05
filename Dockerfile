# Use ROS2 Humble as base image
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    python3-dev \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
RUN useradd -m -s /bin/bash bridge_user && \
    usermod -aG sudo bridge_user

# Set up workspace
WORKDIR /workspace

# Copy requirements first for better caching
COPY requirements.txt .

# Install Python dependencies
RUN pip3 install --no-cache-dir -r requirements.txt

# Install ROS2 Python packages
RUN apt-get update && apt-get install -y \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Copy the application code and setup files
COPY mcs_bridge/ ./mcs_bridge/
COPY setup.py ./

# Install the package in development mode
RUN pip3 install -e .

# Create entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Change ownership to bridge_user
RUN chown -R bridge_user:bridge_user /workspace

# Switch to non-root user
USER bridge_user

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command (can be overridden)
CMD ["--bridge-type", "dummy", "--help"]
