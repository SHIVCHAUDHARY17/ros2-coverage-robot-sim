FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git libgtest-dev \
      python3-colcon-common-extensions python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/ src/

# Build and test with the same commands you use locally.
RUN . /opt/ros/jazzy/setup.sh && colcon build --event-handlers console_direct+
CMD ["bash", "-c", ". /opt/ros/jazzy/setup.sh && colcon test --event-handlers console_direct+ && colcon test-result --verbose"]
