FROM ros:jazzy-ros-base

# Build tools, test tools, and the Python libraries of the perception and web packages
# (all from Ubuntu's own repositories: no pip, reproducible versions).
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git libgtest-dev \
      python3-colcon-common-extensions python3-pip python3-pytest \
      python3-opencv python3-fastapi python3-uvicorn python3-websockets python3-httpx \
    && rm -rf /var/lib/apt/lists/*

# Each rclcpp source file needs ~0.5-1 GB RAM to compile; the laptop's Docker VM
# has 4 GB, so limit parallel compile jobs (colcon/make read MAKEFLAGS).
ENV MAKEFLAGS=-j2

WORKDIR /ws
COPY src/ src/
COPY scripts/ scripts/

# Build and test with the same commands you use locally.
RUN . /opt/ros/jazzy/setup.sh && colcon build --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# Operator web UI (only when launched with web:=true): docker run -p 8080:8080 ...
EXPOSE 8080
CMD ["bash", "-c", ". /opt/ros/jazzy/setup.sh && colcon test --event-handlers console_direct+ && colcon test-result --verbose"]
