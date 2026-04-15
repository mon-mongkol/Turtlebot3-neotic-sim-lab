FROM osrf/ros:noetic-desktop-full

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# ── Install TurtleBot3 + SLAM (gmapping) + navigation packages ──
RUN apt-get update && apt-get install -y \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-turtlebot3-slam \
    ros-noetic-turtlebot3-navigation \
    ros-noetic-turtlebot3-teleop \
    ros-noetic-gmapping \
    ros-noetic-slam-gmapping \
    ros-noetic-navigation \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-amcl \
    ros-noetic-dwa-local-planner \
    ros-noetic-teb-local-planner \
    ros-noetic-rviz \
    ros-noetic-rosbridge-server \
    ros-noetic-tf2-web-republisher \
    ros-noetic-robot-localization \
    ros-noetic-sbpl \
    ros-noetic-sbpl-lattice-planner \
    python3-pip \
    git \
    wget \
    nano \
 && rm -rf /var/lib/apt/lists/*

# ── Copy launch scripts ──
COPY scripts/ /root/scripts/
RUN chmod +x /root/scripts/*.sh

# ── Set default TurtleBot3 model ──
ENV TURTLEBOT3_MODEL=burger

# ── Source ROS setup in every bash session ──
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc

WORKDIR /root

# ── Default command ──
CMD ["/bin/bash"]
