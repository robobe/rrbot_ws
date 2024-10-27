FROM humble:dev

# ** Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
      # python package
      # ros packages
      ros-humble-xacro \
      ros-humble-joint-state-publisher-gui \
      # ros-humble-cv-bridge \
      # ros-humble-rosbridge-suite \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-gazebo-ros2-control \
      ros-humble-rviz2 \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog


# COPY .devcontainer/requirements.txt requirements.txt
# RUN pip3 install -r requirements.txt