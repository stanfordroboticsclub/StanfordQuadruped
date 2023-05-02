FROM ros:noetic-ros-base

RUN apt-get update &&\
  apt-get install -y ros-$ROS_DISTRO-rosserial-arduino ros-$ROS_DISTRO-rosserial git &&\
  apt-get -y clean &&\
  apt-get -y purge &&\
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create a Catkin Workspace
SHELL ["/bin/bash", "-c"]
ENV CATKIN_WS /catkin_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash &&\
  mkdir -p $CATKIN_WS/src &&\
  cd $CATKIN_WS/ &&\
  catkin_make

# Build ROS Serial
RUN source /opt/ros/$ROS_DISTRO/setup.bash &&\
  cd $CATKIN_WS/src &&\
  git clone https://github.com/ros-drivers/rosserial.git &&\
  cd $CATKIN_WS &&\
  catkin_make &&\
  catkin_make install

# Create ROS Serial Arduino builder
RUN source /opt/ros/$ROS_DISTRO/setup.bash &&\
  cd /tmp &&\
  rosrun rosserial_arduino make_libraries.py .
