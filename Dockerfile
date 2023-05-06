FROM osrf/ros:noetic-desktop-full

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gdb \
    apt-utils \
    python3-rosdep \
    python3-pip \
    python3-vcstool \
    python3-pymodbus \
    build-essential \
    ros-noetic-catkin \
    python3-catkin-tools \
    ros-noetic-ros-controllers \
    nano \
    ros-noetic-soem \
    libvlccore-dev \
    libvlc-dev \
    ros-noetic-joy \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    git 

RUN pip3 install \
    #Following are from pupper code
    transforms3d \
    UDPComms \
    serial \
    pyserial \
    pigpio \
    regex \
    matplotlib \
    #Following are Nathan/Alex additions
    pynput \
    spidev \
    #adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-servokit

# Make the prompt a little nicer
RUN echo "PS1='${debian_chroot:+($debian_chroot)}\u@:\w\$ '" >> /etc/bash.bashrc 

WORKDIR /dingo_ws
COPY /dingo_ws/src /dingo_ws/src
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash &&\
catkin_make --directory /dingo_ws -DCMAKE_BUILD_TYPE=Debug'

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "source /dingo_ws/devel/setup.bash" >> /etc/bash.bashrc

ENTRYPOINT ["./ros_entrypoint.sh"]
CMD [ "bash" ]



