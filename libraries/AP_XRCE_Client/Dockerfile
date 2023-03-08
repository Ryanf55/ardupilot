FROM amd64/ros:humble-ros-core

WORKDIR /ardupilot

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y && apt-get install --no-install-recommends -y \
    default-jre \
    git \
    ros-dev-tools

RUN rosdep init && rosdep update

SHELL ["/bin/bash", "-c"]

WORKDIR /ardupilot
RUN git clone -b develop --recurse-submodules https://github.com/eProsima/Micro-XRCE-DDS-Gen.git src/Micro-XRCE-DDS-GEN
WORKDIR /ardupilot/src/Micro-XRCE-DDS-GEN
RUN ./gradlew assemble
RUN echo "export PATH=\$PATH:/ardupilot/src/Micro-XRCE-DDS-Gen/scripts\n" >> ~/.bashrc

# Installing the micro-ROS build system
WORKDIR /ardupilot
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN rosdep install --from-paths src --ignore-src -y
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build

# # Creating the micro-ROS agent
RUN source install/setup.bash && ros2 run micro_ros_setup create_agent_ws.sh
RUN source install/setup.bash && ros2 run micro_ros_setup build_agent.sh
