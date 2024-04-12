FROM nvidia/cuda:12.4.1-runtime-ubuntu22.04
WORKDIR /root

RUN apt update && \
        apt -y install python3-pip curl

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

ENV TZ=Europe/Budapest \
    DEBIAN_FRONTEND=noninteractive

RUN apt update && \
        apt -y install \
        ros-humble-ros-base \
        ros-humble-rviz2 \
        python3-colcon-common-extensions \
        libgl-dev

COPY requirements.txt .
RUN pip3 install -r requirements.txt

RUN echo ". /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN mkdir -p ros_ws/src
COPY pose_estimation/ ros_ws/src/

WORKDIR /root/ros_ws
RUN colcon build

COPY start.sh .

CMD ["/bin/bash", "start.sh"]
EXPOSE 80