FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04
WORKDIR /root

RUN apt update && \
        apt -y install python3.11 python3-pip curl wget

RUN mkdir -p ~/miniconda3
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
RUN bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
RUN rm ~/miniconda3/miniconda.sh

ENV TZ=Europe/Budapest \
    DEBIAN_FRONTEND=noninteractive

RUN apt update && \
        apt -y install \
        libgl-dev \
     && rm -rf /var/lib/apt/lists/*

RUN . ~/miniconda3/bin/activate && \
        conda install python=3.11 

RUN . ~/miniconda3/bin/activate && \
        conda install mamba -c conda-forge

RUN . ~/miniconda3/bin/activate && \
        conda config --env --add channels conda-forge && \
        conda config --env --add channels robostack-staging && \
        conda config --env --add channels robostack-jazzy 

RUN . ~/miniconda3/bin/activate && \
        mamba install ros-jazzy-ros-base ros-jazzy-rviz2

RUN . ~/miniconda3/bin/activate && \
        mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep

RUN echo ". ~/miniconda3/bin/activate" >> /root/.bashrc

COPY requirements.txt .
RUN bash -i -c "pip3 install -r requirements.txt"

RUN mkdir -p ros_ws/src
COPY src ros_ws/src

WORKDIR /root/ros_ws

COPY *.pt .
COPY docker_trick.xml .
COPY start.sh .

RUN /bin/bash -i -c "colcon build"

#CMD ["/bin/bash"]
CMD ["/bin/bash" , "start.sh"]
EXPOSE 80
