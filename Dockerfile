FROM osrf/ros:foxy-desktop
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy
ENV USERNAME=ros
ENV CARLA_ROOT=/opt/carla
USER root

SHELL ["/bin/bash", "-c"]
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    wget \
    unzip \
    coinor-libipopt-dev \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-derived-object-msgs \
    ros-${ROS_DISTRO}-ackermann-msgs \
    vim \
    psmisc

COPY src/ /app/src/
COPY scripts/ /app/scripts/
COPY source_env.sh /app/
WORKDIR /app/scripts/
RUN bash install_yaml.sh && \
    bash install_adolc.sh && \
    bash install_ipopt.sh && \
    bash install_osqp.sh && \
    bash install_qp_oases.sh

COPY scenario_runner-0.9.13/requirements.txt /app/
RUN pip3 install -r /app/requirements.txt && \
    pip3 install empy lark pygame transforms3d pexpect Pillow && \
    rm -rf ~/.cache/pip

WORKDIR /app/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths /app/src --ignore-src -y && \
    colcon build --packages-skip test_controller

RUN apt-get clean && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /app/src /app/scripts /app/source_env.sh /app/requirements.txt \
    /var/lib/apt/lists/* \
    /tmp/*

RUN useradd -m -G sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    chown -R $USERNAME:$USERNAME /app

USER $USERNAME
ENV PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
RUN echo 'export PYTHONPATH=$PYTHONPATH:$(find $CARLA_ROOT/PythonAPI/carla/dist -name "carla-*-linux-x86_64.egg" | paste -sd: -)' >> ~/.bashrc && \
    echo 'export USER=$USERNAME' >> ~/.bashrc && \
    echo "source /ros_entrypoint.sh" >> ~/.bashrc && \
    echo "source /app/source_env.sh" >> ~/.bashrc && \
    echo 'set +e' >> ~/.bashrc && \
    echo 'alias python="python3"' >> ~/.bashrc
WORKDIR /app