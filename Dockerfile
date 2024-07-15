FROM osrf/ros:foxy-desktop
ENV DEBIAN_FRONTEND=noninteractive
USER root

SHELL ["/bin/bash", "-c"]
RUN apt-get update && \
    apt-get install -y python3-pip wget unzip 

COPY src/ /app/src/
COPY scripts/ /app/scripts/
COPY source_env.sh /app/
WORKDIR /app/scripts/

RUN bash install_yaml.sh
RUN bash install_adolc.sh
RUN bash install_ipopt.sh
RUN bash install_osqp.sh
RUN bash install_qp_oases.sh

RUN pip3 install empy lark pygame transforms3d pexpect Pillow
RUN apt-get update && apt-get install -y coinor-libipopt-dev 
RUN apt-get update && apt-get install -y python3-colcon-common-extensions
# ENV CARLA_ROOT=/carla
# ENV PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
# ENV SCENARIO_RUNNER_ROOT=/app/scenario_runner-0.9.13
# COPY PythonAPI/ /carla/PythonAPI

WORKDIR /app/
RUN sudo apt-get update && sudo apt-get install -y  \
    ros-${ROS_DISTRO}-derived-object-msgs \
    ros-${ROS_DISTRO}-ackermann-msgs
RUN source /opt/ros/foxy/setup.bash && rosdep update
RUN source /opt/ros/foxy/setup.bash && rosdep install --from-paths src --ignore-src -y
RUN source /opt/ros/foxy/setup.bash && colcon build --packages-skip test_controller
RUN sudo apt-get update && sudo apt-get install -y vim psmisc

COPY scenario_runner/requirements.txt /app/
RUN pip3 install -r /app/requirements.txt

RUN apt-get clean && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf src scripts source_env.sh \
    /carla/PythonAPI \
    /var/lib/apt/lists/*

ARG USERNAME=ros
RUN useradd  -m -G sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
RUN chown -R ros:ros /app
USER $USERNAME
ENV PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
ENV SCENARIO_RUNNER_ROOT=/app/scenario_runner-0.9.13
RUN echo 'export USER=$USERNAME' >> ~/.bashrc 
RUN echo "source /ros_entrypoint.sh" >> ~/.bashrc
RUN echo "source /app/source_env.sh" >> ~/.bashrc