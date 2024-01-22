# export CARLA_ROOT=/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
export SCENARIO_RUNNER_ROOT=/home/scenario_runner-0.9.13

source /opt/ros/$ROS_DISTRO/setup.bash
source ./install/setup.bash
