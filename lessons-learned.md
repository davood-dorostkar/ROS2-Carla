## run carla server without extra windo
```
./CarlaUE4.sh -RenderOffScreen -prefernvidia
```

## visualize with rviz
after running your node/launch:
```
rviz2 -d config.rviz
```
## create a package
create a python package:
```
ros2 pkg create --build-type ament_python <package_name>
```
## setup.py
### executables
put your executables in a python file and then add it to `setup.py` like:
```py
entry_points={
    "console_scripts": ["simulator = testpkg.simulator:main"],
},
```

### add a launch file
if you have a launch file add it to `setup.py` inside the `data_files`:
```py
(os.path.join("share", package_name), glob("launch/*.launch.py")),
```

## launch file
here is a very good reference for this: [link](https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/)

### overall structure
```py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, EnvironmentVariable, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription
    ([
        #all descriptions
    ])
    return ld


if __name__ == "__main__":
    generate_launch_description()

```
### declare arguments
[arguments list](https://carla.readthedocs.io/projects/ros-bridge/en/latest/run_ros/#configuring-carla-settings)

you can set these in your `*.launch.py` if it is a python package. example:
```py
launch.actions.DeclareLaunchArgument(name="host", default_value="localhost"),
```
### add a node
first define the executable in `setup.py`:
```py
    entry_points={
        "console_scripts": [
            "<exec_name>=<package>.<file>:main"
        ], 
    },
```
then use it with the exec name defined before:
```py
from launch import LaunchDescription
from launch_ros.actions import Node
  
def generate_launch_description():
     
    node_var_name = Node(
        package="<package>",
        executable="<exec_name>",
        name='node_name'
    )
 
    return LaunchDescription([
        node_var_name,
    ])
```
### import other launches
you can include other launch files to your launch file:
```py
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'launch',
                    'example_substitutions_launch.py'
                ])
            ]),
        )
    ])
```
### path joins

- `PathJoinSubstitution` allows you to construct the path to a file. 
- To get the path to the package we use `FindPackageShare` which is also part of the substitutions module. 
- The result of PathJoinSubstitution will be like `<package>/rviz/view_robot.rviz`. It is then handed over as an argument to the Rviz Node.

The following example starts the visualizer Rviz with a config file that was constructed by the PathJoinSubstitution class:

```py
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
  
def generate_launch_description():
 
   rviz_config_file = PathJoinSubstitution(
           [FindPackageShare("<package>"), "rviz", "view_robot.rviz"]
   )
    
   return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),
   ])
```

### substitutions
Substitutions are convenient to make your launch file more flexible. Instead of hard coding all values you can get information from outside your launch file which is then substituted only when you execute it. That information can be `environment variables`, `launch configurations` or even `Python expressions`. 

- **env variables**

    **Example**:
    ```py
    from launch import LaunchDescription
    from launch.substitutions import EnvironmentVariable
    from launch_ros.actions import Node
    
    def generate_launch_description():
    return LaunchDescription([
        Node(
                package='turtlesim',
                executable='turtlesim_node',
                name=EnvironmentVariable('USER'),
        ),
    ])
    ```
- **ros configs**

    The `LaunchConfiguration` is used to access the values of launch arguments (`DeclareLaunchArgument`) during the execution of a launch file. These values can be provided:
    - As command-line arguments when the launch file is executed.
    - As default values specified in the launch file.
  
    The `LaunchConfiguration` object is a placeholder that represents the value of a launch argument, and it will be resolved (substituted) when the launch file is executed.

    **Example**:

    1. **Declare Launch Argument**:
    
    First, a launch argument is declared using `DeclareLaunchArgument`. 
    ```py
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration

    DeclareLaunchArgument(name="host", default_value="localhost")
    ```
    This defines an argument named `host` with a default value of `"localhost"`. If the user doesn't provide a value when launching, the default value will be used.

    2. **Use `LaunchConfiguration`**:
    
    Later in the launch file, when you need to use the value of this argument (e.g., to pass it to another node or launch file), you use `LaunchConfiguration` to retrieve the value:
    ```py
    LaunchConfiguration("host")
    ```
    This refers to the value of the `"host"` argument. If the user provided a value when launching (e.g., `--host 192.168.1.10`), then `LaunchConfiguration("host")` will evaluate to `"192.168.1.10"`. Otherwise, it will use the default value `"localhost"`.

    for example if you want to use it as argument to another imported launch file:
    ```py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('<this_or_other_pkg>'),
                        'launch', # package subfolder
                        '<other_launch_file>.py'
                    ])
                ]),
                launch_arguments={
                    "host": LaunchConfiguration("host"),
                }.items()
            )
    ```
### run a function as an action
you can also run a custom function (rather than a node or launch file) as part of the launch file. the function may return an action:
```py
def myfunc:
    return IncludeLaunchDescription(
        ###
    )

def generate_launch_description():
    ld = LaunchDescription
    ([
       OpaqueFunction(function=myfunc),
    ])
    return ld


if __name__ == "__main__":
    generate_launch_description()


```
## Required Components of a Simulation
these components are discussed in the following parts.
```
├── imports
├── set default arguments
├── add ROS bridge (carla_ros_bridge)
├── add objects (carla_spawn_objects)
├── add pygame visualizer (carla_manual_control)
└── other nodes and packages (control, planner, ...)
```
## Add ROS bridge (carla_ros_bridge)
```py
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [PathJoinSubstitution([FindPackageShare("carla_ros_bridge"), "carla_ros_bridge.launch.py"])]
    ),
    launch_arguments={
        "host": LaunchConfiguration("host"),
        "port": LaunchConfiguration("port"),
        "town": LaunchConfiguration("town"),
        "timeout": LaunchConfiguration("timeout"),
        "passive": LaunchConfiguration("passive"),
        "synchronous_mode": LaunchConfiguration("synchronous_mode"),
        "synchronous_mode_wait_for_vehicle_control_command": LaunchConfiguration(
            "synchronous_mode_wait_for_vehicle_control_command"
        ),
        "fixed_delta_seconds": LaunchConfiguration("fixed_delta_seconds"),
    }.items(),
),
```
## Add Objects to Simulation (carla_spawn_objects)
the package responsible for this in the ROS-CARLA-Bridge is `carla_spawn_objects`. you can select the initial definitions like the car select, sensors attached to car, and other actors in a `.json` file.
1. first add the file to `setup.py`:
```py
data_files=[
        ...
        ("share/" + package_name + "/config", ["config/objects.json"]),
        ...
]
```
2. pass the config file to spawn objects launch file:
```py
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [
            PathJoinSubstitution(
                [FindPackageShare("carla_spawn_objects"), "carla_spawn_objects.launch.py"]
            )
        ]
    ),
    launch_arguments={
        "host": LaunchConfiguration("host"),
        "port": LaunchConfiguration("port"),
        "timeout": LaunchConfiguration("timeout"),
        "vehicle_filter": LaunchConfiguration("vehicle_filter"),
        "role_name": LaunchConfiguration("role_name"),
        "spawn_point": LaunchConfiguration("spawn_point"),
        "objects_definition_file": PathJoinSubstitution(
            [FindPackageShare("testpkg"), "config/objects.json"]
        ),
        "spawn_point_ego_vehicle": LaunchConfiguration("spawn_point"),
    }.items(),
),
```
3. define the file like this:
```json
{
    "objects": [
        {
            ///
        },
        {
            ///
        }
    ]
}
```
4. a reference for sections of the config file:

    [simplified version](/src/testpkg/config/objects.json) 
    
    [full version](/src/ros-bridge/carla_spawn_objects/config/objects.json)

    [list of all cars](https://carla.readthedocs.io/en/latest/catalogue_vehicles/)

5. if you want to visualize the sim in a pygame window, you need to have a camera with `"id": "rgb_view"` in the config file. set its resolution to that of the manual_control node that you import into your simulation (your launch and setup.py file). the default visualization package for this is `carla_manual_control` in the ROS-CARLA-Bridge.
6. having a lot of sensors, especially camera and lidar sensors can significantly reduce the simulation speed, so you can remove unnecessary sensors to improve the simulation speed.

## Add pygame Visualizer (carla_manual_control)
this package is responsible for visalizing the simulation, and get keyboard inputs. 
- in the main loop, the `ManualControl` class render method is called to update the visualization.
  
  >if you want to change the screen size, change it in the main loop. 
- this render updates other rendering tasks as well.
- a subscriber is defined to get the values of `rgb_view` camera which is the main camera.
- other subsribers are also defined to update other data. 
- odometry, gnss, and other data are updated and showed with `HUD` class.
  
  >if you want to change the font size and other attributes of the HUD, change them in the class's `init` or `render` method.
  
  to change the font size:
  ```py
  self._font_mono = pygame.font.Font(mono, 10) # change 10
  ```
  to change the HUD gray background width:
  ```py
  info_surface = pygame.Surface((220, self.dim[1])) # change 220
  ````