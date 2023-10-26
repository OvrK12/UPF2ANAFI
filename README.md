# UPF2ANAFI
This repository contains installation instructions and the required code to run the [REAP-Framework](https://github.com/UniBwM-IFS-AILab/REAP) for Parrot ANAFI drones (tested with ANAFI AI, but setup should work for other ANAFI models as well).

![anafi](https://github.com/UniBwM-IFS-AILab/UPF2ANAFI/assets/92592126/2d215490-cee6-4adc-a424-6f5316cdd93e)

* [System Requirements](#system-requirements)
* [System Architecture](#system-architecture)
* [Installation](#installation)
  + [AI Planning Setup](#ai-planning-setup)
  + [Parrot Sphinx](#parrot-sphinx)
  + [Parrot Olympe](#parrot-olympe)
  + [anafi_ros Bridge](#anafi_ros-bridge)
  + [UPF2ANAFI Action Server](#upf2anafi-action-server)
* [Usage](#usage)
* [Roadmap](#roadmap)


## System Requirements

 - Ubuntu 20.04 or Ubuntu 22.04. The Parrot Sphinx simulation environment supports only these operating systems. **It is also not possible to run the simulation environment in a WSL instance or a virtual machine, you need a native Ubuntu installation**. Also a setup on a Debian 11 system is possible, but this was not tested.
- Linux Kernel Version >= 5.8. At the time of writing, kernel versions >= 6.0 caused a bug when closing the Sphinx simulation environment, which made restarting the simulation environment impossible. This issue could only be fixed by restarting the machine. However, this probably should get fixed by Parrot in the future. We tested with a 5.19 kernel
- ROS2 Humble (other versions might also work, but this is the recommended version for the anafi_ros bridge)

Minimal system requirements for running the Parrot Sphinx simulation environment:

- 8-core CPU @ 2GHz
- 8 GB RAM
- Nvidia GPU with 2GB GDDR (with up-to-date Nvidia drivers)
- 15 GB of free disk space

    


## System Architecture
Below you can find an overview of the system architecture. If you compare the architecture to the [REAP-Framework](https://github.com/UniBwM-IFS-AILab/REAP) with open-source components, you can see that the AI-Planning side remains untouched. The Validation & Visualization side is completely replaced with Parrot specific components. External environment manipulation via Remote Control API or AirSim API is not possible with this setup (there might be ways to achieve environment manipulation with other tools, but this was not tested). QGroundControl still works, but Parrot allows only for one controller at the same time. This means that you can track the position of the drone on QGroundControl, but can't override the Parrot flight controller (e.g. by uploading a custom flight plan on QGroundControl).


![System_diagram_layered drawio](https://github.com/UniBwM-IFS-AILab/UPF2ANAFI/assets/92592126/8353f818-a099-4580-8d25-95759a1f1d8b)

  
## Installation
Following you can find all required steps to setup the REAP Framework for ANAFI drones. **All components must be installed on a native Ubuntu installation**.

### AI Planning Setup
For the setup of the AI planning component follow the [same steps](https://github.com/UniBwM-IFS-AILab/REAP/edit/main/README.md#setup-of-the-ai-planning-component) as given in the original REAP repository.

### Parrot Sphinx
Parrot Sphinx is a proprietary simulation environment for Parrot ANAFI drones. It replaces the Unreal Engine with AirSim plugin in the original REAP-framework. Installation instructions for Sphinx can be found here: https://developer.parrot.com/docs/sphinx/installation.html


Sphinx comes with a range of prebuilt environments which can be used for general testing. 
```
<! get a list of available prebuilt environments>
apt-cache search parrot-ue4
<! install a prebuilt environment e.g. parrot-ue4-empty>
sudo apt install parrot-ue4-empty
```
Sphinx is based on the Unreal Engine and it is possible to import custom environments built in the Unreal Engine (e.g. imported LIDAR point clouds). However, this was not tested yet. For instructions see: https://developer.parrot.com/docs/sphinx/build_ue_editor.html

### Parrot Olympe
Parrot Olympe is an SDK that provides a Python controller programming interface for Parrot ANAFI drones. Olympe is required to connect the anafi_ros bridge to the Parrot flightcontroller. Documentation for Olympe can be found here: https://developer.parrot.com/docs/olympe/olympeapi.html.
The documentation seems to be only available for the latest version of Olympe, but the anafi_ros bridge requires an Olympe version 7.5
```
pip install parrot-olympe==7.5.0
```
### anafi_ros Bridge
The anafi_ros bridge converts ROS2 commands into commands for the Parrot flightcontroller using the Parrot Olympe SDK. Installation instructions can be found under: https://github.com/andriyukr/anafi_ros
You can customize the simulated drone by changing the settings under *anafi_ros_nodes/config*. Also the drone flies quite slow per default. You can change the max speed by editing the following lines in *anafi_ros_nodes/anafi.py*. Change the respective max speed of 1 m/s according to your preferences.

```
self.node.declare_parameter("drone/max_vertical_speed", 1.0, ParameterDescriptor(description="Max vertical speed (in m/s) [0.1, 4.0]",floating_point_range=[FloatingPointRange(from_value=0.1,to_value=4.0,step=0.0)]))
self.node.declare_parameter("drone/max_horizontal_speed", 1.0,  ParameterDescriptor(description="Max horizontal speed (in m/s) [0.1, 15.0]", floating_point_range=[FloatingPointRange(from_value=0.1,to_value=15.0,step=0.0)]))
```
### UPF2ANAFI Action Server
Install the UPF2ANAFI action server by cloning the code from this repository and following these steps:
```
mkdir UPF2ANAFI
cd UPF2ANAFI
mkdir src
cd src
git clone https://github.com/UniBwM-IFS-AILab/UPF2ANAFI.git
cd ..
colcon build
```

> [!WARNING]  
> The execute_fly callback does not work properly because the initial coordinates of the drones are not set correctly. For example, passing the target coordinates (47.23, 53.10) to the drone results in very long flight times because the drone starts at an arbitrary start position (like 0.0, 0.0). It needs to be investigated how to properly set the initial GPS start position of the drone.

## Usage
Start all components in seperate tabs/command line windows using following commands:

```
<! starts the Sphinx simulation environment e.g. with the "parrot-ue4-landscape-mountains" environment>
sphinx --log-level=dbg /opt/parrot-sphinx/usr/share/sphinx/drones/anafi_ai.drone::wifi_iface=::firmware=\"https://firmware.parrot.com/Versions/anafi2/pc/%23latest/images/anafi2-pc.ext2.zip\" & parrot-ue4-landscape-mountains

<! starts the anafi_ros bridge>
cd <anafi_ros_bridge repo>; source install/local_setup.bash; ros2 launch anafi_ros_nodes anafi_launch.py namespace:='anafi' ip:='10.202.0.1' model:='ai'

<! starts the UPF2ANAFI action server. UPF2ANAFI requires msg types from anafi_ros which is why anafi_ros has to be sourced before starting UPF2ANAFI>
cd <anafi_ros_bridge repo>; source install/local_setup.bash; cd <UPF2ANAFI repo>; source install/local_setup.bash; ros2 launch upf2anafi anafi_offboard.launch.py count:=1

<! starts the upf4ros2 main application>
cd <plansys repo>; source install/local_setup.bash; ros2 launch upf4ros2 upf4ros2.launch.py

<! starts the upf4ros2 plan executor>
cd <plansys repo>; source install/local_setup.bash; ros2 launch upf4ros2_demo traverse_areas.launch.py count:=1
```

The *start_anafi.sh* script which is part of this repository automates all these steps. However, you will have to adjust the paths in the script to your machine.
## Roadmap
- landing function has to be implemented
- set correct GPS start location of the drone
- test if critical system functionality (such as when losing communication to ground control station) can be overwritten. This is important to realize fully autonomous drones
- try to import custom Unreal Engine environments (e.g. generated by LIDAR point cloud) in Sphinx


