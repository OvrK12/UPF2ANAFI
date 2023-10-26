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
### anafi_ros Bridge
### UPF2ANAFI Action Server

## Usage

```
<! start the Sphinx simulation environment e.g. with the "parrot-ue4-landscape-mountains" environment>
sphinx --log-level=dbg /opt/parrot-sphinx/usr/share/sphinx/drones/anafi_ai.drone::wifi_iface=::firmware=\"https://firmware.parrot.com/Versions/anafi2/pc/%23latest/images/anafi2-pc.ext2.zip\" & parrot-ue4-landscape-mountains

<! start the Sphinx simulation environment e.g. with the "parrot-ue4-landscape-mountains" environment>
cd <anafi_ros_bridge repo>; source install/local_setup.bash; ros2 launch anafi_ros_nodes anafi_launch.py namespace:='anafi' ip:='10.202.0.1' model:='ai'

<! start the Sphinx simulation environment e.g. with the "parrot-ue4-landscape-mountains" environment>
cd <anafi_ros_bridge repo>; source install/local_setup.bash; cd <UPF2ANAFI repo>; source install/local_setup.bash; ros2 launch upf2anafi anafi_offboard.launch.py count:=1

<! start the Sphinx simulation environment e.g. with the "parrot-ue4-landscape-mountains" environment>
cd <plansys repo>; source install/local_setup.bash; ros2 launch upf4ros2 upf4ros2.launch.py

<! start the Sphinx simulation environment e.g. with the "parrot-ue4-landscape-mountains" environment>
cd <plansys repo>; source install/local_setup.bash; ros2 launch upf4ros2_demo traverse_areas.launch.py count:=1
```

The *start_anafi.sh* script which is part of this repository automates all these steps. However, you will have to adjust the paths in the script to your machine.
## Roadmap
- test if critical system functionality (such as when losing communication to ground control station) can be overwritten. This is important to realize fully autonomous drones
- implement remaining functions like landing, replanning
- set correct GPS start location of the drone
- try to import custom Unreal Engine environments (e.g. generated by LIDAR point cloud) in Sphinx


