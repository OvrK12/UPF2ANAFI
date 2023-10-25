# UPF2ANAFI
This repository contains installation instructions and the required code to run the [REAP-Framework](https://github.com/UniBwM-IFS-AILab/REAP) for Parrot ANAFI drones (tested with ANAFI AI, but setup should work for other ANAFI models as well).

![anafi](https://github.com/UniBwM-IFS-AILab/UPF2ANAFI/assets/92592126/2d215490-cee6-4adc-a424-6f5316cdd93e)

* [System Requirements](#system-requirements)
* [System Architecture](#system-architecture)
* [Installation](#installation)
  + [AI Planning Setup]
  + [Parrot Sphinx]
  + [Parrot Olympe]
  + [anafi_ros Bridge]
  + [UPF2ANAFI Action Server]
* [Usage](#usage)
* [Roadmap](#roadmap)


## System Requirements

 - Ubuntu 20.04 or Ubuntu 22.04. The Parrot Sphinx simulation environment supports only these operating systems. It is also not possible to run the simulation environment in a WSL instance or a virtual machine, you need a native Ubuntu installation. Also a setup on a Debian 11 system is possible, but this was not tested.
- Linux Kernel Version >= 5.8. At the time of writing, kernel versions >= 6.0 caused a bug when closing the Sphinx simulation environment, which made restarting the simulation environment impossible. This issue could only be fixed by restarting the machine. However, this probably should get fixed by Parrot in the future. We tested with a 5.19 kernel
- ROS2 Humble (other versions might also work, but this is the recommended version for the anafi_ros bridge)

## System Architecture
Below you can find an overview of the system architecture. If you compare the architecture to the [REAP-Framework](https://github.com/UniBwM-IFS-AILab/REAP) with open-source components, you can see that the AI-Planning side remains untouched. The Validation & Visualization side is completely replaced with Parrot specific components. External environment manipulation via Remote Control API or AirSim API is not possible with this setup (however, there might be ways to achieve environment manipulation with other tools, but this was not tested). QGroundControl still works, but Parrot allows only for one controller at the same time. This means that ...

![System_diagram_layered drawio](https://github.com/UniBwM-IFS-AILab/UPF2ANAFI/assets/92592126/8353f818-a099-4580-8d25-95759a1f1d8b)

  
## Installation
## Usage

ros2 launch upf2anafi anafi_offboard.launch.py

## Roadmap
