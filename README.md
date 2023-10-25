# UPF2ANAFI
This repository contains installation instructions and the required code to run the [REAP-Framework](https://github.com/UniBwM-IFS-AILab/REAP) for Parrot ANAFI drones (tested with ANAFI AI, but setup should work for other ANAFI models as well).

![anafi](https://github.com/UniBwM-IFS-AILab/UPF2ANAFI/assets/92592126/2d215490-cee6-4adc-a424-6f5316cdd93e)


* [System Architecture](#system-architecture)
* [System Requirements](#system-requirements)
* [Usage](#usage)

## System Architecture
Below you can find an overview of the system architecture. If you compare the architecture to the [REAP-Framework](https://github.com/UniBwM-IFS-AILab/REAP) with open-source components, you can see that the AI-Planning side remains untouched. The Validation & Visualization side is completely replaced with Parrot specific components. External environment manipulation via Remote Control API or AirSim API is not possible with this setup (however, there might be ways to achieve environment manipulation with other tools, but this was not tested). QGroundControl still works, but Parrot allows only for one controller at the same time. This means that ...

![System_diagram_layered drawio](https://github.com/UniBwM-IFS-AILab/UPF2ANAFI/assets/92592126/8353f818-a099-4580-8d25-95759a1f1d8b)

## System Requirements
## Usage

ros2 launch upf2anafi anafi_offboard.launch.py
