#!/bin/bash

tab="--tab"
cmd_sphinx_empty="sphinx --log-level=dbg /opt/parrot-sphinx/usr/share/sphinx/drones/anafi_ai.drone::wifi_iface=::firmware=\"https://firmware.parrot.com/Versions/anafi2/pc/%23latest/images/anafi2-pc.ext2.zip\" & parrot-ue4-empty -gps-json='{\"lat_deg\":48.027342, \"lng_deg\":11.874431, \"elevation\":0.0}'"
cmd_sphinx_forest="sphinx --log-level=dbg /opt/parrot-sphinx/usr/share/sphinx/drones/anafi_ai.drone::wifi_iface=::firmware=\"https://firmware.parrot.com/Versions/anafi2/pc/%23latest/images/anafi2-pc.ext2.zip\" & parrot-ue4-forest -gps-json='{\"lat_deg\":48.027342, \"lng_deg\":11.874431, \"elevation\":0.0}'"
cmd_sphinx_mountain="sphinx --log-level=dbg /opt/parrot-sphinx/usr/share/sphinx/drones/anafi_ai.drone::wifi_iface=::firmware=\"https://firmware.parrot.com/Versions/anafi2/pc/%23latest/images/anafi2-pc.ext2.zip\" & parrot-ue4-landscape-mountains -gps-json='{\"lat_deg\":48.027342, \"lng_deg\":11.874431, \"elevation\":0.0}'"
cmd_bridge="cd; cd anafi_ros_bridge; source install/local_setup.bash; ros2 launch anafi_ros_nodes anafi_launch.py namespace:='anafi' ip:='10.202.0.1' model:='ai'"
cmd_anafi_offboard="cd; cd anafi_ros_bridge; source install/local_setup.bash; cd; cd Repos/UPF2ANAFI; source install/local_setup.bash; ros2 launch upf2anafi anafi_offboard.launch.py count:=1"
cmd_upf4ros_server="cd; cd plansys; source install/local_setup.bash; ros2 launch upf4ros2 upf4ros2.launch.py"
cmd_upf4ros_traverse="cd; cd plansys; source install/local_setup.bash; ros2 launch upf4ros2_demo traverse_areas.launch.py count:=1"


# Create an array containing both commands
cmd_array=("$cmd_sphinx_mountain" "$cmd_bridge" "$cmd_anafi_offboard" "$cmd_upf4ros_server" "$cmd_upf4ros_traverse")

for cmd in "${cmd_array[@]}"
do
    gnome-terminal -- bash -c "$cmd; exec bash"
    sleep 5
done

exit 0
