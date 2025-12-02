#! /bin/bash

source /home/pi/oradar_ws/devel/setup.bash

gnome-terminal -x bash -c "roslaunch auto_wheelchair wheelchair.launch"
wait
exit 0
