#! /usr/bin/env bash

cd 
cd ardupilot
. ~/.profile

cd estuiha_gazebo

clear

BLUE="\e[36m"
GREEN="\e[32m"
ENDCOLOR="\e[0m"
GREEN_LIGHT="\e[92m"
RED="\e[91m"

port_list=("${RED}{ENDCOLOR} ${GREEN_LIGHT}
                                                                            



	
SIHA UDP Ports 
Main UAV ${BLUE} 14700 14705 ${ENDCOLOR}
Second UAV ${BLUE} 14710 14715 ${ENDCOLOR}

Web Server Ports ${GREEN}"http://0.0.0.0:8080/stream?topic=/cessna/image_raw"${ENDCOLOR}
")
echo -e "$port_list"

PS3='Choose your competition:'
opt=("UIHA" "SIHA")
select opt in "${opt[@]}"
do
    case $opt in
    "UIHA")
    	map=uiha
    	screen -S vehicle -d -m bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris --add-param-file ~/estuiha_gazebo/full_tuned.parm -m --mav20 -I0 --out=127.0.0.1:14700 --out=127.0.0.1:14705 --console --map"
	break
	;;
    "SIHA")
    	map=siha
    	screen -S vehicle -d -m bash -c "sim_vehicle.py -v ArduPlane -f gazebo-zephyr -m --mav20 -I0 --out=127.0.0.1:14700 --out=127.0.0.1:14705 --console --map --add-param-file ~/estuiha_gazebo/params/cessna.parm"
		sleep 5
    	screen -S vehicle2 -d -m bash -c "sim_vehicle.py -v ArduPlane -f gazebo-zephyr -m --mav20 -I1 --out=127.0.0.1:14710 --out=127.0.0.1:14715 --console --map --add-param-file ~/estuiha_gazebo/params/zephyr.parm"
	break
	;;
    esac
done
screen -S server -d -m bash -c "rosrun web_video_server web_video_server"
screen -S simulation -T -d -m bash -c "roslaunch ~/estuiha_gazebo/worlds/launcher.launch world:=$map.world"
killall screen
pkill -9 -f mavproxy.py  
pkill -9 -f arduplane
pkill -9 -f arducopter
