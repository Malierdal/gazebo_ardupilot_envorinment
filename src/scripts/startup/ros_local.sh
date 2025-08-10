#!/bin/bash

echo "ROS yerel ağ üstünden yayın yapıyor."

export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

echo "  ROS_HOSTNAME=$ROS_HOSTNAME"
echo "  ROS_MASTER_URI=$ROS_MASTER_URI"

if ! pgrep -x "roscore" > /dev/null
then
    echo "roscore başlatılıyor..."
    ( roscore ) &
    sleep 2
else
    echo "roscore zaten çalışıyor."
fi

echo "Launch dosyaları taranıyor..."

for package_dir in ~/catkin_ws/src/*/
do
    package_name=$(basename "$package_dir")
    
    if [ "$package_name" = "build" ]; then
        echo "Skipping build directory..."
        continue
    fi

    launch_file=~/catkin_ws/src/${package_name}/launch/${package_name}.launch

    if [ -f "$launch_file" ]; then
        echo "$package_name başlatılıyor..."
        ( roslaunch ${package_name} ${package_name}.launch ) &
        sleep 1
    else
        echo "$package_name için launch dosyası bulunamadı."
    fi
done

wait