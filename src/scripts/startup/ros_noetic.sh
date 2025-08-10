#!/bin/bash

echo "ROS internet üzerinden yayın yapıyor."

IP=$(hostname -I | awk '{print $1}')
export ROS_HOSTNAME=$IP
export ROS_MASTER_URI=http://$IP:11311

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

echo "  ROS_HOSTNAME=$ROS_HOSTNAME"
echo "  ROS_MASTER_URI=$ROS_MASTER_URI"

if ! rostopic list > /dev/null 2>&1; then
    echo "roscore başlatılıyor..."
    roscore &
    sleep 2
else
    echo "roscore zaten çalışıyor."
fi

for package_dir in ~/catkin_ws/src/*/
do
    if [ ! -f "${package_dir}/CMakeLists.txt" ]; then
        continue
    fi
    script_dir="${package_dir}/scripts"
    if [ -d "$script_dir" ]; then
        echo "  $(basename "$package_dir") içindeki script'ler kontrol ediliyor..."
        find "$script_dir" -type f -name "*.py" -exec chmod +x {} \;
    fi
done

echo "Tüm sistem başlatılıyor..."
roslaunch master.launch
