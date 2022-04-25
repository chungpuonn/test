#! /bin/sh

CURRENT_USER=$(whoami)
ROS_DISTRO=$(rosversion -d)

# replace ros_distro version and username
for files in  *.service; do
sed -i 's@ros_distro_version@'"$ROS_DISTRO"'@g' ${files};
sed -i 's@USERNAME@'"$CURRENT_USER"'@g' ${files};
done

# sudo cp roscore.service /etc/systemd/system/roscore.service
# sudo cp env.sh /etc/ros/env.sh
# sudo cp roslaunch.service /etc/systemd/system/roslaunch.service
# sudo cp roslaunch /usr/sbin/roslaunch

# revert the changes back to slove git issues
for files in  *.service; do
sed -i 's/'"$ROS_DISTRO"'/ros_distro_version/g' ${files};
sed -i 's/'"$CURRENT_USER"'/USERNAME/g' ${files};
done


# $ sudo systemctl enable roscore.service
# $ sudo systemctl enable roslaunch.service
# $ sudo chmod +x /usr/sbin/roslaunch