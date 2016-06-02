# Running ROS launch files on a robot from the cloud with Salt

This little project was meant to test using Saltstack for the orchestration of robotic tasks on robots from the cloud. 
Rather than developing our own process management framework /  protocol on top of MQTT or whatever communication primitive we will choose to implement a Cloud Robotics PaaS, the idea is to use what's already there and for remote process execution and configuration management.

Robots can physically be anywhere and most likely will not be directly addressable (either because NATted or firewalled). Salt lets us get around this asymmetric communication issue by relying on an agent installed on the machines/robots to be controlled (salt-minions) to connect to the ZeroMQ on the salt-master which runs in the cloud.

In this project you find:
 - the 'salt-master' directory
 - this README.md with some simple commands and instructions

## Master Installation


# run minimal
salt turtlebot.robonet cmd.run_bg "source /opt/ros/indigo/setup.bash && source ~/catkin_ws/devel/setup.bash && source ~/turtlebot/devel/setup.bash && export ROS_MASTER_URI=http://turtlebot:11311 && export ROS_HOSTNAME=turtlebot && roslaunch icclab_turtlebot minimal_with_rplidar.launch" runas=turtlebot shell="/bin/bash" cwd="/home/turtlebot"

# run amcl
salt turtlebot.robonet cmd.run_bg "source /opt/ros/indigo/setup.bash && source ~/catkin_ws/devel/setup.bash && source ~/turtlebot/devel/setup.bash && export ROS_MASTER_URI=http://turtlebot:11311 && export ROS_HOSTNAME=turtlebot && roslaunch icclab_turtlebot amcl_icclab.launch map_file:=/home/turtlebot/catkin_ws/src/icclab_turtlebot/icclab_latest_map.yaml" runas=turtlebot shell="/bin/bash" cwd="/home/turtlebot"

# kill by PID
salt turtlebot.robonet ps.kill_pid 27961 signal=15
# at the moment it's only killing the process started by salt and not the actual roslaunch process and its siblings
# there must be a better way

# hacky way using PGID and kill command
# find out the process group if PGID by looking up the salt process id (returned to master)
on minion: ps x -o  "%p %r %y %x %c " | grep XXXPID_HEREXXX
e.g.,
  turtlebot@turtlebot:~$ ps x -o  "%p %r %y %x %c " | grep 7043
  7043  6965 ?        00:00:00 bash

#kill from master: 
salt turtlebot.robonet cmd.run "kill -TERM -6965"

# all in once (untested)
salt turtlebot.robonet cmd.run "kill -- -$(ps -o pgid= $PID | grep -o '[0-9]*')"

