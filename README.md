# Running ROS launch files on a robot from the cloud with Salt

This little project was meant to test using Saltstack (https://saltstack.com/) for the orchestration of robotic tasks on robots from the cloud. 
Rather than developing our own process management framework /  protocol on top of MQTT or whatever communication primitive we will choose to implement a Cloud Robotics PaaS, the idea is to use what's already there for remote process execution and configuration management.

Robots can physically be anywhere and most likely will not be directly addressable (either because NATted or firewalled or both). Salt lets us get around this asymmetric communication issue by relying on an agent installed on the machines/robots to be controlled (salt-minions) to connect to the ZeroMQ on the salt-master which runs in the cloud.

In this project you find:
 - the 'salt_master' directory
 - this README.md with some simple commands and instructions

## Master Execution

The salt_master directory contains the Dockerfile for building the salt-master container, a build file, and a script to launch it.
Since the container is already available on Docker hub, all you need to do to run your own salt master is to launch 'run_container.sh'

    cd salt_master
    ./run_container.sh

If you look at what the script does, it is mounting 'salt_master/master_etc_salt' and 'salt_master/master_srv_salt' respectively in '/etc/salt' and 'srv/salt' in the container plus opening the needed ports to allow ZeroMQ connection.
The reason we do this is to preserve the keys of the master and the public key of the minions that it controls across runs of the container. The same thing can be said about the 'srv' folder.

You can access the master by name:

    docker exec -it salt_master bash

This will give you a console with all the master salt commands.

## Adding a minion

This works exactly as explained on the salt installation instructions: https://repo.saltstack.com/

Login to your minion and run:

    curl -L https://bootstrap.saltstack.com -o install_salt.sh
    sudo sh install_salt.sh -P

The next step is configuration (https://docs.saltstack.com/en/latest/ref/configuration/index.html)

On your master, run:

    salt-key -F master

Copy the master.pub fingerprint from the Local Keys section, and then set this value as the 'master_finger' in the minion configuration file. I.e., on your minion, edit the file /etc/salt/minion:
  - uncomment the line saying '#master: salt' and replace 'salt' with your master (host) IP address
  - edit the line saying 'master_finger' adding the value of the master.pub fingerprint

Start (or restart) the salt-minion:

    salt-minion -l info

Now we can finally accept the key of the minion on the master. On the master,
run:

    salt-key -L

This will show you the name of your minion and its key as being in the "Unaccepted Keys" state.
You can accept all keys with:

    salt-key -A

Done!

## Running ROS launch files on your minions

We tested our setup on our turtlebots at the office. The goal for now was just to launch a couple of ROS launch files as background commands remotely. In the future we will probably use the concept of "state" in Salt to schedule different concurrent robotic behaviors on our robots. But for now we just used this:

### Run minimal
This uses the salt module for running a task in background, sources a couple of needed files, and runs roslaunch with our own specific turtlebot files using a laser scanner. On the master, run:

    salt turtlebot.robonet cmd.run_bg "source /opt/ros/indigo/setup.bash && source ~/catkin_ws/devel/setup.bash && source ~/turtlebot/devel/setup.bash && export ROS_MASTER_URI=http://turtlebot:11311 && export ROS_HOSTNAME=turtlebot && roslaunch icclab_turtlebot minimal_with_rplidar.launch" runas=turtlebot shell="/bin/bash" cwd="/home/turtlebot"

Basically, we trigger on the target "turtlebot.robonet" (one of our turtlebots) that long concatenated command and we run it as a specific user.

### Run amcl
This uses the same setup to run the ROS amcl node (http://wiki.ros.org/amcl) allowing our robots to find themselves and navigate on our lab map:

    salt turtlebot.robonet cmd.run_bg "source /opt/ros/indigo/setup.bash && source ~/catkin_ws/devel/setup.bash && source ~/turtlebot/devel/setup.bash && export ROS_MASTER_URI=http://turtlebot:11311 && export ROS_HOSTNAME=turtlebot && roslaunch icclab_turtlebot amcl_icclab.launch map_file:=/home/turtlebot/catkin_ws/src/icclab_turtlebot/icclab_latest_map.yaml" runas=turtlebot shell="/bin/bash" cwd="/home/turtlebot"


