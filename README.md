# RoboCup2017 Rescue Simulation Virtual Robot League Models
This repository includes a robot model and field models used in RoboCup World Champion Ship 2017 Rescue Simulation Virtual Robot League(RC2017RVRL).  
You can find other records of the RC2017RVRL game in [wiki page of this repository](https://github.com/m-shimizu/RoboCup2017RVRL_Demo/wiki).
And you can find important information like the latest rule in [the rescue virtual robot league wiki page](http://wiki.robocup.org/Rescue_Simulation_Virtual_Robot_Competition).

In this year, we used "pioneer3at" robot, 5 fields for 3 preliminary games and a final game including 2 runs.  

And 4 kinds of victims were prepared; hot victim, moving victim, voice victim, dead victim.  
The hot victim can be seen as white by using a thermal camera.  
The moving victim is waving his right arm.
The voice victim is saying "Help me".
Dead victim is not hot, does not move his arm, and does not say anything.
In all game, only hot victims and dead victims were used. Because Moving victims took CPU power too much and almost of all teams wanted to use their own pioneer3at models that did not have a microphone.  
At the first team leader meeting(night of 25th), we defined sensor parameter values to keep same performance(Real Time Factor of gazebo). Each team did set those values of sensor parameters into their robot code.  

Sensor parameters were:  
    HOKUYO:  
        The number of beams: 1040  
        Frequency: 30  
    Camera:  
        Resolusion: 320 x 240  
        Frequency: 30  
    Thermal Camera:  
        Resolusion: 160 x 120  
        Frequency: 10  

## PC SPECIFICATION REQUIRMENT OF THIS REPOSITORY  
Several field models are heavy. You should use a desktop machine having a graphic card.  
Following is a common specification of PCs used in RC2017RVRL.  

    CPU: intel Core i-7 4790K 4GHz 8 cores  
    MEM: 16G Bytes  
    GPU: nVidia GTX 1070  

## SOFTWARE REQUIREMENT OF THIS REPOSITORY
### Install Ubuntu 16.04 LTS
Please done installation of Ubuntu 16.04 LTS (64bit).

### Install ROS Kinetic and Gazebo7 from PPA
#### *[Ubuntu install of ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
#### *[Install Gazebo using Ubuntu packages](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install)  
Do followings:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  
    sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116  
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'  
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -  
    sudo apt-get update  
    sudo apt-get install -y cmake g++ protobuf-compiler pavucontrol libgazebo7 libgazebo7-dev ros-kinetic-desktop ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-image-view2 ros-kinetic-rqt ros-kinetic-rqt-common-plugins ros-kinetic-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-message-to-tf ros-kinetic-tf2-geometry-msgs ros-kinetic-audio-common ros-kinetic-costmap-2d ros-kinetic-image-transport ros-kinetic-image-transport-plugins ros-kinetic-hector-mapping ros-kinetic-hector-geotiff ros-kinetic-hector-pose-estimation ros-kinetic-hector-gazebo-plugins ros-kinetic-hector-gazebo-worlds ros-kinetic-hector-sensors-description   
    sudo rosdep init  
    rosdep update  
    sudo apt-get install -y python−rosinstall  
    gazebo (and wait for finish of downloading fundamental models)  

__IMPORTANT NOTICE__  
At last, WE USED GAZEBO VERSION 7.7.0 in the final.  
In Gazebo version 7.8.1, pioneer3at written in sdf slipped.  
In Gazebo version 7.0.0, each pioneer3at written in sdf and urdf could move normally, but a field model for 2nd run of final game could not be loaded.  
Gazebo version 7.7.0 could let pioneer3at move normally and load the last field model.  

If you can not install gazebo version 7.7.0 in binary package, you can install it from source.  
See [here](https://github.com/m-shimizu/p3at_for_ros_with_modelsdf/wiki/Installing_Gazebo7).  

## HOW TO GET THIS REPOSITORY
Type following commands in a terminal.  

    $ cd  
    $ git clone https://github.com/m-shimizu/RC2017RVRL  

## HOW TO PREPARE TO USE THIS REPOSITORY  
Build packages including this repository.  

    $ cd ~/RC2017RVRL  
    $ catkin_make  
    
## GAME FIELD SPECIFICATIONS  
In the preliminary 1, by solving each team's connectivity trouble between each the game server and the team own robot controller, an old "victim175" model was used. All 5 victims were alive instead of "4 alive victims and 1 dead victim".  
In this repository, You can see 4 alive victims and 1 dead victim in the preliminary 1 field.   

|Game|Size|# of robots|# of alive victims|# of dead victims|  
|:---:|:---:|:---:|:---:|:---:|  
|Preliminary 1|88m x 92m|4|4|1|  
|Preliminary 2|90m x 70m|4|4|4|  
|Preliminary 3|220m x 200m|4|4|4|  
|Final 1st run|154m x 162m|4|4|4|  
|Final 2nd run|104m x 204m|4|4|6|  

## HOW TO SPAWN ROBOTS AND GAME FIELDS
At first, run following commands in each terminal:  

    $ cd  
    $ cd RC2017RVRL  
    $ source setup.bash  
    
Then run a command which you want to try.  
You should use a set of terminals by a team and a game slot.  
To increase stability, by every game, we checked connectivity between each game server and each team's own robot control software, and improved server side launch files.  

__FOR THE NEXT YEAR__   
We will have to start getting a commonsense about server side launch files and robot models from the next January. Checking game environment by each team at least 1 month before the next competition will be a required condition from the next competition.  

__[Preliminary 1] __  

|Team Name/Terminal #|Command|  
| :--- | :--- |  
|__* ChukyoRescue A__||
|Terminal 1|roslaunch rc2017rvrl  PreL1_sdf_org.launch|  
|||
|__* ChukyoRescue B__||
|Terminal 1|roslaunch rc2017rvrl  PreL1_sdf_org.launch|  
|||
|__* Echoic__||
|Terminal 1|roslaunch rc2017rvrl PreL1_urdf_echoic.launch|  
|Terminal 2|roslaunch pioneer2017 spawn_robots_PreL1.launch|  
|||
|__* MRL__||
|Terminal 1|roslaunch rc2017rvrl  PreL1_sdf_org.launch|  
|||
|__* SOSVR__||
|Terminal 1|roslaunch p3at_urdf PreL1.launch|  
|||
|__* Yildiz__||  
|Terminal 1|roslaunch rc2017rvrl PreL1_sdf_yildiz.launch|  

__[Preliminary 2] __  

|Team Name/Terminal #|Command|  
| :--- | :--- |  
|__* ChukyoRescue A__||
|Terminal 1|roslaunch rc2017rvrl  PreL2_sdf_org.launch|  
|||
|__* ChukyoRescue B__||
|Terminal 1|roslaunch rc2017rvrl  PreL2_sdf_org.launch|  
|||
|__* Echoic__||
|Terminal 1|roslaunch rc2017rvrl PreL2_urdf_echoic.launch|  
|Terminal 2|roslaunch pioneer2017 spawn_robots_PreL2.launch|  
|||
|__* MRL__||
|Terminal 1|roslaunch rc2017rvrl  PreL2_sdf_MRL.launch|  
|||
|__* SOSVR__||
|Terminal 1|roslaunch rc2017rvrl PreL2_urdf_sos.launch|  
|Terminal 2|roslaunch rc2017rvrl PreL2_urdf_sos_spawn.launch|  
|Terminal 3|roslaunch rc2017rvrl PreL2_urdf_sos_spawn_1.launch|  
|Terminal 4|roslaunch rc2017rvrl PreL2_urdf_sos_spawn_2.launch|  
|Terminal 5|roslaunch rc2017rvrl PreL2_urdf_sos_spawn_3.launch|  
|||
|__* Yildiz__||  
|Terminal 1|roslaunch rc2017rvrl PreL2_sdf_yildiz.launch|  

__[Preliminary 3] __  

|Team Name/Terminal #|Command|  
| :--- | :--- |  
|__* ChukyoRescue A__||
|Terminal 1|roslaunch rc2017rvrl  PreL3_sdf_org.launch|  
|||
|__* ChukyoRescue B__||
|Terminal 1|roslaunch rc2017rvrl  PreL3_sdf_org.launch|  
|||
|__* Echoic__||
|Terminal 1|roslaunch rc2017rvrl PreL3_urdf_no_robot.launch|  
|Terminal 2|roslaunch pioneer2017 spawn_robots_PreL3.launch|  
|||
|__* MRL__||
|Terminal 1|roslaunch rc2017rvrl  PreL3_sdf_MRL.launch|  
|||
|__* SOSVR__||
|Terminal 1|roslaunch rc2017rvrl PreL3_no_robot.launch|  
|Terminal 2|roslaunch rc2017rvrl PreL3_urdf_sos_spawn.launch|  
|Terminal 3|roslaunch rc2017rvrl PreL3_urdf_sos_spawn_1.launch|  
|Terminal 4|roslaunch rc2017rvrl PreL3_urdf_sos_spawn_2.launch|  
|Terminal 5|roslaunch rc2017rvrl PreL3_urdf_sos_spawn_3.launch|  
|||
|__* Yildiz__||  
|Terminal 1|roslaunch rc2017rvrl PreL3_sdf_yildiz.launch|  

__[Final 1st run] __  

|Team Name/Terminal #|Command|  
| :--- | :--- |  
|__* ChukyoRescue B__||
|Terminal 1|roslaunch rc2017rvrl  F1_sdf_org.launch|  
|||
|__* Echoic__||
|Terminal 1|roslaunch rc2017rvrl F1_urdf_no_robot.launch|  
|Terminal 2|roslaunch pioneer2017 spawn_robots_F1_1.launch|  
|Terminal 3|roslaunch pioneer2017 spawn_robots_F1_2.launch|  
|Terminal 4|roslaunch pioneer2017 spawn_robots_F1_3.launch|  
|Terminal 5|roslaunch pioneer2017 spawn_robots_F1_4.launch|  
|||
|__* SOSVR__||
|Terminal 1|roslaunch rc2017rvrl F1_no_robot.launch|  
|Terminal 2|roslaunch rc2017rvrl F1_urdf_sos_spawn.launch|  
|Terminal 3|roslaunch rc2017rvrl F1_urdf_sos_spawn_1.launch|  
|Terminal 4|roslaunch rc2017rvrl F1_urdf_sos_spawn_2.launch|  
|Terminal 5|roslaunch rc2017rvrl F1_urdf_sos_spawn_3.launch|  
|||
|__* Yildiz__||  
|Terminal 1|roslaunch rc2017rvrl F1_sdf_yildiz.launch|  

__[Final 2nd run] __  

|Team Name/Terminal #|Command|  
| :--- | :--- |  
|__* ChukyoRescue B__||
|Terminal 1|roslaunch rc2017rvrl  F2_sdf_org.launch|  
|||
|__* Echoic__||
|Terminal 1|roslaunch rc2017rvrl F2_urdf_no_robot.launch|  
|Terminal 2|roslaunch pioneer2017 spawn_robots_F2_1.launch|  
|Terminal 3|roslaunch pioneer2017 spawn_robots_F2_2.launch|  
|Terminal 4|roslaunch pioneer2017 spawn_robots_F2_3.launch|  
|Terminal 5|roslaunch pioneer2017 spawn_robots_F2_4.launch|  
|||
|__* SOSVR__||
|Terminal 1|roslaunch rc2017rvrl F2_no_robot.launch|  
|Terminal 2|roslaunch rc2017rvrl F2_urdf_sos_spawn.launch|  
|Terminal 3|roslaunch rc2017rvrl F2_urdf_sos_spawn_1.launch|  
|Terminal 4|roslaunch rc2017rvrl F2_urdf_sos_spawn_2.launch|  
|Terminal 5|roslaunch rc2017rvrl F2_urdf_sos_spawn_3.launch|  
|||
|__* Yildiz__||  
|Terminal 1|roslaunch rc2017rvrl F2_sdf_yildiz.launch|  

## HOW TO CONTROL ROBOTS  
At first, you need to check topic names of each robot.  
And then, you can cnotrol a robot.  
Followings are an exampole in case of pioneer3at_ros was spawned.  

    $ rostopic list  
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/pioneer3at_ros/cmd_vel  

## NETWORK CONFIGURATION  
A network configuration set shown as below was used.
All PC has next 6 entries in /etc/hosts.

    host1 10.3.2.1
    host2 10.3.2.2
    host3 10.3.2.3
    host4 10.3.2.4
    player1 10.3.2.5
    player6 10.3.2.6

Below table shows each PC's hostname, role and IP address.

|Hostname|Role|/etc/hostname|IP address|
|:---:|:---:|:---:|:---:|
|host1|Game server|host1|10.3.2.1|
|host2|Game server|host2|10.3.2.2|
|host3|Test server<br>/Game server|host3|10.3.2.3|
|host4|Test server<br>/Game server|host4|10.3.2.4|
|player1|Team robot controller1|player1|10.3.2.5|
|player2|Team robot controller2|player2|10.3.2.6|

IP addresses, a network mask and name server addresses were given by a network administrators of the venue. 

#### Changed 6/Aug./2017
