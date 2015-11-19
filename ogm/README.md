# Ocuppancy Grid Mapping


Probabilistic Robotics - Algorithms Implementations

Thrun, Sebastian; Burgard, Wolfram and Fox, Dieter. Probabilistic Robotics, MIT Press, 2005, ISBN 0262201623, 9780262201629. Cap. 9, pg 206, Table 9.1

## How to use:

Obs: You need the PioneerModel to use the simulation environment

Clone the project in your workspace (e.g. $catkin_ws/src/)

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/LCAD-UFES/probabilistic-robotics.git
    $ cd ~/catkin_ws
    $ catkin_make

### Running:
Bring up gazebo simulation and rviz:

    $ roslaunch ogm simulation.launch

After, in other terminal

  if you want control with joystick (with joy node):

     conect the joystick
     $ sudo chmod 777 /dev/input/js0
     $ roslaunch ogm ogmJoy.launch

  if you want control with keyboard

     $ roslaunch ogm ogm.launch
     Note: keep focus on terminal window to input the arrow commands

Drive around and enjoy the map discovery :)


