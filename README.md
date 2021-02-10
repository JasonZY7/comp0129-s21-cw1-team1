Author: Team1 Zinan Liu, Chi Zhang, Zhiyuan Song
Description: The source code aim to solve the question in COMP0129 CW1:
Q1. Transformation between given frames. Publish framerate with 100Hz. Calculate 2_T_3.
Q2. Pressing different keys to manipulate robot pose and print 0_T_e for end effector pose.
Q3. Pergorm pick and place task
-------------------------------------------------------------------------------

===============================================================================
Question 1.4
===============================================================================
Compilation
---------------------------------------
> cd ~/cd ws_comp0129
> catkin build

-------------------------------------------------------------------------------

Execution
---------------------------------------
Terminal 1:
> roscore

Terminal 2:
> cd ~/ws_comp0129
> source devel/setup.bash
> roslaunch cw1 cw1.launch

===============================================================================
Question 1.5
===============================================================================
Compilation
---------------------------------------
> cd ~/ws_comp0129
> catkin build

-------------------------------------------------------------------------------

Execution
---------------------------------------
Terminal 1:
> roscore

Terminal 2:
> cd ~/ws_comp0129
> source devel/setup.bash
> roslaunch cw1 cw1.launch

===============================================================================
Question 2.1
===============================================================================
Compilation
---------------------------------------
> cd ~/ws_comp0129
> catkin build

-------------------------------------------------------------------------------

Execution
---------------------------------------
Terminal 1:
> roscore

Terminal 2:
> cd ws_comp0129
> source devel/setup.bash
> roslaunch cw1 cw1.launch
> h

===============================================================================
Question 2.2
===============================================================================
Compilation
---------------------------------------
> cd ~/ws_comp0129
> catkin build

-------------------------------------------------------------------------------

Execution
---------------------------------------
Terminal 1:
> roscore

Terminal 2:
> cd ~/ws_comp0129
> source devel/setup.bash
> roslaunch cw1 cw1.launch
> h
> p
> k
> p

===============================================================================
Question 2.3
===============================================================================
Compilation
---------------------------------------
> cd ~/ws_comp0129
> catkin build

-------------------------------------------------------------------------------

Execution
---------------------------------------
Terminal 1:
> roscore

Terminal 2:
> cd ~/ws_comp0129
> source devel/setup.bash
> roslaunch cw1 cw1.launch

Terminal 3:
> rostopic echo /ee_posse

===============================================================================
Question 3.1
===============================================================================

Compilation
---------------------------------------
Terminal 1:
> cd ~/ws_comp0129
> catkin build


Execution
---------------------------------------
Terminal 1:
> roscore

Terminal 2:
> cd ~/ws_comp0129
> source devel/setup.bash
> roslaunch cw1 cw1.launch

Terminal 3:
> rostopic echo /object_pose

===============================================================================
Question 3.2
===============================================================================

Compilation
---------------------------------------
Terminal 1:
> cd ~/ws_comp0129
> catkin build


Execution
---------------------------------------
Terminal 1:
> roscore

Terminal 2:
> cd ~/ws_comp0129
> source devel/setup.bash
> roslaunch cw1 cw1.launch

Terminal 3:
> rostopic echo /ee_obj_close

===============================================================================
Question 3.3.1
===============================================================================

Compilation
---------------------------------------
Terminal 1:
> cd ~/ws_comp0129
> catkin build


Execution
---------------------------------------
Terminal 1:
> roscore

Terminal 2:
> cd ~/ws_comp0129
> source devel/setup.bash
> roslaunch cw1 cw1.launch
> r

===============================================================================
Question 3.3.2
===============================================================================

Compilation
---------------------------------------
Terminal 1:
> cd ~/ws_comp0129
> catkin build


Execution
---------------------------------------
Terminal 1:
> roscore

Terminal 2:
> cd ~/ws_comp0129
> source devel/setup.bash
> roslaunch cw1 cw1.launch
> r
> s (or t)

===============================================================================
Question 3.3.3
===============================================================================

Compilation
---------------------------------------
Terminal 1:
> cd ~/ws_comp0129
> catkin build


Execution
---------------------------------------
Terminal 1:
> roscore

Terminal 2:
> cd ~/ws_comp0129
> source devel/setup.bash
> roslaunch cw1 cw1.launch
> r
> s (or t)
> g

Terminal 3:
> rostopic echo /ee_obj_close

===============================================================================
Question 3.3.4
===============================================================================

Compilation
---------------------------------------
Terminal 1:
> cd ~/ws_comp0129
> catkin build


Execution
---------------------------------------
Terminal 1:
> roscore

Terminal 2:
> cd ~/ws_comp0129
> source devel/setup.bash
> roslaunch cw1 cw1.launch
> r
> s (or t)
> g
> m

Terminal 3:
> rostopic echo /bumper2_touched
