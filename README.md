<a id="readme-top"></a>

# FUN4
**This is fun4 assignment in FRA501(Robotics Development) class at **FIBO**. With content about manipulator control system for 3 DOF robot arm.**


<p align="center"><img src="images/robot_initial_pose.png" alt="Image Description" /></p>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
            <ul>
                <li><a href="#python-packages">Python packages</a></li>
                <li><a href="#ros2-packages">ROS2 packages</a></li>
            </ul>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <ul>
        <li><a href="#launch-the-project">Launch the project</a></li>
        <li><a href="#service-call-in-this-project">Service call in this project</a></li>
    </ul>
    <li><a href="#features">Features</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
## About The Project


<!-- GETTING STARTED -->
## Getting Started
### Prerequisites
To use this project. You need to have all of prerequisites for this project.
#### Python packages
⚠️  **Warning:**    Make sure you have python version >= 3.6 already.
*   setuptool
    ```
    pip3 install setuptools==59.6.0
    ```
*   numpy
    ```
    pip3 install numpy==1.24.1
    ```
*   scipy
    ```
    pip3 install scipy==1.8.0
    ```
*   matplotlib
    ```
    pip3 install matplotlib==3.5.1
    ```
*   robotics toolbox
    ```
    pip3 install roboticstoolbox-python
    ```
#### ROS2 packages
⚠️  **Warning:**     Make sure you have ROS2 humble already.
*   teleop_twist_keyboard
    ```
    sudo apt-get install ros-humble-teleop-twist-keyboard
    ```

### Installation
Follow the command below to dowload and install package.
1.  Go to home directory
    ```
    cd
    ```
2.  Clone the repository
    ```
    git clone https://github.com/nakerin7588/RoboticsDev.git --branch=fun4 & cd RoboticsDev
    ```
3.  Build & Source the packages
    ```
    colcon build & source install/setup.bash
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE -->
## Usage
⚠️  **Warning:**    Before use this project you need to `source ~/RoboticsDev/install/setup.bash` and `source /opt/ros/humble/setup.bash` everytime that you open new terminal. If you want to make sure that 2 path has been source everytime when open new terminal you can follow the command below and next time you open new terminal .bashrc will source everything you write on that file.
```
echo "source ~/RoboticsDev/install/setup.bash" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### Launch the project
```
ros2 launch fun4 robot_bringup.launch.py
```
After launch the project rviz2 window will show on your screen with red RRR robot arm like this picture below.
![alt text](images/rviz_screen_after_launch_project.png)
### Service call in this project
1. **Mode select**
    ```
    ros2 service call /mode_select fun4_interfaces/srv/SetModePosition "mode: <mode>"
    ```
    Change `<mode>` to mode that you want to use such as :
    *  `0` for `wait` mode
    *  `1` for `inverse kinematic` mode
    *  `2` for `teleop/reference velocity from end-effector frame` mode
    *  `3` for `teleop/reference velocity from world frame` mode
    *  `4` for `auto` mode
<p align="right">How to use mode will explain in the next section.</p>

2. **Inverse kinematic mode target**
    ```
    ros2 service call /ik_target fun4_interfaces/srv/SetModePosition "position: {x: <x_pos>, y: <y_pos>, z: <z_pos>}"
    ```
    Change `<x_pos>`, `<x_pos>`, `<x_pos>` to position of axis that you want.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Features
This section explains what this or those features do in this project.
1.  **Show workspace of the robot**

    To show workspace of the robot you must execute the `workspace.ipynb` in VScode. You can follow by this command below.
    ```
    code ~/RoboticDev/workspace/workspace.ipynb
    ```
    After that you can see the workspace in VScode like this picture below.

2.  **Show End-effector and Target pose**

    This feature can show the current end-effector pose and target pose(From inverse kinematic and auto mode). After you launch the project you can see the axis ball on end-effector of robot and robot base like the picture below that is an initial end-effector pose and target pose.
    <p align="center"><img src="images/robot_initial_pose_fade.png" alt="Image Description" /></p>

3.  **Wait mode**

    This mode work like idle mode to wait for the command when you launch the project robot will entry to this mode first.

    To change the mode when you are in other mode. You can follow this command below.
    ```
    ros2 service call /mode_select fun4_interfaces/srv/SetModePosition "mode: 0"
    ```
    After that robot will stop at current position until you change mode to move the robot.

4.  **Inverse kinematic mode**

    This mode has ability to move the robot end-effector to the target position you want by calculate inverse kinematic.

    ⚠️  **Warning:** Make sure you launch this project first.

    To use this mode you can follow by this command below.
    ```
    ros2 service call /mode_select fun4_interfaces/srv/SetModePosition "mode: 1"
    ```
    After that you can set the target of the robot by this commad below. In this example will define the target as x: 0.01, y: 0.2, z: 0.04
    ```
    ros2 service call /ik_target fun4_interfaces/srv/SetModePosition "position: {x: 0.01, y: 0.2, z: 0.04}"
    ```
    After run this command the robot on rviz will move to the target and end-effector/target ball will show together.

    <p align="center"><img src="images/ik_move.gif" alt="Image Description" /></p>

5.  **Teleoperation mode**

    This mode has ability to move the robot by using `teleop_twist_keyboard` that can set the velocity and send it to the robot. This mode has 2 sub-mode
    *   Velocity that reference from end-effector frame
    *   Velocity that reference from world frame
6.  **Auto mode**

    This mode is like Inverse kinematic mode that will move the robot end-effector to the target position but this mode will random the target from environment mode and then will send the target to inverse kinematic mode to calculate. **You can see the architecture diagram on the top of this README**
    
    ⚠️  **Warning:** Make sure you launch this project first.

    To use this mode you can follow by this command below.
    ```
    ros2 service call /mode_select fun4_interfaces/srv/SetModePosition "mode: 4"
    ```
    After that robot will automatically random the target and move the robot to that target until the robot reach target. Target will random new position.

    <p align="center"><img src="images/auto_move.gif" alt="Image Description" /></p>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

Nakarin Jettanatummajit - nakerin7588@gmail.com - nakarin.jett@mail.kmutt.ac.th

Project Link: [https://github.com/nakerin7588/RoboticsDev/tree/fun4](https://github.com/nakerin7588/RoboticsDev/tree/fun4)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

<p align="right">(<a href="#readme-top">back to top</a>)</p>