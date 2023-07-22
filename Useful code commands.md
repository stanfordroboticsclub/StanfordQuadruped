# Useful code commands


# Adding changed files to Github Repository

Add all changed files, then commit them with a message and push

```python
git add --all
git commit -m "Insert your commit message here"
git push
```

# Managing Wifi Networks and Connections

Connections on the pi can be managed by running:

```python
sudo nano /etc/netplan/50-cloud-init.yaml
```

# ROS launch for LCD at startup

This site explains how the LCD launch file can be called at startup using the robot_upstart Package.

[https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/](https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/)

This can be enabled using:

```python
rosrun robot_upstart install dingo_peripheral_interfacing/launch/LCD_service.launch --job LCD_ros --symlink
```

In the line above, `dingo_peripheral_interfacing` is the package, `LCD_service.launch` is the launch file and `LCD_ros`is the new job name that automates the launch file being called at startup. 

The `--symlink` creates a symbolic link from the launch file to the film in systemctl so that any changes to the original file will update. 

You will now be instructed to run:

```python
sudo systemctl daemon-reload && sudo systemctl start LCD_ros
```

## Run LCD node manually

If it doesn’t properly start on startup, LCD can be turned on using:

```python
rosrun dingo_peripheral_interfacing dingo_lcd_interfacing.py
```

The LCD module is currently set to run at startup using ROS Upstart, and from the dingo.launch file.

# Create a pull request on Github

Include the Notion task link in your PR description. 

# Launch Dingo control in simulation

```python
source devel/setup.bash
roslaunch dingo dingo.launch is_physical:=0 is_sim:=1 launch_sim_environment:=1
```

# View joint or task space objectives

Make sure dingo.launch has been started first then:

```python
source devel/setup.bash
rostopic echo /task_space_goals
rostopic echo /joint_space_goals
```

# Calibrate Robot Servos

The **HardwareInterface.py** code uses a matrix of offset angles to ensure the robot’s motors are calibrated. Calibration can be tweaked in two ways

1. **CalibrateServos.py** contains extensive calibration instructions. It requires running the script multiple times and tweaking offset values individually. Note that this file is not used by any other files, so once the offset values have been set here, they can be copied across the the **HardwareInterface.py** code for use in the main files. 
    
    This calibration can be run using:
    
    ```python
    rosrun dingo_servo_interfacing CalibrateServos.py
    ```
    
    1. At the bottom of the [**HardwareInterface.py](http://HardwareInterface.py)** code, there is a section that allows testing by sending all legs to the same set of 3 joint angles (defining hip, upper and lower leg servo angles for the leg. This is not the suggested method but can be good if only a single motor needs to be tweaked. This file can be run using:
    
    ```python
    rosrun dingo_servo_interfacing HardwareInterface.py
    ```