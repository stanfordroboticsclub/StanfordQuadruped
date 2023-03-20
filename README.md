# Dingo Quadruped

## Overview
This repository hosts the code for the Dingo Quadruped. This repo is a forked, updated and heavily modified version of the Stanford Pupper and Stanford Woofer code, which can be found [here](https://github.com/stanfordroboticsclub/StanfordQuadruped)

## How it works
The main program is ```run_robot.py``` which is located in this directory. The robot code is run as a loop, with a joystick interface, a controller, and a hardware interface orchestrating the behavior. 

The joystick interface is responsible for reading joystick inputs from a UDP socket and converting them into a generic robot ```command``` type. A separate program, ```joystick.py```, publishes these UDP messages, and is responsible for reading inputs from the PS4 controller over bluetooth. The controller does the bulk of the work, switching between states (trot, walk, rest, etc) and generating servo position targets. A detailed model of the controller is shown below. The third component of the code, the hardware interface, converts the position targets from the controller into PWM duty cycles.

The gait scheduler is responsible for planning which feet should be on the ground (stance) and which should be moving forward to the next step (swing) at any given time. In a trot for example, the diagonal pairs of legs move in sync and take turns between stance and swing. As shown in the diagram, the gait scheduler can be thought of as a conductor for each leg, switching it between stance and swing as time progresses. 

The stance controller controls the feet on the ground, and is actually quite simple. It looks at the desired robot velocity, and then generates a body-relative target velocity for these stance feet that is in the opposite direction as the desired velocity. It also incorporates turning, in which case it rotates the feet relative to the body in the opposite direction as the desired body rotation. 

The swing controller picks up the feet that just finished their stance phase, and brings them to their next touchdown location. The touchdown locations are selected so that the foot moves the same distance forward in swing as it does backwards in stance. For example, if in stance phase the feet move backwards at -0.4m/s (to achieve a body velocity of +0.4m/s) and the stance phase is 0.5 seconds long, then we know the feet will have moved backwards -0.20m. The swing controller will then move the feet forwards 0.20m to put the foot back in its starting place. You can imagine that if the swing controller only put the leg forward 0.15m, then every step the foot would lag more and more behind the body by -0.05m. 

Both the stance and swing controllers generate target positions for the feet in cartesian coordinates relative the body center of mass. It's convenient to work in cartesian coordinates for the stance and swing planning, but we now need to convert them to motor angles. This is done by using an inverse kinematics model, which maps between cartesian body coordinates and motor angles. These motor angles, also called joint angles, are then populated into the ```state``` variable and returned by the model. 

## How to Install and Run Dingo Code
This repository includes several options for building and running Dingo. To get started, follow these instructions:
### (Only for implementing this code on the quadruped itself) Installing Ubuntu 20.04 Server onto a Raspberry Pi

### (For all users) Installing this repository natively on Ubuntu20.04:
- Install [ros-noetic](http://wiki.ros.org/noetic/Installation/Ubuntu). 
- Install git via `sudo apt-get install git`
- Create a new folder in your home folder: `mkdir ~/any_folder_name`
- Change directory to this new folder: `cd ~/any_folder_name`
- Clone this repository into the folder using the most convenient clone method for you: `git clone ...`

### (For all users) Running natively
For native installs, the dingo_ws folder contains everything you need and you can ignore the files inside the base folder, as these are for building a docker container.
- TODO

### [optional] Running from vscode via a docker container
The files inside the base directory enable a docker container to be built and the code to inspected and debugged in visual studio code. This is mostly for debugging purposes, and is best for an external device debugging or adding to the code, rather than being used on the quadruped itself. Note: These instructions currently assume a linux OS. For windows the steps may vary in implementation but the goal of each step should be the same.
#### Preparing vscode
- Install [docker](https://docs.docker.com/engine/install/ubuntu/)
- Install [vscode](https://code.visualstudio.com/docs/setup/linux)
- Open vscode and add the following extensions: [C/C++ Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack), [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker), [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers), [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
- close vscode once extensions are installed
#### Building and/or opening the container in vscode
- In terminal, open the base folder containing the dingo quadruped code: `cd ~/any_folder_name/DingoQuadruped`
- run `. code` to open the dingo quadruped base folder in vscode
- A prompt will appear saying either to build the container or run it, click "build" or "run"
- Wait for the container to be built and initialised
- (First time only) Once the container is built, Check that "ROS1.noetic" appears in the bottom left to indicate that the ros extension has correctly detected the ros version inside the container. If it does not appear, follow [these steps](https://youtu.be/JbBMF1aot5k?t=356)
#### Using the ros workspace in vscode
The ROS extension provides options to roslaunch and rosrun files inside vscode via the inbuilt terminal without needing to use the native linux terminal. The commands to do so are the same as in a native linux terminal. Currently, this repo has no support for nodes that use visualisation tools but any other files should launch or run as they would if you were using the native linux terminal. 

To start/stop a roscore daemon inside vscode, you can type `ctrl+shift+P` in vscode, and then type `ROS: Start` to start and `ROS: Stop` to stop the roscore daemon.

To build or rebuild the ros workspace, type `ctrl+shift+B`. If this does not work, you may need to edit the tasks.json file which tells vscode how to build the container. Ensure that the catkin build task defined in tasks.json includes the option `-DCMAKE_BUILD_TYPE=Debug`, as without this the vscode debugger will not work correctly.

An important note, as the entire ros workspace is volume mounted, files can be edited inside the container and reflected in your native linux filesystem and vice versa. This means the code can be changed and debugged in the vscode container but run in the native linux terminal, with all changes being reflected. 

#### Debugging with vscode
The ROS extension has two options to enable debugging. The first is to attach to a running node which you start via the terminal with `rosrun package_name node_name`. The second is to debug from a launch file, where you use the debugger menu in vscode to play a launch file and then set waypoints in any nodes which the launch file starts. To set these up, please watch [this video](https://youtu.be/N2vqBvPQdhE?list=PL2dJBq8ig-vihvDVw-D5zAYOArTMIX0FA)

If the debugger is not stopping at breakpoints, you may need to edit the tasks.json file which tells vscode how to build the container. Ensure that the catkin build task defined in tasks.json includes the option `-DCMAKE_BUILD_TYPE=Debug`.





