# Folders in ROS packages
- src: For files containing classes and/or functions only, non-executables
- scripts: For executables. Need to add to install part of cmakelists.txt to be executable with rosrun. See run_robot.py in dingo_control as an example.
- launch: For launch files