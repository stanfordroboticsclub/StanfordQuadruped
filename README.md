# Stanford Quadruped

## Overview
This repository hosts the code for Stanford Pupper and Stanford Woofer, Raspberry Pi-based quadruped robots that can trot, walk, and jump. 

![Pupper CC Max Morse](https://live.staticflickr.com/65535/49614690753_78edca83bc_4k.jpg)

Video of pupper in action: https://youtu.be/NIjodHA78UE

Project page: https://stanfordstudentrobotics.org/pupper

Documentation & build guide: https://pupper.readthedocs.io/en/latest/

## How it works
![Overview diagram](imgs/diagram1.jpg)
The main program is ```run_robot.py``` which is located in this directory. The robot code is run as a loop, with a joystick interface, a controller, and a hardware interface orchestrating the behavior. 

The joystick interface is responsible for reading joystick inputs from a UDP socket and converting them into a generic robot ```command``` type. A separate program, ```joystick.py```, publishes these UDP messages, and is responsible for reading inputs from the PS4 controller over bluetooth. The controller does the bulk of the work, switching between states (trot, walk, rest, etc) and generating servo position targets. A detailed model of the controller is shown below. The third component of the code, the hardware interface, converts the position targets from the controller into PWM duty cycles, which it then passes to a Python binding to ```pigpiod```, which then generates PWM signals in software and sends these signals to the motors attached to the Raspberry Pi.
![Controller diagram](imgs/diagram2.jpg)
This diagram shows a breakdown of the robot controller. Inside, you can see four primary components: a gait scheduler (also called gait controller), a stance controller, a swing controller, and an inverse kinematics model. 

The gait scheduler is responsible for planning which feet should be on the ground (stance) and which should be moving forward to the next step (swing) at any given time. In a trot for example, the diagonal pairs of legs move in sync and take turns between stance and swing. As shown in the diagram, the gait scheduler can be thought of as a conductor for each leg, switching it between stance and swing as time progresses. 

The stance controller controls the feet on the ground, and is actually quite simple. It looks at the desired robot velocity, and then generates a body-relative target velocity for these stance feet that is in the opposite direction as the desired velocity. It also incorporates turning, in which case it rotates the feet relative to the body in the opposite direction as the desired body rotation. 

The swing controller picks up the feet that just finished their stance phase, and brings them to their next touchdown location. The touchdown locations are selected so that the foot moves the same distance forward in swing as it does backwards in stance. For example, if in stance phase the feet move backwards at -0.4m/s (to achieve a body velocity of +0.4m/s) and the stance phase is 0.5 seconds long, then we know the feet will have moved backwards -0.20m. The swing controller will then move the feet forwards 0.20m to put the foot back in its starting place. You can imagine that if the swing controller only put the leg forward 0.15m, then every step the foot would lag more and more behind the body by -0.05m. 

Both the stance and swing controllers generate target positions for the feet in cartesian coordinates relative the body center of mass. It's convenient to work in cartesian coordinates for the stance and swing planning, but we now need to convert them to motor angles. This is done by using an inverse kinematics model, which maps between cartesian body coordinates and motor angles. These motor angles, also called joint angles, are then populated into the ```state``` variable and returned by the model. 


## How to Build Pupper
Main documentation: https://pupper.readthedocs.io/en/latest/

You can find the bill of materials, pre-made kit purchasing options, assembly instructions, software installation, etc at this website.


## Help
- Feel free to raise an issue (https://github.com/stanfordroboticsclub/StanfordQuadruped/issues/new/choose) or email me at nathankau [at] stanford [dot] edu
- We also have a Google group set up here: https://groups.google.com/forum/#!forum/stanford-quadrupeds


## Using DJI Pupper (for Stuart)
### Set up
* Clone this repo and checkout this branch ("dji")
* Clone https://github.com/stanfordroboticsclub/PupperKeyboardController and follow its README instructions

### First Time Setup
* Plug the Teensy into your computer and figure out which tty device it is.
  * It shows up on my computer as "/dev/tty.usbmodem78075901" but it's probably different on yours
  * Run `ls /dev | grep tty.usbmodem` to easily find out
  * Update `SERIAL_PORT` in `djipupper/IndividualConfig.py` with the specific port name

### Using DJI Pupper
* Plug in the battery
* Plug in the Teensy to your computer
* Run the keyboard joystick program
  * Run `python3 keyboard_joystick.py`
  * It'll open up a small window and once it's loaded it'll say something like "click to enable"
  * Note that that window has to be the "active" window on your computer for it to capture keyboard events. So make sure you click it before trying to give commands.
  * Joystick to keyboard mapping:
    * L1: q (activate/deactive)
    * R1: e (trot/rest)
    * Left joystick: wasd (forward/back & left/right)
    * Right joystick: arrow keys (tilting up/down and yawing left/right)
    * D-pad: ijkl (i/k for moving body up/down and j/l for rolling)
* IMPORTANT: Orient the robot so that all the actuators are in their "zero" position. This means that the legs are extended and pointing straight down and that the abduction motors are perfectly horizontal. Plus/minus 5 degrees is usually what I go for although the closer you can make it the better.
* Run the python controller with the motor-zeroing option
  * `python3 run_djipupper.py --zero`
  * The zeroing option tells the Teensy to store the current leg configuration as the "zero" state
  * The Teensy will remember this position for as long as it's turned on, so even if we quit the `run_djipupper.py` program, we can re-use the calibration if we don't send the zeroing command.
* When you're done using the robot, deactivate it by pressing `q` with the keyboard window active. Then press control-c.
* For all subsequent runs, where you don't want to re-zero, run `python3 run_djipupper.py`. Since we're reusing the calibration from the earlier run, you don't need to move the legs to their zero position before running this command.
* IMPORTANT: When in doubt, press `q` to deactivate.

### Tuning
* You can mess with the cartesian PD control gains by changing values in `djipupper/HardwareConfig.py`
  * `MAX_CURRENT`: it's interesting to put it a little lower, like 4A, to test squishiness.
  * `CART_POSITION_KPS`: Stiffness in the x, y, z directions. I've found 400 - 4000 to be interesting values. 400 is quite loose while 4000-6000 is very stiff. Much higher (>6000) and you risk uncontrolled oscillations even with higher damping.
  * `CART_POSITION_KDS`: Damping in the x, y, z directions. 100 to 500 seems to be a good range. Use higher values when you're using higher stiffnesses to avoid oscillations, which totally does happen when you use, for example, kp=4000 and kd=1500. 
  * `POSITION_KP` and `POSITION_KD` unused for cartesian pd control.
* You can also mess with the usual values like x and y velocity, z_clearance (stepping height), etc in `djipupper/Config.py`