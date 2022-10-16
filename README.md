# Stanford Quadruped

## Pupper V3
### Set up
* Build this ros2 colcon workspace https://github.com/Nate711/pupperv3-testing 
* Source its installation folder to get the custom messages

### Running
#### Robot Option A: Start simulated robot
From inside the pupperv3-testing workspace:

```ros2 launch pupper_mujoco fixed_base_launch.py```

#### Robot Option B: Start real robot
From inside the pupperv3-testing workspace:
1. Turn on robot
2. `sudo bash can0_initialize.bash`
3. `ros2 run cpp_pubsub motor_controller_node_test`

#### Controller A: Trotting in place
From inside this repo
1. `python3 examples/pupper_v3_trot_in_place.py`

#### Controller B: Joystick control
From inside this repo
1. ```ros2 run joy joy_node &```
2. ```python3 examples/pupper_v3_ds4_control.py```