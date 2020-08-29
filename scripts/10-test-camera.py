import cv2
import numpy as np
from stanford_quad.sim.simulator2 import PupperSim2

sim = PupperSim2(debug=True)
sim.reset()
sim.add_stairs(step_width=2, step_depth=0.3, step_height=0.05, offset=(0.3, 0, 0), random_color=True)

for step in range(100000):
    # sim.action(motorPo)
    action = np.random.uniform(-0.3, 0.3, 12)
    sim.action(action)
    sim.step()

    # render an image
    img = sim.take_photo()

    # display the image (converting from RGB to BGR because OpenCV is weird)
    cv2.imshow("rendered img", img[:, :, ::-1])
    cv2.waitKey(10)
