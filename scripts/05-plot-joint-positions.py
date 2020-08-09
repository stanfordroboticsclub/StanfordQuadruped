import pickle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

saved_states = pickle.load(open("saved-states.pkl", "rb"))
foot_locations = np.array([s.foot_locations for s in saved_states])
joint_angles = np.array([s.joint_angles for s in saved_states])

# print (foot_locations[0,[0,2],:])
#
# print (foot_locations.shape)
#
# foot_locations_x = foot_locations[50:,0,:]
# foot_locations_y = foot_locations[50:,2,:]
#
# print (foot_locations_x[:10])
# print (foot_locations_y[:10])
#
# norm = colors.Normalize(vmin=0, vmax=239)
#
# for i in range(4):
#     plt.scatter(foot_locations_x,foot_locations_y,label=f"foot {i}")
#
# plt.legend()
# plt.show()


x = np.arange(0,len(joint_angles))
for i in range(3):
    plt.plot(x, joint_angles[:,i,0], label=f"joint {i}")

plt.legend()
plt.tight_layout()
plt.show()



