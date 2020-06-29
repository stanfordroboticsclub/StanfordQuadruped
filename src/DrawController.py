import numpy as np

class DrawController:
    PEN_DOWN_HEIGHT = 0.003 + 0.01
    PEN_UP_HEIGHT = 0.02 + 0.01

    MAX_LIN_DIST = 0.00075
    MAX_DRAW_SIZE = 0.15
    def __init__(self, config, fname='/home/pi/StanfordQuadruped/smiley.csv'):
        self.config = config
        self.draw_leg_index = 0
        self.vertices = np.loadtxt(fname, dtype=np.float, delimiter=',')
        self._normalize_drawing()
        self.drawing_index = 0
        self.previous_position = None

    def _normalize_drawing(self):
        min_xy = np.min(self.vertices[:, :2], axis=0)
        max_xy = np.max(self.vertices[:, :2], axis=0)
        scale = min(1, self.MAX_DRAW_SIZE / np.linalg.norm(max_xy - min_xy))
        mean_xy = (min_xy + max_xy) / 2
        self.vertices[:, :2] -= mean_xy
        self.vertices[:, :2] *= scale
        self.vertices = np.insert(self.vertices, 0, [0, 0, 1], axis=0)
        self.vertices[:, :2] += self.config.default_stance[:2, self.draw_leg_index] + np.array([0, 0.03])
        self.vertices[:, 2] = (self.PEN_UP_HEIGHT - self.PEN_DOWN_HEIGHT) * self.vertices[:, 2] + self.PEN_DOWN_HEIGHT

    def reset_drawing(self, state):
        self.drawing_index = 0
        self.previous_position = state.foot_locations[:, self.draw_leg_index]
        self.previous_position[2] = 0

    def get_next_position(self):
        previous_position = self.previous_position
        if (self.drawing_index >= self.vertices.shape[0]):
            return previous_position
        desired_position = self.vertices[self.drawing_index]
        delta = desired_position - self.previous_position
        dist = np.linalg.norm(delta)
        if dist > self.MAX_LIN_DIST:
            new_pos = previous_position + delta * (self.MAX_LIN_DIST / dist)
        else:
            self.drawing_index += 1
            new_pos = desired_position
        self.previous_position = new_pos
        return new_pos