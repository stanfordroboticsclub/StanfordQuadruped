import numpy as np
from quadprog import solve_qp
from matplotlib import pyplot as plt

class Polytope:
    def __init__(self, margin) -> None:
        self.margin = margin

    def calculate_halfspaces(self, vertices):
        self.vertices = vertices
        A_rows = []
        b_rows = []
        for i in range(len(vertices)):
            v0 = vertices[i]
            v1 = vertices[(i + 1) % len(vertices)]
            dx = v1[0] - v0[0]
            dy = v1[1] - v0[1]
            normal_vector = np.array([-dy, dx]) / np.sqrt(dx ** 2 + dy ** 2)
            normal_distance = normal_vector.dot(v0) + self.margin
            A_rows.append(normal_vector)
            b_rows.append(normal_distance)
        self.A = np.vstack(A_rows)
        self.b = np.array(b_rows)
    
    def closest_feasible_point(self, point):
        G = np.eye(len(point))
        a = point
        try:
            xf, f, xu, iters, lagr, iact = solve_qp(G, a, self.A.T, self.b)
            return xf
        except ValueError: # If no feasible point exists
            avg = sum(self.vertices) / len(self.vertices)
            return avg
