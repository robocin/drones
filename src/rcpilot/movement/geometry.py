import numpy as np

def distance_between_points(point_1, point_2):
    point_1 = np.array(point_1)
    point_2 = np.array(point_2)

    return np.linalg.norm(point_1 - point_2)