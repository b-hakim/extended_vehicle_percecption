import random
import numpy as np


def euclidean_distance(p1, p2):
    if type(p1) is not np.ndarray:
        p1 = np.array(p1)

    if type(p2) is not np.ndarray:
        p2 = np.array(p2)

    return np.linalg.norm(p1-p2)


def move_point(point, angle, distance):
    return [point[0] + np.sin(np.deg2rad(angle)) * distance,
            point[1] + np.cos(np.deg2rad(angle)) * distance]


def get_new_abs_pos(sender_pos, sender_noisy_pos, obj_pos):
    sender_pos = np.array(sender_pos)
    obj_pos = np.array(obj_pos)
    sender_noisy_pos = np.array(sender_noisy_pos)

    rel_obj_pos = obj_pos - sender_pos
    dir = 1 if random.random() > 0.5 else -1
    rel_obj_pos[0] = rel_obj_pos[0] + dir * random.randint(1, 5) / 10
    dir = 1 if random.random() > 0.5 else -1
    rel_obj_pos[1] = rel_obj_pos[1] + dir * random.randint(1, 5) / 10

    abs_obj_noisy_pos = rel_obj_pos + sender_noisy_pos

    return abs_obj_noisy_pos.tolist()

