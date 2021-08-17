import numpy as np


def euclidean_distance(p1, p2):
    if type(p1) is not np.ndarray:
        p1 = np.array(p1)

    if type(p2) is not np.ndarray:
        p2 = np.array(p2)

    return np.linalg.norm(p1-p2)


def angle_between_two_vectors(v1, v2):
    angle = np.arctan2(np.cross(v1, v2), np.dot(v1, v2))
    angle = np.rad2deg(angle)
    return angle


def get_vector(pt1, pt2):
    return [pt2[0]-pt1[0], pt2[1]-pt1[1]]


def cross_product(v1, v2):
    return v1[0] * v2[1] - v1[1] * v2[0]


def check_turn(line, pt):
    '''
    Explanation
    https://algorithmtutor.com/Computational-Geometry/Determining-if-two-consecutive-segments-turn-left-or-right/
    '''
    (x1, y1, x2, y2) = line
    if y2 > y1:
        x1, y1, x2, y2 = x2, y2, x1, y1

    v1 = (x2-x1, y2-y1)
    v2 = (pt[0] - x1, pt[1] - y1)
    cp = cross_product(v1, v2)

    if cp < 0: # Turn Left
        return -1
    elif cp > 0:
        return 1
    else:
        return 0


def intersection_exists(l1, l2):
    a = check_turn(l2, l1[0:2])
    b = check_turn(l2, l1[2:4])
    c = check_turn(l1, l2[0:2])
    d = check_turn(l1, l2[2:4])

    return a != b and c != d


def does_line_intersect_polygon(line, poly):
    for i in range(1, len(poly)):
        if intersection_exists(line, poly[i] + poly[i - 1]):
            return True
    return False


def euclidean_distance(pt1, pt2):
    if type(pt1) is not np.ndarray:
        pt1 = np.array(pt1)

    if type(pt2) is not np.ndarray:
        pt2 = np.array(pt2)

    return np.linalg.norm(pt1-pt2)
