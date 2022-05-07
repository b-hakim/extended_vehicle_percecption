import random

import numpy as np


def euclidean_distance(p1, p2):
    if type(p1) is not np.ndarray:
        p1 = np.array(p1)

    if type(p2) is not np.ndarray:
        p2 = np.array(p2)

    return np.linalg.norm(p1-p2)


def inner_angle_between_two_vectors(v1, v2):
    angle1 = angle_between_two_vectors(v1, v2)
    angle2 = angle_between_two_vectors(v2, v1)

    return angle1 if angle1 < angle2 else angle2


def angle_between_two_vectors(v1, v2):
    angle = np.arctan2(np.cross(v1, v2), np.dot(v1, v2))
    angle = np.rad2deg(angle)

    if angle < 0:
        angle += 360

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


def get_slope_y_intercept(line):
    (x1, y1), (x2, y2) = line

    if abs(x2 - x1) < 0.0001:
        slope = None
        c = x1
    else:
        if abs(y2 - y1) < 0.0001:
            y2 = y1
        slope = (y2 - y1) / (x2 - x1)
        c = y1 - slope * x1

    return slope, c


def distance_from_point_2_line(pt, line_param):
    if line_param[0] is None:
        return abs(line_param[1] - pt[0])

    return abs(line_param[0] * pt[0] - pt[1] + line_param[1]) / np.sqrt(line_param[0] * line_param[0] + 1)


def in_segment(pt, line):
    # line_param = get_slope_y_intercept(line)
    # d = distance_from_point_2_line (pt, line_param)
    # near_line = d < 0.5
    minx, miny, maxx, maxy = min(line[0][0],line[1][0]), min(line[0][1],line[1][1]), \
                             max(line[0][0],line[1][0]), max(line[0][1],line[1][1])

    in_segment = minx - 0.5 < pt[0] < maxx + 0.5 and miny - 0.5 < pt[1] < maxy + 0.5

    # return near_line and in_segment
    return in_segment


def in_and_near_edge(cv2x_veh_pos, edge_segments):
    for i in range(1, len(edge_segments)):
        line = edge_segments[i-1], edge_segments[i]

        if in_segment(cv2x_veh_pos, line):
            return True

    return False


def get_dist_from_to(pos_from, pos_to, edge_segments):
    dist = 0
    first_pt_found = False

    for i in range(1, len(edge_segments)):
        line = edge_segments[i-1], edge_segments[i]

        if not first_pt_found:
            if in_segment(pos_from, line):
                first_pt_found = True

                if in_segment(pos_to, line):
                    return euclidean_distance(pos_from, pos_to)
                else:
                    dist += euclidean_distance(pos_from, line[1])
        else:
            if in_segment(pos_to, line):
                dist += euclidean_distance(line[0], pos_to)
                return dist
            else:
                dist += euclidean_distance(line[0], line[1])

    assert False


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


def get_polygon_area(poly_pts):
    sum1, sum2 = 0, 0
    # poly_pts = [(-3, -2), (-1, 4), (6, 1), (3, 10), (-4, 9)][::-1]
    # poly_pts.insert(0, poly_pts[-1])
    poly_pts.append(poly_pts[0])

    # for i in range(-1, -len((poly_pts)), -1):
    for i in range(len((poly_pts))-1):
        sum1 += poly_pts[i][0]*poly_pts[i+1][1]
        sum2 += poly_pts[i][1]*poly_pts[i+1][0]

    area = (sum2-sum1)/2

    if area < 0:
        print("area less than a zero")
        exit()

    return area
