import random
from typing import Type

import traci

from math_utils import euclidean_distance, inner_angle_between_two_vectors, get_vector, does_line_intersect_polygon, \
    in_and_near_edge, get_dist_from_to, move_point, get_new_abs_pos

import numpy as np


class Vehicle:
    def __init__(self, vehicle_id, view_range, fov):
        self.dimension = (traci.vehicle.getWidth(vehicle_id),
                          traci.vehicle.getLength(vehicle_id),
                          traci.vehicle.getHeight(vehicle_id))

        self.vehicle_id = vehicle_id
        self.viewing_range = view_range
        self.fov = fov
        self.__previous_edge_road = traci.vehicle.getRoute(self.vehicle_id)[0]
        self.gps_pos_error = None
        self._pos = None
        self._center = None
        self._speed = None
        self._acc = None
        self._orientation_ang_degree = None
        self._heading_unit_vec = None

    def set_gps_error(self, noise_distance):
        if noise_distance == 0:
            return
        angle = np.random.randint(0, 360)

        self.gps_pos_error = [np.sin(np.deg2rad(angle)) * noise_distance,
                              np.cos(np.deg2rad(angle)) * noise_distance]

    def get_pos(self, with_gps_error=True):
        pos = list(traci.vehicle.getPosition(self.vehicle_id))
        self._pos = pos

        if self.gps_pos_error is None or not with_gps_error:
            return pos

        return [pos[0] + self.gps_pos_error[0],
                pos[1] + self.gps_pos_error[1]]

    @property
    def center_pos(self):
        def rotate_vector(vec, ceta_degree, pivot):
            vec -= pivot.reshape((2, 1))
            ceta_rad = np.deg2rad(ceta_degree)
            rot_mat = np.array([[np.cos(ceta_rad), -np.sin(ceta_rad)],
                                [np.sin(ceta_rad), np.cos(ceta_rad)]])
            rotated_vector = np.dot(rot_mat, vec)
            rotated_vector += pivot.reshape((2, 1))
            return rotated_vector

        center = np.array([
            [self.get_pos()[0], self.get_pos()[1] - self.dimension[1] / 2],
        ])

        ang = self.orientation_angle_degree - 90
        center = rotate_vector(center.transpose(), ang, np.array(self.get_pos())).transpose()
        self._center = center[0]
        return center[0]

    @property
    def speed(self):
        speed = traci.vehicle.getSpeed(self.vehicle_id)
        self._speed = speed
        return speed

    @property
    def acceleration(self):
        acc =  traci.vehicle.getAcceleration(self.vehicle_id)
        self._acc = acc
        return acc

    @property
    def orientation_angle_degree(self):
        vehicle_angle_degree = traci.vehicle.getAngle(self.vehicle_id)
        # - angle for making it ccw
        # +90 for the desired angle is based on the x axis while traci has angle based on y axis
        vehicle_angle_degree = (- vehicle_angle_degree + 90) % 360

        # if vehicle_angle_degree < 0:
        #     vehicle_angle_degree += 360
        self._orientation_ang_degree = vehicle_angle_degree
        return vehicle_angle_degree

    @property
    def heading_unit_vector(self):
        heading_unit_vector = [np.cos(self.orientation_angle_degree * np.pi / 180),
                               np.sin(self.orientation_angle_degree * np.pi / 180)]
        # heading_unit_vector = heading_unit_vector / np.sqrt(np.dot(heading_unit_vector, heading_unit_vector))
        self._heading_unit_vec = np.array(heading_unit_vector)
        return np.array(heading_unit_vector)

    @staticmethod
    def dist_between_edges(first_edge, next_edge):
        r = traci.simulation.findRoute(first_edge._id, next_edge._id)
        # assert len(r.edges) == 2 or len(r.edges) == 1
        return r.length - first_edge.getLength() - next_edge.getLength()

    @staticmethod
    def get_route_travel_time(route, net, veh_pos):
        d = 0
        prev_edge = None

        for edge_id in route:
            curr_edge = net.getEdge(edge_id)
            d += curr_edge.getLength()

            if in_and_near_edge(veh_pos, curr_edge.getShape()):
                d -= curr_edge._length
                d += get_dist_from_to(veh_pos, curr_edge.getShape()[-1], curr_edge.getShape())

            if prev_edge is not None:
                junction_dist = Vehicle.dist_between_edges(prev_edge, curr_edge)
                d += junction_dist

            prev_edge = curr_edge

        return 3600 * (d / 40000)

    def get_future_route(self, net, time_threshold):
        full_route = traci.vehicle.getRoute(self.vehicle_id)
        current_road = traci.vehicle.getRoadID(self.vehicle_id)

        if current_road[0] == ":":
            current_road = self.__previous_edge_road
            start = full_route.index(current_road) + 1
        else:
            start = full_route.index(current_road)

        future_route = full_route[start:]

        ## make sure this road is reacheable within 10 sec at most for 40km/h i.e. ~111 meters
        trimmed_future_route = []

        for edge in future_route:
            if Vehicle.get_route_travel_time(trimmed_future_route + [edge], net, self.get_pos()) <= time_threshold:
                trimmed_future_route.append(edge)
            else:
                break

        current_road = traci.vehicle.getRoadID(self.vehicle_id)

        if current_road[0] == ":":
            trimmed_future_route = [current_road] + trimmed_future_route

        return trimmed_future_route

    def update_latest_edge_road(self, new_road_id):
        if new_road_id[0] != ":":
            self.__previous_edge_road = new_road_id

    def get_current_road(self):
        return traci.vehicle.getRoadID(self.vehicle_id)

    def vehicle_in_sight(self, obstacle_vehicle, destination_vehicle, gps_error):
        # assert False ## need to find static obstacles to destination
        # intersect between line to destination and obj box

        destination_vehicle_corners = destination_vehicle.get_vehicle_boundaries(False) # False as this is always a nav

        # While the following shall be True, it is set to False as it is desired to get
        # what is actually seen with camera whether or not there is an issue in the sender GPS information
        obstacle_vehicle_corners = [v.tolist() for v in obstacle_vehicle.get_vehicle_boundaries(False)]

        # Get a line from my pos to the 4 corners of the destination
        # check if any of these passes through the box of a car, if at least 3 corners are invisible,
        # then it the object occludes the destination

        lines = [list(self.get_pos(gps_error)) + destination_vehicle_corners[0].tolist(),
                 list(self.get_pos(gps_error)) + destination_vehicle_corners[1].tolist(),
                 list(self.get_pos(gps_error)) + destination_vehicle_corners[2].tolist(),
                 list(self.get_pos(gps_error)) + destination_vehicle_corners[3].tolist()]

        invisibilities = np.array([does_line_intersect_polygon(line, obstacle_vehicle_corners) for line in lines])
        return np.count_nonzero(invisibilities) >= 3

    def get_vehicle_boundaries(self, with_gps_error):
        # calculate 4 vectors based on v's heading to get vectors to the 4 corners

        def rotate_vector(vec, ceta_degree, pivot):
            vec -= pivot.reshape((2, 1))
            ceta_rad = np.deg2rad(ceta_degree)
            rot_mat = np.array([[np.cos(ceta_rad), -np.sin(ceta_rad)],
                                [np.sin(ceta_rad), np.cos(ceta_rad)]])
            rotated_vector = np.dot(rot_mat, vec)
            rotated_vector += pivot.reshape((2, 1))
            return rotated_vector

        corners = np.array([
            [self.get_pos(with_gps_error)[0] - self.dimension[0] / 2, self.get_pos(with_gps_error)[1]],
            [self.get_pos(with_gps_error)[0] - self.dimension[0] / 2, self.get_pos(with_gps_error)[1] - self.dimension[1]],
            [self.get_pos(with_gps_error)[0] + self.dimension[0] / 2, self.get_pos(with_gps_error)[1] - self.dimension[1]],
            [self.get_pos(with_gps_error)[0] + self.dimension[0] / 2, self.get_pos(with_gps_error)[1]],
        ])

        # center = corners.mean(axis=0)
        ang = self.orientation_angle_degree - 90
        corners = rotate_vector(corners.transpose(), ang, np.array(self.get_pos(with_gps_error))).transpose()
        # np.array([
        #     rotate_vector(corners[0], ang),
        #     rotate_vector(corners[1], ang),
        #     rotate_vector(corners[2], ang),
        #     rotate_vector(corners[3], ang),
        #     ])
        return corners

    '''
    Checks if the vehicle is in the perception range of AV
    '''

    def has_in_perception_range(self, vehicle, sender_with_gps_error, object_with_gps_error, detection_probability=1.0, noise=None):
        if random.random() > detection_probability:
            return False

        # First check that the distance to the vehicle is less than the given range
        vehicle_corners = vehicle.get_vehicle_boundaries(object_with_gps_error)

        if noise is not None:
            vehicle_corners = [get_new_abs_pos(noise[0], noise[1], c) for c in vehicle_corners]

        dists = [
            euclidean_distance(self.get_pos(sender_with_gps_error), vehicle_corners[0]),
            euclidean_distance(self.get_pos(sender_with_gps_error), vehicle_corners[1]),
            euclidean_distance(self.get_pos(sender_with_gps_error), vehicle_corners[2]),
            euclidean_distance(self.get_pos(sender_with_gps_error), vehicle_corners[3])
        ]

        if np.array([d > self.viewing_range for d in dists]).all():
            return False

        # Second check that the vehicle is in the given FoV
        angles = [abs(inner_angle_between_two_vectors(self.heading_unit_vector,
                                                      get_vector(self.get_pos(sender_with_gps_error), vehicle_corners[0]))),
                  abs(inner_angle_between_two_vectors(self.heading_unit_vector,
                                                      get_vector(self.get_pos(sender_with_gps_error), vehicle_corners[1]))),
                  abs(inner_angle_between_two_vectors(self.heading_unit_vector,
                                                      get_vector(self.get_pos(sender_with_gps_error), vehicle_corners[2]))),
                  abs(inner_angle_between_two_vectors(self.heading_unit_vector,
                                                      get_vector(self.get_pos(sender_with_gps_error), vehicle_corners[3])))]

        return np.array([angle <= self.fov / 2 for angle in angles]).any()

    def can_see_building(self, building):
        # First check that the distance to the vehicle is less than the given range
        building_corners = building.shape

        dists = [euclidean_distance(self.get_pos(), c) for c in building_corners]

        if np.array([d > self.viewing_range for d in dists]).all():
            return False

        # Second check that the vehicle is in the given FoV
        angles = [abs(inner_angle_between_two_vectors(self.heading_unit_vector, get_vector(self.get_pos(), c)))
                  for c in building_corners]

        return np.array([angle <= self.fov / 2 for angle in angles]).any()

    def calculate_probability_av_sees_nav(self, av, nav, vehicles_in_my_perception_range,
                                          buildings, detection_probability, continous_probability):
        # First make sure that the cv2x can see the non_cv2x i.e. inside viewing range and FoV
        # If the cv2x and the non_cv2x are both located inside the viewing range and FoV of self, then
        # Let L = list of objects occluding cv2x and noncv2x
        # If I can see any of these objects, then return 0, if any of these objects is occluded return 0.5
        # Else: (case no objects occluding but as a sender I might not know!)
        # interpolate the line from center_cv2x and noncv2x corners into set of points
        # if i cannot see any of these points due to occlusion with another object:
        # then return 0.5
        # Else return 1 (no objects between them and I can see all points)
        # Else
        # return 0.5

        # Validate that the vehicle is in the range of av receiver
        # receiver av must include its noise as this is estimated relative to the sender av which knows the receiver pos
        # with errors in the pos (aka through cv2x)
        if not av.has_in_perception_range(nav, True, False, detection_probability=1,
                                          noise=(self.get_pos(False), self.get_pos())):
            return 0

        if not continous_probability:
            uncertain_probability = 0.5
        else:
            d = euclidean_distance(av.get_pos(), nav.get_pos())
            if d < 1:
                d = 1
            uncertain_probability = 1/d

        if not self.has_in_perception_range(av, True, True, detection_probability=1):
            return uncertain_probability
            # return 0.5

        # non_cv2x_vehicle_corners = nav.get_vehicle_boundaries(False) # False as this is a nav
        #
        # lines = [list(av.get_pos()) + get_new_abs_pos(self.get_pos(False), self.get_pos(),
        #                                               non_cv2x_vehicle_corners[0].tolist()),
        #          list(av.get_pos()) + get_new_abs_pos(self.get_pos(False), self.get_pos(),
        #                                               non_cv2x_vehicle_corners[1].tolist()),
        #          list(av.get_pos()) + get_new_abs_pos(self.get_pos(False), self.get_pos(),
        #                                               non_cv2x_vehicle_corners[2].tolist()),
        #          list(av.get_pos()) + get_new_abs_pos(self.get_pos(False), self.get_pos(),
        #                                               non_cv2x_vehicle_corners[3].tolist())]

        LoS = list(av.get_pos()) + get_new_abs_pos(self.get_pos(False), self.get_pos(), nav.get_pos())

        # for LoS in lines:
        # list of objects occluding cv2x and noncv2x
        for occlusion_vehicle in vehicles_in_my_perception_range:
            if occlusion_vehicle.vehicle_id == av.vehicle_id or occlusion_vehicle.vehicle_id == nav.vehicle_id:
                continue
            occlusion_vehicle_corners = [get_new_abs_pos(self.get_pos(False), self.get_pos(), v)
                                         for v in occlusion_vehicle.get_vehicle_boundaries(True)] # True as this can be an av

            if does_line_intersect_polygon(LoS, occlusion_vehicle_corners):
                # If I can see any of these objects,
                # then return 0,
                # if any of these objects is occluded return 0.5
                line_sender_to_occluding_object = list(self.get_pos()) + get_new_abs_pos(self.get_pos(False),
                                                                                         self.get_pos(),
                                                                                         occlusion_vehicle.get_pos())
                for occlusion_vehicle_2 in vehicles_in_my_perception_range:
                    if occlusion_vehicle_2.vehicle_id == av.vehicle_id \
                            or occlusion_vehicle_2.vehicle_id == nav.vehicle_id \
                            or occlusion_vehicle_2.vehicle_id == occlusion_vehicle.vehicle_id:
                        continue

                    occlusion_vehicle2_corners = [get_new_abs_pos(self.get_pos(False), self.get_pos(), v) for v in
                                                  occlusion_vehicle_2.get_vehicle_boundaries(True)] # True as this can be an av
                    if does_line_intersect_polygon(line_sender_to_occluding_object, occlusion_vehicle2_corners):
                        if random.random() > detection_probability:  # not perceived
                            continue
                        else:
                            return uncertain_probability
                            # return 0.5
                    else:
                        # sender can see that there is an occluder between receiver AV and the nAV, thus return nLOS
                        if random.random() > detection_probability:  # not perceived
                            continue
                        else:
                            return 0

        buildings_in_sight = []

        for building in buildings:
            if self.can_see_building(building):
                buildings_in_sight.append(building)

        interpolations = np.linspace(LoS[0:2],
                                     LoS[2:4],
                                     int(euclidean_distance(LoS[0:2], LoS[2:4]) * 2))

        for occlusion_vehicle in vehicles_in_my_perception_range:
            if occlusion_vehicle.vehicle_id == av.vehicle_id \
                    or occlusion_vehicle.vehicle_id == nav.vehicle_id:
                continue

            occlusion_vehicle_corners = [get_new_abs_pos(self.get_pos(False), self.get_pos(), v)
                                         for v in occlusion_vehicle.get_vehicle_boundaries(True)]
            for point in interpolations:
                line_sender_to_interpolated_point = list(self.get_pos()) + point.tolist()

                if does_line_intersect_polygon(line_sender_to_interpolated_point, occlusion_vehicle_corners):
                    if random.random() > detection_probability:  # not perceived
                        continue
                    else:
                        return uncertain_probability
                        # return 0.5

        for building in buildings_in_sight:
            for point in interpolations:
                line_sender_to_interpolated_point = list(self.get_pos()) + point.tolist()

                if does_line_intersect_polygon(line_sender_to_interpolated_point, building.shape):
                    return uncertain_probability
                    # return 0.5

            # self can see these points, but need to validate if an occlusion occurs.

            # can be speeded up by reducing the buildings to only those in the viewing_range and FoV of self

        return 1

    def building_in_sight(self, building, gps_error, destination_vehicle):
        destination_vehicle_corners = destination_vehicle.get_vehicle_boundaries(False) # False as it is always a nav

        # Get a line from my pos to the 4 corners of the destination
        # check if any of these passes through the box of a car, if at least 3 corners are invisible,
        # then the object occludes the destination

        lines = [list(self.get_pos(gps_error)) + destination_vehicle_corners[0].tolist(),
                 list(self.get_pos(gps_error)) + destination_vehicle_corners[1].tolist(),
                 list(self.get_pos(gps_error)) + destination_vehicle_corners[2].tolist(),
                 list(self.get_pos(gps_error)) + destination_vehicle_corners[3].tolist()]

        invisibilities = np.array([does_line_intersect_polygon(line, building) for line in lines])
        return np.count_nonzero(invisibilities) >= 3


if __name__ == '__main__':
    heading_unit_vector = [0, 1]
    a1 = inner_angle_between_two_vectors(heading_unit_vector, get_vector([0, 0], [4, -2]))
    a2 = inner_angle_between_two_vectors(get_vector([0, 0], [4, -2]), heading_unit_vector)
    print(a1, a2)
    x = 1
