from typing import Type

import traci

from math_utils import euclidean_distance, angle_between_two_vectors, get_vector, does_line_intersect_polygon
import numpy as np


class Vehicle:
    def __init__(self, vehicle_id, hyper_params):
        self.dimension = (traci.vehicle.getWidth(vehicle_id),
                          traci.vehicle.getLength(vehicle_id),
                          traci.vehicle.getHeight(vehicle_id))

        self.vehicle_id = vehicle_id
        self.perception_range = hyper_params["view_range"]
        self.perception_angle = hyper_params["fov"]
        self.__previous_edge_road = traci.vehicle.getRoute(self.vehicle_id)[0]

    @property
    def pos(self):
        return traci.vehicle.getPosition(self.vehicle_id)

    @property
    def speed(self):
        return traci.vehicle.getSpeed(self.vehicle_id)

    @property
    def acceleration(self):
        return traci.vehicle.getAcceleration(self.vehicle_id)

    @property
    def heading_unit_vector(self):
        vehicle_angle_degree = traci.vehicle.getAngle(self.vehicle_id)
        heading_unit_vector = [np.cos(vehicle_angle_degree * np.pi / 180),
                                    np.sin(vehicle_angle_degree * np.pi / 180)]
        heading_unit_vector = heading_unit_vector / np.sqrt(np.dot(heading_unit_vector, heading_unit_vector))
        return heading_unit_vector

    def get_future_route(self):
        full_route = traci.vehicle.getRoute(self.vehicle_id)
        current_road = traci.vehicle.getRoadID(self.vehicle_id)

        if current_road[0] == ":":
            current_road = self.__previous_edge_road

        start = full_route.index(current_road)
        return full_route[start:]

    def update_latest_edge_road(self, new_road_id):
        if new_road_id[0] != ":":
            self.__previous_edge_road = new_road_id

    def get_current_road(self):
        return traci.vehicle.getRoadID(self.vehicle_id)

    def vehicle_in_sight(self, obstacle_vehicle, destination_vehicle):
        # assert False ## need to find static obstacles to destination
        #intersect between line to destination and obj box

        destination_vehicle_corners = destination_vehicle.get_vehicle_boundaries()
        obstacle_vehicle_corners = [v.tolist() for v in obstacle_vehicle.get_vehicle_boundaries()]

        # Get a line from my pos to the 4 corners of the destination
        # check if any of these passes through the box of a car, if at least 3 corners are invisible,
        # then it the object occludes the destination

        lines = [list(self.pos) + destination_vehicle_corners[0].tolist(),
                 list(self.pos) + destination_vehicle_corners[1].tolist(),
                 list(self.pos) + destination_vehicle_corners[2].tolist(),
                 list(self.pos) + destination_vehicle_corners[3].tolist()]

        invisibilities = np.array([does_line_intersect_polygon(line, obstacle_vehicle_corners) for line in lines])
        return np.count_nonzero(invisibilities) >= 3

    def get_vehicle_boundaries(self):
        # calculate 4 vectors based on v's heading to get vectors to the 4 corners
        def rotate_vector(vec, ceta_degree):
            ceta_rad = np.deg2rad(ceta_degree)
            rot_mat = np.array([[np.cos(ceta_rad), -np.sin(ceta_rad)],
                                [np.sin(ceta_rad), np.cos(ceta_rad)]])
            rotated_vector = np.dot(rot_mat, vec)
            return rotated_vector

        magnitude = np.linalg.norm([self.dimension[0] / 2, self.dimension[1] / 2])
        corners = [magnitude * rotate_vector(self.heading_unit_vector, 45) + self.pos,
                   magnitude * rotate_vector(self.heading_unit_vector, 135) + self.pos,
                   magnitude * rotate_vector(self.heading_unit_vector, -135) + self.pos,
                   magnitude * rotate_vector(self.heading_unit_vector, -45) + self.pos]

        return corners

    def can_see_vehicle(self, non_cv2x_vehicle):
        # First check that the distance to the vehicle is less than the given range
        non_cv2x_vehicle_corners = non_cv2x_vehicle.get_vehicle_boundaries()

        dists = [
            euclidean_distance(self.pos, non_cv2x_vehicle_corners[0]),
            euclidean_distance(self.pos, non_cv2x_vehicle_corners[1]),
            euclidean_distance(self.pos, non_cv2x_vehicle_corners[2]),
            euclidean_distance(self.pos, non_cv2x_vehicle_corners[3])
        ]

        if np.array([d > self.perception_range for d in dists]).all():
            return False

        # Second check that the vehicle is in the given FoV
        angles = [abs(angle_between_two_vectors(self.heading_unit_vector, get_vector(self.pos, non_cv2x_vehicle_corners[0]))),
                  abs(angle_between_two_vectors(self.heading_unit_vector, get_vector(self.pos, non_cv2x_vehicle_corners[1]))),
                  abs(angle_between_two_vectors(self.heading_unit_vector, get_vector(self.pos, non_cv2x_vehicle_corners[2]))),
                  abs(angle_between_two_vectors(self.heading_unit_vector, get_vector(self.pos, non_cv2x_vehicle_corners[3])))]

        return np.array([angle <= self.perception_angle/2 for angle in angles]).any()

    def get_probability_cv2x_sees_non_cv2x(self, cv2x_vehicle, non_cv2x_vehicle,
                                           remaining_perceived_non_cv2x_vehicles, buildings):
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

        if not cv2x_vehicle.can_see_vehicle(non_cv2x_vehicle):
            return 0

        if not self.can_see_vehicle(cv2x_vehicle) or not self.can_see_vehicle(non_cv2x_vehicle):
            return 0.5

        non_cv2x_vehicle_corners = non_cv2x_vehicle.get_vehicle_boundaries()

        lines = [list(cv2x_vehicle.pos) + non_cv2x_vehicle_corners[0].tolist(),
                 list(cv2x_vehicle.pos) + non_cv2x_vehicle_corners[1].tolist(),
                 list(cv2x_vehicle.pos) + non_cv2x_vehicle_corners[2].tolist(),
                 list(cv2x_vehicle.pos) + non_cv2x_vehicle_corners[3].tolist()]

        for cv2x_to_non_cv2x_corner_line in lines:
            # list of objects occluding cv2x and noncv2x
            for occlusion_vehicle in remaining_perceived_non_cv2x_vehicles:
                occlusion_vehicle_corners = [v.tolist() for v in occlusion_vehicle.get_vehicle_boundaries()]
                if does_line_intersect_polygon(cv2x_to_non_cv2x_corner_line, occlusion_vehicle_corners):
                    # If I can see any of these objects,
                    # then return 0,
                    # if any of these objects is occluded return 0.5
                    line_self_to_object_interest = list(self.pos) + list(occlusion_vehicle.pos)
                    for occlusion_vehicle_2 in remaining_perceived_non_cv2x_vehicles:
                        if occlusion_vehicle_2.vehicle_id != occlusion_vehicle.vehicle_id:
                            occlusion_vehicle2_corners = [v.tolist() for v in
                                                         occlusion_vehicle_2.get_vehicle_boundaries()]
                            if does_line_intersect_polygon(line_self_to_object_interest, occlusion_vehicle2_corners):
                                return 0.5
                            else:
                                return 0

            interpolations = np.linspace(cv2x_to_non_cv2x_corner_line[0:2], cv2x_to_non_cv2x_corner_line[2:4],
                                         int(euclidean_distance(cv2x_to_non_cv2x_corner_line[0:2],
                                                                cv2x_to_non_cv2x_corner_line[2:4])*2))

            for point in interpolations:
                # self can see these points, but need to validate if an occlusion occurs.
                for occlusion_vehicle in remaining_perceived_non_cv2x_vehicles:
                    line_self_to_interpolated_point = list(self.pos) + point.tolist()
                    occlusion_vehicle_corners = [v.tolist() for v in occlusion_vehicle.get_vehicle_boundaries()]

                    if does_line_intersect_polygon(line_self_to_interpolated_point, occlusion_vehicle_corners):
                        return 0.5

                    # can be speeded up by reducing the buildings to only those in the viewing_range and FoV of self
                    for building in buildings:
                        if does_line_intersect_polygon(line_self_to_interpolated_point, building.shape):
                            return 0.5
        return 1

    def building_in_sight(self, building, destination_vehicle):
        destination_vehicle_corners = destination_vehicle.get_vehicle_boundaries()

        # Get a line from my pos to the 4 corners of the destination
        # check if any of these passes through the box of a car, if at least 3 corners are invisible,
        # then the object occludes the destination

        lines = [list(self.pos) + destination_vehicle_corners[0].tolist(),
                 list(self.pos) + destination_vehicle_corners[1].tolist(),
                 list(self.pos) + destination_vehicle_corners[2].tolist(),
                 list(self.pos) + destination_vehicle_corners[3].tolist()]

        invisibilities = np.array([does_line_intersect_polygon(line, building) for line in lines])
        return np.count_nonzero(invisibilities) >= 3
