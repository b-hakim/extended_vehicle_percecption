from typing import Type

import traci

from math_utils import euclidean_distance, angle_between_two_vectors, get_vector, does_line_intersect_polygon
import numpy as np


class Vehicle:
    def __init__(self, future_routes, dimension, pos, speed, acc, vehicle_id, vehicle_angle_degree,
                 perception_range, perception_angle):
        self.future_routes = future_routes
        self.dimension = dimension
        self.pos = pos
        self.speed = speed
        self.acceleration = acc

        self.heading_unit_vector = [np.cos(vehicle_angle_degree * np.pi / 180),
                                    np.sin(vehicle_angle_degree * np.pi / 180)]
        self.heading_unit_vector = self.heading_unit_vector / np.sqrt(np.dot(self.heading_unit_vector, self.heading_unit_vector))

        self.vehicle_id = vehicle_id
        self.perception_range = perception_range
        self.perception_angle = perception_angle


    def vehicle_in_sight(self, obstacle_vehicle, destination_vehicle):
        # assert False ## need to find static obstacles to destination
        #intersect between line to destination and obj box

        destination_vehicle_corners = Vehicle.get_vehicle_boundaries(destination_vehicle)
        obstacle_vehicle_corners = Vehicle.get_vehicle_boundaries(obstacle_vehicle)

        # Get a line from my pos to the 4 corners of the destination
        # check if any of these passes through the box of a car, if at least 3 corners are invisible,
        # then it the object occludes the destination

        lines = [[self.pos, destination_vehicle_corners[0]],
                 [self.pos, destination_vehicle_corners[1]],
                 [self.pos, destination_vehicle_corners[2]],
                 [self.pos, destination_vehicle_corners[3]]]

        invisibilities = np.array([does_line_intersect_polygon(line, obstacle_vehicle_corners) for line in lines])
        return np.count_nonzero(invisibilities) >= 3

    @staticmethod
    def get_vehicle_boundaries(v):
        # calculate 4 vectors based on v's heading to get vectors to the 4 corners
        def rotate_vector(vec, ceta_degree):
            ceta_rad = np.deg2rad(ceta_degree)
            rot_mat = np.array([[np.cos(ceta_rad), -np.sin(ceta_rad)],
                                [np.sin(ceta_rad), np.cos(ceta_rad)]])
            rotated_vector = np.dot(rot_mat, vec)
            return rotated_vector

        magnitude = np.linalg.norm([v.dimension[0]/2, v.dimension[1]/2])
        corners = [magnitude * rotate_vector(v.heading_unit_vector, 45) + v.pos,
                   magnitude * rotate_vector(v.heading_unit_vector, 135)+ v.pos,
                   magnitude * rotate_vector(v.heading_unit_vector, -135)+ v.pos,
                   magnitude * rotate_vector(v.heading_unit_vector, -45)+ v.pos]

        return corners

    def can_see_vehicle(self, non_cv2x_vehicle):
        # First check that the distance to the vehicle is less than the given range
        non_cv2x_vehicle_corners = Vehicle.get_vehicle_boundaries(non_cv2x_vehicle)

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
                                           potential_occlusion_vehicles_ids, buildings):
        # First make sure that the cv2x and the non_cv2x are both located inside the viewing range and FoV of self, then
        # if distance between cv2x and noncv2x > 75 then return 0
        # if noncv2x is out of FoV range of cv2x, then return 0
        # interpolate the line from center_cv2x and noncv2x corners into set of points
        # if i cannot see any of these points due to occlusion with another object or outside my range, then return 0.5
        if not self.can_see_vehicle(cv2x_vehicle) or not self.can_see_vehicle(non_cv2x_vehicle):
            return 0.5

        non_cv2x_vehicle_corners = Vehicle.get_vehicle_boundaries(non_cv2x_vehicle)

        dists = [
            euclidean_distance(cv2x_vehicle.pos, non_cv2x_vehicle_corners[0]),
            euclidean_distance(cv2x_vehicle.pos, non_cv2x_vehicle_corners[1]),
            euclidean_distance(cv2x_vehicle.pos, non_cv2x_vehicle_corners[2]),
            euclidean_distance(cv2x_vehicle.pos, non_cv2x_vehicle_corners[3])
        ]

        if np.array([d > cv2x_vehicle.perception_range for d in dists]).all():
            return 0

        # Second check that the vehicle is in the given FoV
        angles = [abs(angle_between_two_vectors(cv2x_vehicle.heading_unit_vector,
                                                get_vector(cv2x_vehicle.pos, non_cv2x_vehicle_corners[0]))),
                  abs(angle_between_two_vectors(cv2x_vehicle.heading_unit_vector,
                                                get_vector(cv2x_vehicle.pos, non_cv2x_vehicle_corners[1]))),
                  abs(angle_between_two_vectors(cv2x_vehicle.heading_unit_vector,
                                                get_vector(cv2x_vehicle.pos, non_cv2x_vehicle_corners[2]))),
                  abs(angle_between_two_vectors(cv2x_vehicle.heading_unit_vector,
                                                get_vector(cv2x_vehicle.pos, non_cv2x_vehicle_corners[3])))]

        if np.array([angle > cv2x_vehicle.perception_angle / 2 for angle in angles]).all():
            return 0

        lines = [[cv2x_vehicle.pos, non_cv2x_vehicle_corners[0]],
                 [cv2x_vehicle.pos, non_cv2x_vehicle_corners[1]],
                 [cv2x_vehicle.pos, non_cv2x_vehicle_corners[2]],
                 [cv2x_vehicle.pos, non_cv2x_vehicle_corners[3]]]

        for cv2x_to_non_cv2x_corner_line in lines:
            interpolations = np.linspace(cv2x_to_non_cv2x_corner_line[0], cv2x_to_non_cv2x_corner_line[1],
                                         euclidean_distance(cv2x_to_non_cv2x_corner_line[0],
                                                            cv2x_to_non_cv2x_corner_line[1])*2)

            for point in interpolations:
                # self can see these points, but need to validate if an occlusion occurs.
                for occlusion_vehicle_id in potential_occlusion_vehicles_ids:
                    line_self_to_interpolated_point = [self.pos, point]
                    occlusion_vehicle = Vehicle(future_routes=[],
                                                dimension=(traci.vehicle.getWidth(occlusion_vehicle_id),
                                                           traci.vehicle.getLength(occlusion_vehicle_id),
                                                           traci.vehicle.getHeight(occlusion_vehicle_id)),
                                                pos=traci.vehicle.getPosition(occlusion_vehicle_id),
                                                speed=traci.vehicle.getSpeed(occlusion_vehicle_id),
                                                acc=traci.vehicle.getAcceleration(occlusion_vehicle_id),
                                                vehicle_id=occlusion_vehicle_id,
                                                vehicle_angle_degree=traci.vehicle.getAngle(occlusion_vehicle_id),
                                                perception_range=-1, perception_angle=-1)
                    occlusion_vehicle_corners = Vehicle.get_vehicle_boundaries(occlusion_vehicle)

                    if does_line_intersect_polygon(line_self_to_interpolated_point, occlusion_vehicle_corners):
                        return 0.5

                    # can be speeded up by reducing the buildings to only those in the viewing_range and FoV of self
                    for building in buildings:
                        if does_line_intersect_polygon(line_self_to_interpolated_point, building):
                            return 0.5
        return 1




    def building_in_sight(self, building, destination_vehicle):
        destination_vehicle_corners = Vehicle.get_vehicle_boundaries(destination_vehicle)

        # Get a line from my pos to the 4 corners of the destination
        # check if any of these passes through the box of a car, if at least 3 corners are invisible,
        # then the object occludes the destination

        lines = [[self.pos, destination_vehicle_corners[0]],
                 [self.pos, destination_vehicle_corners[1]],
                 [self.pos, destination_vehicle_corners[2]],
                 [self.pos, destination_vehicle_corners[3]]]

        invisibilities = np.array([does_line_intersect_polygon(line, building) for line in lines])
        return np.count_nonzero(invisibilities) >= 3


