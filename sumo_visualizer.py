from typing import List

import cv2
import sumolib
import numpy as np

from vehicle_info import Vehicle


class Utils:
    @staticmethod
    def reverse_y_axis(poly, max_height):
        poly[..., 1] = float(max_height) - poly[..., 1]

        return poly
        # return [[p[0], max_height-p[1]] for p in poly]
    # def get_boundaries(self, poly):
    # @staticmethod
    # def adjust_padding(poly, padding):
    #     poly[..., 0] += padding
    #     poly[..., 1] += padding
    #     return poly

    @staticmethod
    def sumo2opencv_coord(poly, shape, scale=10):
        poly = Utils.reverse_y_axis(poly, shape[0]/scale)
        # poly = Utils.adjust_padding(poly, padding)
        poly *= scale
        return poly.astype(int)

class SumoVisualizer:
    '''
    1- Get the size of the scenario
    2- Initialize an image with the scenario size
    3- Draw buildings as poly
    4- Draw Vehicles as poly based on its boundaries
    '''
    def __init__(self, paths):
        net = sumolib.net.readNet(paths['scenario_path'])
        buildings = sumolib.shapes.polygon.read(paths['scenario_polys'])
        edges = net.getEdges()
        junctions = net.getNodes()
        polys = np.concatenate([building.shape for building in buildings]
                               + [edge.getShape() for edge in edges]
                               + [junction.getShape() for junction in junctions], axis=0)

        xmax, ymax = polys[:, 0].max(), polys[:, 1].max()
        # self.padding = 100
        self.img = np.ones((10*(int(ymax+1)), 10*(int(xmax+1)), 3), dtype=np.uint8) * 255
        buildings.sort(key=lambda x:x.layer)

        for building in buildings:
            poly = np.array(building.shape).reshape(1, -1, 2)
            poly = Utils.sumo2opencv_coord(poly, self.img.shape)

            if building.fill == '1':
                cv2.fillPoly(self.img, poly, (building.color.b, building.color.g, building.color.r))
            else:
                cv2.polylines(self.img, poly, False, (building.color.b, building.color.g, building.color.r))

        # for edge in edges:
        #     polylines = np.array(edge.getShape()).astype(int).reshape(1, -1, 2)
        #     polylines = Utils.sumo2opencv_coord(polylines, self.img.shape, self.padding)
        #     cv2.polylines(self.img, polylines, False, (0, 0, 0), thickness=3)

    def draw_vehicles(self, vehicles:List[Vehicle]):
        for vehicle in vehicles:
            self.draw_vehicle_body(vehicle)

            pos = Utils.sumo2opencv_coord(np.array(vehicle.pos), self.img.shape)
            cv2.circle(self.img, tuple(np.array(pos).astype(int).tolist()), 1, (0, 255, 0), -1)

            magnitude = np.linalg.norm([vehicle.dimension[0] / 2, vehicle.dimension[1] / 2])
            heading_point = magnitude * vehicle.heading_unit_vector + vehicle.pos
            heading_point = Utils.sumo2opencv_coord(np.array(heading_point), self.img.shape)
            line = tuple(np.array(pos).astype(int).tolist()), tuple(heading_point.astype(int).tolist())
            cv2.line(self.img, line[0], line[1], (0, 255, 0))

    def draw_vehicle_perception(self, vehicle, color):
        self.draw_vehicle_body(vehicle, color)
        semi_viewing_fov = vehicle.fov/2

        a1 = (vehicle.orientation_angle_degree - semi_viewing_fov) % 360
        a2 = (vehicle.orientation_angle_degree + semi_viewing_fov) % 360

        right_limit = np.array([np.cos(a1 * np.pi / 180),
                               np.sin(a1 * np.pi / 180)])
        left_limit = np.array([np.cos(a2 * np.pi / 180),
                               np.sin(a2 * np.pi / 180)])

        right_pt = vehicle.viewing_range * right_limit + np.array(vehicle.pos)
        left_pt = vehicle.viewing_range * left_limit + np.array(vehicle.pos)

        pos = Utils.sumo2opencv_coord(np.array(vehicle.pos), self.img.shape)
        right_pt = Utils.sumo2opencv_coord(right_pt, self.img.shape)
        left_pt = Utils.sumo2opencv_coord(left_pt, self.img.shape)

        cv2.line(self.img, tuple(pos.astype(int)), tuple(right_pt.astype(int)), (0, 0, 0))
        cv2.line(self.img, tuple(pos.astype(int)), tuple(left_pt.astype(int)), (0, 0, 0))

        center = (int(pos[0]), int(pos[1]))
        axes = (vehicle.viewing_range*10, vehicle.viewing_range*10)

        # Make angles CW for ellipse
        startAngle = 360 - a1
        endAngle = 360 - a2

        if abs(endAngle - startAngle) > 180:
            startAngle = (startAngle - 180) % 360
            endAngle = (endAngle + 180) % 360
            ang = 180
        else:
            ang = 0

        cv2.ellipse(self.img, center, axes, ang, startAngle, endAngle, (0,0,0))


    def save_img(self, img_path="./map.png"):
        cv2.imwrite(img_path, self.img)

    def draw_vehicle_body(self, vehicle, color=(128, 128, 127)):
        poly = vehicle.get_vehicle_boundaries()
        poly = poly.reshape(1, -1, 2)
        poly = Utils.sumo2opencv_coord(poly, self.img.shape)
        cv2.fillPoly(self.img, poly, color)


if __name__ == '__main__':
    hyper_params = {}
    hyper_params['scenario_path'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/test.net.xml"
    hyper_params['scenario_polys'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/map.poly.xml"

    SumoVisualizer(hyper_params)


