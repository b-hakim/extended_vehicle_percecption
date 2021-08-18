from typing import List

import cv2
import sumolib
import numpy as np

from vehicle_info import Vehicle


class Utils:
    @staticmethod
    def reverse_y_axis(poly, max_height):
        poly[..., 1] = max_height - poly[..., 1]

        return poly
        # return [[p[0], max_height-p[1]] for p in poly]
    # def get_boundaries(self, poly):
    @staticmethod
    def adjust_padding(poly, padding):
        poly[..., 0] += padding
        poly[..., 1] += padding
        return poly

    @staticmethod
    def sumo2opencv_coord(poly, shape, padding, scale=10):
        poly = Utils.reverse_y_axis(poly, shape[0]/scale - 2 * padding)
        poly = Utils.adjust_padding(poly, padding)
        poly *= scale
        return np.round(poly).astype(int)

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
        self.padding = 100
        self.img = np.ones((10*(int(ymax+1)+self.padding*2), 10*(int(xmax+1) + self.padding*2), 3), dtype=np.uint8) * 255
        buildings.sort(key=lambda x:x.layer)

        for building in buildings:
            poly = np.array(building.shape).reshape(1, -1, 2)
            poly = Utils.sumo2opencv_coord(poly, self.img.shape, self.padding)

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
            poly = vehicle.get_vehicle_boundaries()
            poly = np.round(poly).astype(int).reshape(1, -1, 2)
            poly = Utils.sumo2opencv_coord(poly, self.img.shape, self.padding)
            cv2.fillPoly(self.img, poly, (128, 128, 128))
            pos = Utils.sumo2opencv_coord(np.array(vehicle.pos), self.img.shape, self.padding)
            print(pos)
            cv2.circle(self.img, tuple(np.array(pos).astype(int).tolist()), 2, (0, 0, 255))
            magnitude = np.linalg.norm([self.dimension[0] / 2, self.dimension[1] / 2])
            heading_point = magnitude * self.heading_unit_vector + self.pos
            line = tuple(np.array(pos).astype(int).tolist()), tuple(heading_point.astype(int).tolist())
            cv2.line(self.img, line[0], line[1], (0, 0, 255))

    def save_img(self, img_path="./map.png"):
        cv2.imwrite(img_path, self.img)


if __name__ == '__main__':
    hyper_params = {}
    hyper_params['scenario_path'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/test.net.xml"
    hyper_params['scenario_polys'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/map.poly.xml"

    SumoVisualizer(hyper_params)


