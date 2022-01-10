import os, sys
import random
from ast import literal_eval
from typing import List, Dict

import numpy as np
import sumolib
import traci

from math_utils import euclidean_distance, in_and_near_edge, get_dist_from_to, in_segment
from solver import Solver
from sumo_visualizer import SumoVisualizer
from vehicle_info import Vehicle


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


def distance_prev_curr_edge(prev_edge_points, curr_edge_points):
    return np.min([euclidean_distance(prev_edge_points[0], curr_edge_points[0]),
                   euclidean_distance(prev_edge_points[0], curr_edge_points[-1]),
                   euclidean_distance(prev_edge_points[-1], curr_edge_points[0]),
                   euclidean_distance(prev_edge_points[-1], curr_edge_points[-1])])


class Simulation:
    def __init__(self, hyper_params, id):
        self.hyper_params = hyper_params
        self.net = sumolib.net.readNet(hyper_params['scenario_path'])
        self.sim_id = id

    def run(self):
        import traci

        sumoBinary = "/usr/bin/sumo"
        # sumoBinary = "/usr/bin/sumo-gui"
        sumoCmd = [sumoBinary, "-c", self.hyper_params['scenario_map']]
        traci.start(sumoCmd)

        buildings = sumolib.shapes.polygon.read(self.hyper_params['scenario_polys'])
        tmp = []

        for building in buildings:
            if building.type != "unknown":
                tmp.append(building)

        step=0
        n_not_0 = False
        max_n = 0

        while step < 100000:
            step += 1

            traci.simulationStep()
            traci.route.getIDList()

            # 1) Get All Vehicles with Wireless
            vehicle_ids = traci.vehicle.getIDList()

            if len(vehicle_ids) > 0:
                n_not_0 = True

            if len(vehicle_ids) > max_n:
                max_n = len(vehicle_ids)

            if len(vehicle_ids) == 0 and n_not_0:
                break
        print(f"Simulation #{self.sim_id} terminated!")#, scores_per_cv2x.items())
        traci.close()
        return max_n


if __name__ == '__main__':
    hyper_params = {}
    # hyper_params['scenario_path'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_test/test.net.xml"
    # hyper_params['scenario_map'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_test/net.sumo.cfg"
    # hyper_params['scenario_polys'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_test/map.poly.xml"
    hyper_params['scenario_path'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_4/0/test.net.xml"
    hyper_params['scenario_map'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_4/0/net.sumo.cfg"
    hyper_params['scenario_polys'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_4/0/map.poly.xml"
    hyper_params["cv2x_N"] = 0.25
    hyper_params["fov"] = 120
    hyper_params["view_range"] = 75
    # hyper_params["base_station_position"] = 1600, 600
    hyper_params["base_station_position"] = (650, 500)
    hyper_params["num_RBs"] = 30
    hyper_params['message_size'] = 2000*8
    hyper_params['tot_num_vehicles'] = 150
    hyper_params['time_threshold'] = 10
########################################################################################################################
    path = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/"
    maps = os.listdir(path)
    maps = [path + map for map in maps]
    maps.sort()
    min_n = 0

    for map in maps:
        maps = os.listdir(path)
        maps = [path + map for map in maps]
        maps.sort(key=lambda x: int(x.split('/')[-1]))

    hyper_params['scenario_path'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_9/0/test.net.xml"
    hyper_params['scenario_map'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_9/0/net.sumo.cfg"
    hyper_params['scenario_polys'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_9/0/map.poly.xml"
    hyper_params["cv2x_N"] = 0.25
    hyper_params["fov"] = 120
    hyper_params["view_range"] = 75
    # hyper_params["base_station_position"] = 1600, 600
    hyper_params["base_station_position"] = (650, 500)
    hyper_params["num_RBs"] = 20
    hyper_params['message_size'] = 2000*8
    hyper_params['tot_num_vehicles'] = 150
    hyper_params['time_threshold'] = 10

    sim = Simulation(hyper_params, "4_0")
    sim.run()
