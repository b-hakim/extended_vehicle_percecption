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

    def get_seen_vehicles(self, cv2x_vehicles, non_cv2x_vehicles, buildings):
        """
        1- Use angle of view to get all vehicles in range
        2- Use vector from each cv2x_vehicle to all non-cv2x-vehicles
        3- if this vector passes through building or through 60% of the mid-vehicle or the distance is > 70m
            then skip this non-cv2x
        4- else, add this non-cv2x to the seen vehicle of this specific cv2x-vehicle
        """
        cv2x_vehicles_perception = {}

        for cv2x_vehicle in cv2x_vehicles:
            for non_cv2x_vehicle in non_cv2x_vehicles:
                if cv2x_vehicle.can_see_vehicle(non_cv2x_vehicle):
                    is_occluded = False

                    for building in buildings:
                        if cv2x_vehicle.building_in_sight(building.shape, non_cv2x_vehicle):
                            is_occluded = True
                            break

                    if not is_occluded:
                        for non_cv2x_obstacle_vehicle in non_cv2x_vehicles+cv2x_vehicles:
                            if non_cv2x_obstacle_vehicle.vehicle_id != non_cv2x_vehicle.vehicle_id and \
                                non_cv2x_obstacle_vehicle.vehicle_id != cv2x_vehicle.vehicle_id:
                                if cv2x_vehicle.vehicle_in_sight(non_cv2x_obstacle_vehicle, non_cv2x_vehicle):
                                    is_occluded = True
                                    break

                    if not is_occluded:
                        if cv2x_vehicle.vehicle_id in cv2x_vehicles_perception:
                            cv2x_vehicles_perception[cv2x_vehicle.vehicle_id].append(non_cv2x_vehicle.vehicle_id)
                        else:
                            cv2x_vehicles_perception[cv2x_vehicle.vehicle_id] = [non_cv2x_vehicle.vehicle_id]

        return cv2x_vehicles_perception

    def get_interest_cv2x_in_vehicle(self, cv2x_vehicle:Vehicle, non_cv2x_vehicle:Vehicle, p:float, time_threshold=10):
        # Traj = the trajectory of the cv2x vehicle.
        # Assume max velocity of cv2x = 40 m/s. Therefore, Traj for 5 sec has a max of 200m travel distance
        # Get trajectory from="noncv2x_vehicle" to differnet segment of "Traj".
        # Let noncv2x_vehicle speed = 40 m/s,
        #           calculate the time taken to reach different segemnts of "Traj"
        #           ignore any time above 5 sec.
        #           minT = minimum time taken for the noncv2x to reach one of the Traj segment + cv2x to reach that pt
        #           p = probability the cv2x seeing the non_cv2x
        #           interest = (1/minT) * (p)
        cv2x_future_edges = cv2x_vehicle.get_future_route(self.net, time_threshold)
        # print(cv2x_vehicle.vehicle_id,non_cv2x_vehicle.vehicle_id)

        min_dist, min_dist_edge = self.get_shortest_route(cv2x_vehicle.pos, non_cv2x_vehicle.pos,
                                                          non_cv2x_vehicle.get_current_road(), cv2x_future_edges)
        # min_dist, min_dist_edge = self.get_shortest_route(cv2x_vehicle.center_pos, non_cv2x_vehicle.center_pos,
        #                                                   non_cv2x_vehicle.get_current_road(), cv2x_future_edges)

        # no edge
        if min_dist_edge is None:
            return 0

        t = 3600 * ((min_dist+0.0000001)/40000) # get time in seconds for a speed of 40km/h

        if t > time_threshold:
            return 0
        else:
            return p/t

    def calculate_scores_per_cv2x(self, cv2x_perceived_non_cv2x_vehicles,
                                  cv2x_vehicles, non_cv2x_vehicles,
                                  buildings, time_threshold):
        cv2x_ids = list(cv2x_vehicles.keys())
        scores_per_cv2x = {}

        for sender_cv2x_id, perceived_non_cv2x_ids in cv2x_perceived_non_cv2x_vehicles.items():
            sender_cv2x_vehicle = cv2x_vehicles[sender_cv2x_id]
            other_cv2x_ids = list(set(cv2x_ids) - set([sender_cv2x_id]))
            scores = []

            for receiver_cv2x_id in other_cv2x_ids:
                for perceived_non_cv2x_id in perceived_non_cv2x_ids:
                    remaining_perceived_non_cv2x_ids = list(set(perceived_non_cv2x_ids)-set([perceived_non_cv2x_id]))
                    remaining_perceived_non_cv2x_vehicles = [non_cv2x_vehicles[id] for id in remaining_perceived_non_cv2x_ids]

                    receiver_cv2x_vehicle = cv2x_vehicles[receiver_cv2x_id]
                    perceived_non_cv2x_vehicle = non_cv2x_vehicles[perceived_non_cv2x_id]

                    p = sender_cv2x_vehicle.get_probability_cv2x_sees_non_cv2x(receiver_cv2x_vehicle, perceived_non_cv2x_vehicle,
                                                                                remaining_perceived_non_cv2x_vehicles, buildings)
                    if p == 0:
                        scores.append((receiver_cv2x_id,0))
                    else:
                        scores.append((receiver_cv2x_id,
                                       self.get_interest_cv2x_in_vehicle(receiver_cv2x_vehicle, perceived_non_cv2x_vehicle, p, time_threshold)))

            scores_per_cv2x[sender_cv2x_id] = scores
        return scores_per_cv2x

    def get_shortest_route(self, cv2x_veh_pos, noncv2x_veh_pos, non_cv2x_edge, list_destination_edges):
        min_distance = 100000000000
        min_dist_destination = None

        for destination_edge in list_destination_edges:
            route = traci.simulation.findRoute(non_cv2x_edge, destination_edge)

            if len(route.edges) == 0:
                continue

            cv2x_dist_to_segment = traci.simulation.findRoute(list_destination_edges[0], destination_edge).length

            if list_destination_edges[0][0] != ":":
                first_edge = self.net.getEdge(list_destination_edges[0])
                if not in_and_near_edge(cv2x_veh_pos, first_edge.getShape()):
                    # this means there is an error in the start position of the edge,
                    # so the new length is the edge length + veh_pos to start
                    # cv2x_dist_to_segment += euclidean_distance(cv2x_veh_pos, first_edge.getShape()[0])

                    # Edit: do nothing and just assume the vehicle is at the begining
                    pass
                else:
                    cv2x_dist_to_segment -= first_edge._length
                    cv2x_dist_to_segment += get_dist_from_to(cv2x_veh_pos, first_edge.getShape()[-1], first_edge.getShape())

            route_length = route.length + cv2x_dist_to_segment

            # if the destination is the first edge
            # remove the length for the last edge and add length between first point of last edge and cv2x vehicle pos
            if destination_edge[0] != ":":
                last_edge = self.net.getEdge(route.edges[-1])

                if in_and_near_edge(cv2x_veh_pos, last_edge.getShape()):
                    if destination_edge == list_destination_edges[0]:
                        route_length = route_length - last_edge._length

                        if route.edges == 1:
                            assert in_and_near_edge(noncv2x_veh_pos, last_edge.getShape())
                            route_length += get_dist_from_to(noncv2x_veh_pos, cv2x_veh_pos, last_edge.getShape())
                        else:
                            # need to make sure that the [0] is the entrance of this road
                            route_length += get_dist_from_to(last_edge.getShape()[0], cv2x_veh_pos, last_edge.getShape())
                            # route_length += euclidean_distance(last_edge.getShape()[0], cv2x_veh_pos)

            # remove the length for the first edge and add from src_vehicle position to the end of the first edge
            if non_cv2x_edge[0] != ":":
                first_edge = self.net.getEdge(route.edges[0])

                if in_and_near_edge(noncv2x_veh_pos, first_edge.getShape()):
                    route_length = route_length - first_edge._length

                    if route.edges == 1:
                        assert in_and_near_edge(cv2x_veh_pos, first_edge.getShape())
                        route_length += get_dist_from_to(noncv2x_veh_pos, cv2x_veh_pos, first_edge.getShape())
                        # route_length += euclidean_distance(noncv2x_veh_pos, cv2x_veh_pos)
                    else:
                        route_length += get_dist_from_to(noncv2x_veh_pos, first_edge.getShape()[-1], first_edge.getShape())
                        # route_length += euclidean_distance(noncv2x_veh_pos, first_edge.getShape()[1])

            if route_length < min_distance:
                min_distance = route_length
                min_dist_destination = destination_edge

            # route_distance = 0
            # prev_edge_points = None
            #
            # for edge_id in route.edges:
            #     if edge_id[0] == ":":# or edge_id == route.edges[-1]:
            #         continue
            #
            #     curr_edge_points = self.net.getEdge(edge_id).getShape()
            #
            #     for i in range(1, len(curr_edge_points)):
            #         d = euclidean_distance(curr_edge_points[i], curr_edge_points[i-1])
            #         route_distance += d

                # if prev_edge_points is not None:
                #     dist = distance_prev_curr_edge(prev_edge_points, curr_edge_points)
                #     route_distance += dist

                # prev_edge_points = curr_edge_points

            # x=1
        # return route_distance
        return min_distance, min_dist_destination

    def make_unique_requests(self, score_per_cv2x):
        receivers = {}

        for sender_cv2x_id, receiver_score in score_per_cv2x.items():
            receiver_cv2x, score = receiver_score

            if receiver_cv2x in receivers:
                if receivers[receiver_cv2x] < score:
                    receivers[receiver_cv2x] = score
            else:
                receivers[receiver_cv2x] = score

        return receivers

    def get_channel_gain_vehicle(self, tx_pos, receiver_vehicles, K):
        h_n = {}
        # wavelength = 5.5*10**-7
        # d0_squared = 10000
        # wavelength_squared = 3.025e-13

        y = 4

        for vehicle in receiver_vehicles:
            distance = euclidean_distance(tx_pos, vehicle.pos)
            # h_n_k = (distance**-4)*random.random()*np.random.exponential(1)
            tmp = []
            # this approach is canceled as it is more accurate and detailed but related to the exact wavelengths
            # changed by approach 2 (as sent in the email)
            # d0 = 100
            # L = 20 * np.log10(wavelength / (4 * np.pi * d0))

            for i in range(K):
                # r = 10 + random.random() * 90 # random number between 10 to 100
                # channel_rand = random.randint(80, 120)/100
                # h_n_k = channel_rand * L * (d0 / distance) ** y
                h_n_k = np.random.exponential(1)/distance**y
                # h_n_k = 1
                tmp.append(h_n_k)

            h_n[vehicle.vehicle_id] = (tmp)

        return h_n

    def run(self, repeat_experiment=False):
        sumoBinary = "/usr/bin/sumo"
        # sumoBinary = "/usr/bin/sumo-gui"
        sumoCmd = [sumoBinary, "-c", self.hyper_params['scenario_map'], '--no-warnings']
        traci.start(sumoCmd)
        step = 0
        viz = SumoVisualizer(self.hyper_params)

        repeat_path = os.path.join(os.path.dirname(self.hyper_params['scenario_path']), "repeat.txt")
        ########################################## Load Rand State ################################################
        if repeat_experiment:
            with open(repeat_path) as fr:
                np_rand_state = (fr.readline().strip(),np.array(literal_eval(fr.readline().strip())), int(fr.readline().strip()),
                                 int(fr.readline().strip()), float(fr.readline().strip()))
                py_rand_state = literal_eval(fr.readline())

            np.random.set_state(np_rand_state)
            random.setstate(py_rand_state)
        #######################################################################################################
        # Save Rand State
        with open(repeat_path, 'w') as fw:
            fw.write(str(np.random.get_state()[0]).replace("\n", "")+"\n")
            fw.write(str(np.random.get_state()[1].tolist()).replace("\n", "")+"\n")
            fw.write(str(np.random.get_state()[2]).replace("\n", "")+"\n")
            fw.write(str(np.random.get_state()[3]).replace("\n", "")+"\n")
            fw.write(str(np.random.get_state()[4]).replace("\n", "")+"\n")
            fw.write(str(random.getstate())+"\n")

        vehicles = {}
        buildings = sumolib.shapes.polygon.read(self.hyper_params['scenario_polys'])
        tmp = []

        for building in buildings:
            if building.type != "unknown":
                tmp.append(building)

        buildings = tmp

        while step < 100000:
            step += 1
            # print("Step:", step)
            # if traci.inductionloop.getLastStepVehicleNumber("1") > 0:
            #     traci.trafficlight.setRedYellowGreenState("0", "GrGr")

            traci.simulationStep()
            traci.route.getIDList()

            # 1) Get All Vehicles with Wireless
            vehicle_ids = traci.vehicle.getIDList()
            # print(vehicle_list)

            for vid in vehicle_ids:
                v_found = False

                for vehicle_id, vehicle in vehicles.items():
                    if vehicle_id == vid:
                        v_road_id = traci.vehicle.getRoadID(vid)
                        vehicle.update_latest_edge_road(v_road_id)
                        v_found = True

                if not v_found:
                    vehicles[vid] = Vehicle(vehicle_id=vid, hyper_params=self.hyper_params)

            need_to_remove = set(vehicles.keys()) - set(vehicle_ids)

            for vid in need_to_remove:
                vehicles.pop(vid)

            # if len(vehicle_ids) == 0:
            #     break
            # else:
            snapshot = False
            if len(vehicle_ids) >= self.hyper_params['tot_num_vehicles']:
                # if np.random.random() > 0.5:
                snapshot = True

            # print(f"Step={step}, N={len(vehicle_ids)}")
            # print("=============================================================")

            if not snapshot:
                continue

            # print(vehicles.keys())
            viz.draw_vehicles(vehicles.values())

            cv2x_len = int(self.hyper_params["cv2x_N"] * len(vehicle_ids))
            cv2x_vehicles_indexes = random.sample(range(len(vehicle_ids)), cv2x_len)
            non_cv2x_vehicles_indexes = list(set(range(len(vehicle_ids))) - set(cv2x_vehicles_indexes))
            # Transform indexes into IDs and vehicles
            cv2x_vehicles = {vehicle_ids[index]:vehicles[vehicle_ids[index]] for index in cv2x_vehicles_indexes}
            non_cv2x_vehicles = {vehicle_ids[index]:vehicles[vehicle_ids[index]] for index in non_cv2x_vehicles_indexes}

            show_id = None

            for cv2x_id, vehicle in cv2x_vehicles.items():
                if cv2x_id != show_id and show_id is not None:
                    continue

                viz.draw_vehicle_perception(vehicle, (185, 218, 255))

            # 2) Get seen non-cv2x vehicles by each cv2x_vehicle
            cv2x_perceived_non_cv2x_vehicles = self.get_seen_vehicles(list(cv2x_vehicles.values()), list(non_cv2x_vehicles.values()), buildings)
            tot_perceived_objects = 0

            for cv2x_id, non_cv2x_ids in cv2x_perceived_non_cv2x_vehicles.items():
                if cv2x_id != show_id and show_id is not None:
                    continue

                cv2x_non_vehs = [vehicles[id] for id in non_cv2x_ids]

                for vehicle in cv2x_non_vehs:
                    viz.draw_vehicle_body(vehicle, color=(0, 0, 128))

                tot_perceived_objects += len(non_cv2x_ids)

            save_path = os.path.join(os.path.dirname(self.hyper_params['scenario_path']),
                                     "map_" + str(self.hyper_params['cv2x_N'])
                                     + "_" + str(self.hyper_params['fov'])
                                     + "_" + str(self.hyper_params["view_range"])
                                     + "_" + str(self.hyper_params["num_RBs"])
                                     + "_" + str(self.hyper_params["tot_num_vehicles"])
                                     + "_" + str(self.hyper_params['time_threshold'])
                                     + ".png")
            viz.save_img(save_path)

            # 3) Solve which info to send to base station
            # 3.1) Calculate required information
            scores_per_cv2x = self.calculate_scores_per_cv2x(cv2x_perceived_non_cv2x_vehicles, cv2x_vehicles, non_cv2x_vehicles,
                                                             buildings, self.hyper_params['time_threshold'])

            # 3.2) Get Highest Score for each cv2x vehicle
            score_per_cv2x = {cv2x: max(scores, key=lambda x: x[1]) for cv2x, scores in scores_per_cv2x.items()}

            # 3.3) Prevent Vehicles from sending with score = 0
            score_per_cv2x = {cv2x:score_receiver for cv2x, score_receiver in score_per_cv2x.items() if score_receiver[1] !=687}

            total_requests_duplicated = len(score_per_cv2x)

            # 3.4) Make unique
            score_per_cv2x = self.make_unique_requests(score_per_cv2x)

            h_n_k = self.get_channel_gain_vehicle(self.hyper_params["base_station_position"],
                                                  [vehicle for vehicle in vehicles.values() if
                                                   vehicle.vehicle_id in score_per_cv2x],
                                                  self.hyper_params['num_RBs'])

            # 3.3) Baseline to solve the problem and select which requests to send
            solver = Solver(score_per_cv2x, h_n_k,
                            self.hyper_params['num_RBs'], self.hyper_params['message_size'])

            save_path = os.path.join(os.path.dirname(self.hyper_params['scenario_path']),
                                     "results_" + str(self.hyper_params['cv2x_N'])
                                     + "_" + str(self.hyper_params['fov'])
                                     + "_" + str(self.hyper_params["view_range"])
                                     + "_" + str(self.hyper_params["num_RBs"])
                                     + "_" + str(self.hyper_params["tot_num_vehicles"])
                                     + "_" + str(self.hyper_params['time_threshold'])
                                     +".txt")

            sent, unsent = solver.find_optimal_assignment(save_path)

            with open(save_path, 'a') as fw:
                fw.write("Total Duplicate Requests: " + str(total_requests_duplicated)
                         + "\nTotal Unique Requests: " + str(sent+unsent)
                         + "\nSent: " + str(sent)
                         + "\nUnsent: " + str(unsent)
                         + "\nPerceived Vehicles: " + str(tot_perceived_objects))

            if snapshot:
                break

        viz.save_img()
        # input("Simulation ended, close GUI?")
        print(f"Simulation #{self.sim_id} terminated!")#, scores_per_cv2x.items())
        traci.close()


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
    hyper_params['scenario_path'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_0/0/test.net.xml"
    hyper_params['scenario_map'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_0/0/net.sumo.cfg"
    hyper_params['scenario_polys'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_0/0/map.poly.xml"
    hyper_params["cv2x_N"] = 0.25
    hyper_params["fov"] = 120
    hyper_params["view_range"] = 75
    # hyper_params["base_station_position"] = 1600, 600
    hyper_params["base_station_position"] = (650, 500)
    hyper_params["num_RBs"] = 50
    hyper_params['message_size'] = 2000*8
    hyper_params['tot_num_vehicles'] = 100
    hyper_params['time_threshold'] = 10

    sim = Simulation(hyper_params, "0_0")
    sim.run()
