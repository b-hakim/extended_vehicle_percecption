import os, sys
import pickle
import random
import time
from ast import literal_eval
from typing import List, Dict

import numpy as np
import sumolib
import traci

from math_utils import euclidean_distance, in_and_near_edge, get_dist_from_to, in_segment, move_point, get_new_abs_pos
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
        cv2x_vehicles_perception_visible = {}

        for cv2x_vehicle in cv2x_vehicles:
            for non_cv2x_vehicle in non_cv2x_vehicles:
                if cv2x_vehicle.has_in_perception_range(non_cv2x_vehicle, False, False, detection_probability=1):
                    is_occluded = False

                    for building in buildings:
                        if cv2x_vehicle.building_in_sight(building.shape, False, non_cv2x_vehicle):
                            is_occluded = True
                            break

                    if not is_occluded:
                        for obstacle_vehicle in non_cv2x_vehicles+cv2x_vehicles:
                            if obstacle_vehicle.vehicle_id != non_cv2x_vehicle.vehicle_id and \
                                    obstacle_vehicle.vehicle_id != cv2x_vehicle.vehicle_id:
                                if cv2x_vehicle.vehicle_in_sight(obstacle_vehicle, non_cv2x_vehicle, False):
                                    is_occluded = True
                                    break

                    if not is_occluded:
                        if cv2x_vehicle.has_in_perception_range(non_cv2x_vehicle, False, False, self.hyper_params["perception_probability"]):
                            if cv2x_vehicle.vehicle_id in cv2x_vehicles_perception:
                                cv2x_vehicles_perception[cv2x_vehicle.vehicle_id].append(non_cv2x_vehicle.vehicle_id)
                            else:
                                cv2x_vehicles_perception[cv2x_vehicle.vehicle_id] = [non_cv2x_vehicle.vehicle_id]

                        if cv2x_vehicle.vehicle_id in cv2x_vehicles_perception_visible:
                            cv2x_vehicles_perception_visible[cv2x_vehicle.vehicle_id].append(non_cv2x_vehicle.vehicle_id)
                        else:
                            cv2x_vehicles_perception_visible[cv2x_vehicle.vehicle_id] = [non_cv2x_vehicle.vehicle_id]

            # Code for perceived cv2x using communication sensors
            # Decision: Do not add AV to the perceived set to differentiate between AV and NAV,
            # Also, we do check the perceived vehicles with all the possible receivers later in the code
            #
            # for other_cv2x in cv2x_vehicles:
            #     if other_cv2x.vehicle_id == cv2x_vehicle.vehicle_id:
            #         continue
            #     if cv2x_vehicle.has_in_perception_range(other_cv2x, True, detection_probability=1):
            #         if cv2x_vehicle.vehicle_id in cv2x_vehicles_perception:
            #             cv2x_vehicles_perception[cv2x_vehicle.vehicle_id].append(other_cv2x.vehicle_id)
            #         else:
            #             cv2x_vehicles_perception[cv2x_vehicle.vehicle_id] = [other_cv2x.vehicle_id]
            #
            #         if cv2x_vehicle.vehicle_id in cv2x_vehicles_perception_visible:
            #             cv2x_vehicles_perception_visible[cv2x_vehicle.vehicle_id].append(
            #                 other_cv2x.vehicle_id)
            #         else:
            #             cv2x_vehicles_perception_visible[cv2x_vehicle.vehicle_id] = [other_cv2x.vehicle_id]

        return cv2x_vehicles_perception, cv2x_vehicles_perception_visible

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

        min_dist, min_dist_edge = self.get_shortest_route(cv2x_vehicle.get_pos(), non_cv2x_vehicle.get_pos(),
                                                          non_cv2x_vehicle.get_current_road(), cv2x_future_edges)
        # min_dist, min_dist_edge = self.get_shortest_route(cv2x_vehicle.center_pos, non_cv2x_vehicle.center_pos,
        #                                                   non_cv2x_vehicle.get_current_road(), cv2x_future_edges)

        # no edge
        if min_dist_edge is None:
            return 0

        t = (min_dist+0.0000001)/self.hyper_params["avg_speed_sec"] # get time in seconds for a speed of 40km/h

        if t > time_threshold:
            return 0
        else:
            return p/t

    def calculate_scores_per_cv2x(self, av_perceiving_nav_vehicles,
                                  av, non_av,
                                  buildings, time_threshold):
        correct_los, correct_nlos, unsure_los, unsure_nlos, incorrect_los, incorrect_nlos = 0, 0, 0, 0, 0, 0

        av_ids = list(av.keys())
        scores_per_av = {}

        for sender_av_id, perceived_nav_ids in av_perceiving_nav_vehicles.items():
            sender_av = av[sender_av_id]
            other_av_ids = list(set(av_ids) - set([sender_av_id]))
            scores = []
            perceived_av = []

            for perceived_av_id in other_av_ids:
                # This is considered as ignoring the sender camera and using location from the receiver GNSS location
                # The receiver GNSS error is done inside the receiver get_pos
                # The sender GNSS error affects the calculation of perceiving the receiver AV --> hence, param: True
                if sender_av.has_in_perception_range(av[perceived_av_id], True, True,
                                                               self.hyper_params["perception_probability"]):
                    perceived_av.append(av[perceived_av_id])

            for receiver_av_id in other_av_ids:
                receiver_is_in_sender_perception_area = False

                if av[receiver_av_id] in perceived_av:
                    perceived_av.remove(av[receiver_av_id])
                    receiver_is_in_sender_perception_area = True

                for perceived_nav_id in perceived_nav_ids:

                    remaining_perceived_non_cv2x_ids = list(set(perceived_nav_ids)-set([perceived_nav_id]))
                    remaining_perceived_nav = [non_av[id] for id in remaining_perceived_non_cv2x_ids]

                    receiver_av = av[receiver_av_id]
                    perceived_nav = non_av[perceived_nav_id]

                    p = sender_av.calculate_probability_av_sees_nav(receiver_av,
                                                                    perceived_nav,
                                                                    remaining_perceived_nav + perceived_av,
                                                                    buildings,
                                                                    self.hyper_params["perception_probability"],
                                                                    self.hyper_params["continous_probability"])

                    if p == 1:
                        # if sender sees that LOS between receiver and perceived_obj and is LOS then correct LOS ++
                        # if sender sees that LOS between receiver and perceived_obj and is NLOS then incorrect LOS ++
                        if receiver_av_id in av_perceiving_nav_vehicles:
                            if perceived_nav_id in av_perceiving_nav_vehicles[receiver_av_id]:
                                correct_los += 1
                            else:
                                incorrect_los += 1
                        else:
                            incorrect_los += 1
                    elif p == 0:
                        # if sender sees that NLOS between receiver and perceived_obj is NLOS then correct NLOS ++
                        # if sender sees that NLOS between receiver and perceived_obj is LOS then incorrect NLOS ++
                        if receiver_av_id in av_perceiving_nav_vehicles:
                            if perceived_nav_id in av_perceiving_nav_vehicles[receiver_av_id]:
                                incorrect_nlos += 1
                            else:
                                correct_nlos += 1
                        else:
                            correct_nlos += 1
                    else:
                        if receiver_av_id in av_perceiving_nav_vehicles:
                            if perceived_nav_id in av_perceiving_nav_vehicles[receiver_av_id]:
                                unsure_los += 1
                            else:
                                unsure_nlos += 1
                        else:
                            unsure_nlos += 1

                    if self.hyper_params["estimate_detection_error"]:
                        if p == 1: # cv2x receiver sees object to be sent, then add probability it doesnt sees it!
                            if random.random() > self.hyper_params["perception_probability"]: # probably receiver doesnt see it!
                                p = 0.5

                    p = 1 - p # set to 0 if cv2x sees the object to be sent

                    if p == 0:
                        scores.append((receiver_av_id, 0, perceived_nav, p))
                    else:
                        v = self.get_interest_cv2x_in_vehicle(receiver_av,
                                                          perceived_nav, p, time_threshold)
                        scores.append((receiver_av_id, v, perceived_nav, p))

                if receiver_is_in_sender_perception_area:
                    perceived_av.append(av[receiver_av_id])

            scores_per_av[sender_av_id] = scores
        return scores_per_av, [correct_los, correct_nlos, incorrect_los, incorrect_nlos, unsure_los, unsure_nlos]

    def get_shortest_route(self, cv2x_veh_pos, noncv2x_veh_pos, non_cv2x_edge, list_destination_edges):
        min_distance = 100000000000
        min_dist_destination = None

        for destination_edge in list_destination_edges:
            route = traci.simulation.findRoute(non_cv2x_edge, destination_edge)

            if len(route.edges) == 0:
                continue

            cv2x_dist_to_segment = traci.simulation.findRoute(list_destination_edges[0], destination_edge).length

            if list_destination_edges[0][0] == ":":
                if cv2x_dist_to_segment == 0:
                    first_edge = self.net.getEdge(list_destination_edges[1])
                    dist_closer_point = max(euclidean_distance(cv2x_veh_pos, first_edge.getShape()[0]),
                                                            euclidean_distance(cv2x_veh_pos, first_edge.getShape()[1]))
                    cv2x_dist_to_segment += dist_closer_point
            else:
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
            receiver_cv2x, score, perceived_object, p = receiver_score

            if receiver_cv2x in receivers:
                if receivers[receiver_cv2x][1] < score:
                    receivers[receiver_cv2x] = [sender_cv2x_id, score, perceived_object, p]
            else:
                receivers[receiver_cv2x] = [sender_cv2x_id, score, perceived_object, p]

        return receivers

    def get_channel_gain_vehicle(self, tx_pos, receiver_vehicles, K):
        h_n = {}
        # wavelength = 5.5*10**-7
        # d0_squared = 10000
        # wavelength_squared = 3.025e-13

        y = 4

        for vehicle in receiver_vehicles:
            distance = euclidean_distance(tx_pos, vehicle._pos)
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

    def run(self):

        path = os.path.dirname(self.hyper_params['scenario_path'])
        state_path = os.path.join(path, "saved_state",
                                  "state_" + str(self.hyper_params['cv2x_N'])
                                        + "_" + str(self.hyper_params['fov'])
                                        + "_" + str(self.hyper_params["view_range"])
                                        + "_" + str(self.hyper_params["num_RBs"])
                                        + "_" + str(self.hyper_params["tot_num_vehicles"])
                                        + "_" + str(self.hyper_params['time_threshold'])
                                        + "_" + str(self.hyper_params['perception_probability'])
                                        + ("_ede" if self.hyper_params["estimate_detection_error"] else "_nede")
                                        + "_" + str(self.hyper_params["noise_distance"])
                                        + ("_egps" if self.hyper_params["noise_distance"] else "")
                                        + ("_cont_prob" if self.hyper_params["continous_probability"] else "_discont_prob")
                                  + ".pkl")

        with open(state_path, 'rb') as fr:
            cv2x_vehicles, non_cv2x_vehicles, buildings, cv2x_perceived_non_cv2x_vehicles,\
            scores_per_cv2x, los_statuses, vehicles, cv2x_perceived_non_cv2x_vehicles,\
            cv2x_vehicles_perception_visible, tot_perceived_objects, tot_visible_objects = pickle.load(fr)

        # Count number of perceived
        perceived_but_considered_to_send = 0
        not_perceived_but_visible_considered_to_send = 0

        for sender_cv2x_id, scores in scores_per_cv2x.items():
            for (receiver_id, score, perceived_non_cv2x_vehicle, p) in scores: #
                perceived = False
                if score != 0:
                    if receiver_id in cv2x_perceived_non_cv2x_vehicles:
                        # Sender does not believed the receiver sees the object cause of the score
                        # However, it is seen by the receiver
                        perceived_non_cv2x_vehicles = cv2x_perceived_non_cv2x_vehicles[receiver_id]

                        if perceived_non_cv2x_vehicle.vehicle_id in perceived_non_cv2x_vehicles:
                            perceived_but_considered_to_send += 1
                            perceived = True

                    if receiver_id in cv2x_vehicles_perception_visible and not perceived:
                        visible_non_cv2x_vehicles = cv2x_vehicles_perception_visible[receiver_id]

                        if perceived_non_cv2x_vehicle.vehicle_id in visible_non_cv2x_vehicles:
                            # Sender does not believed the receiver sees the objectit
                            # It is not perceived by the receiver
                            # However, it is visible to the receiver --> due to detection or gps error!
                            not_perceived_but_visible_considered_to_send += 1

        # 3.2) Get Highest Score for each cv2x vehicle
        s_processing_t = time.time()
        score_per_cv2x = {cv2x: max(scores, key=lambda x: x[1]) for cv2x, scores in scores_per_cv2x.items()}
        e_processing_t = time.time()

        perceived_but_sent_to_BS = 0
        not_perceived_but_visible_sent_to_BS = 0

        for sender_cv2x_id, scores in score_per_cv2x.items():
            (receiver_id, score, perceived_non_cv2x_vehicle, p) = scores
            perceived = False

            if score != 0:
                if receiver_id in cv2x_perceived_non_cv2x_vehicles:
                    # Sender does not believed the receiver sees the object cause of the score
                    # However, it is seen by the receiver
                    perceived_non_cv2x_vehicles = cv2x_perceived_non_cv2x_vehicles[receiver_id]

                    if perceived_non_cv2x_vehicle.vehicle_id in perceived_non_cv2x_vehicles:
                        perceived_but_sent_to_BS += 1
                        perceived = True

                if receiver_id in cv2x_vehicles_perception_visible and not perceived:
                    visible_non_cv2x_vehicles = cv2x_vehicles_perception_visible[receiver_id]

                    if perceived_non_cv2x_vehicle.vehicle_id in visible_non_cv2x_vehicles:
                        # Sender does not believed the receiver sees the objectit
                        # It is not perceived by the receiver
                        # However, it is visible to the receiver --> due to detection or gps error!
                        not_perceived_but_visible_sent_to_BS += 1

        tot_time = e_processing_t - s_processing_t

        # 3.3) Prevent Vehicles from sending with score = 0
        s_processing_t = time.time()
        score_per_cv2x = {cv2x:score_receiver for cv2x, score_receiver in score_per_cv2x.items() if score_receiver[1] != 0}

        total_requests_duplicated = len(score_per_cv2x)

        # 3.4) Make unique
        score_per_cv2x = self.make_unique_requests(score_per_cv2x)

        h_n_k = self.get_channel_gain_vehicle(self.hyper_params["base_station_position"],
                                              [vehicle for vehicle in vehicles.values() if
                                               vehicle.vehicle_id in score_per_cv2x],
                                              self.hyper_params['num_RBs'])

        # 3.3) Baseline to solve the problem and select which requests to send
        s = time.time()
        solver = Solver(score_per_cv2x, h_n_k,
                        self.hyper_params['num_RBs'], self.hyper_params['message_size'])

        save_path = os.path.join(os.path.dirname(self.hyper_params['scenario_path']),
                                 "results_" + str(self.hyper_params['cv2x_N'])
                                 + "_" + str(self.hyper_params['fov'])
                                 + "_" + str(self.hyper_params["view_range"])
                                 + "_" + str(self.hyper_params["num_RBs"])
                                 + "_" + str(self.hyper_params["tot_num_vehicles"])
                                 + "_" + str(self.hyper_params['time_threshold'])
                                 + "_" + str(self.hyper_params['perception_probability'])
                                 + ("_ede" if self.hyper_params["estimate_detection_error"] else "_nede")
                                 + "_" + str(self.hyper_params["noise_distance"])
                                 + ("_egps" if self.hyper_params["noise_distance"] != 0 else "")
                                 + ("_cont_prob" if self.hyper_params["continous_probability"] else "_discont_prob")
                                 +".txt")

        sent, unsent, selected_messages_requests = solver.find_optimal_assignment(save_path)
        # selected_messages_requests >> [sender, receiver, score, non_cv2x_pos]
        e = time.time()
        e_processing_t = time.time()
        print("Optimization time", e-s)
        print("Preprocessing time + Optimization time", tot_time + e_processing_t - s_processing_t)

        if self.hyper_params["save_gnss"]:
            # Save location of objects and their IDs in a file
            # Save the sender and receiver and objects in another file
            with open(save_path.replace('results', 'GNSS').replace(".txt",'.pkl'), 'wb') as fw:
                pickle.dump(({v:vehicles[v].get_pos() for v in vehicles.keys()}, selected_messages_requests), fw)

        perceived_but_sent_by_BS = 0
        not_perceived_but_visible_sent_by_BS = 0

        for (msg_sender_id, [msg_receiver_id, m_s, msg_obj_pos]) in selected_messages_requests:
            msg_found = False

            for sender_cv2x_id, scores in scores_per_cv2x.items():
                if sender_cv2x_id != msg_sender_id:
                    continue

                for (receiver_id, score, perceived_non_cv2x_vehicle, p) in scores:
                    pos = perceived_non_cv2x_vehicle._pos
                    if receiver_id != msg_receiver_id or \
                            (pos[0] - msg_obj_pos[0] > 0.0001 and pos[1] - msg_obj_pos[1] > 0.0001):
                        continue

                    perceived = False

                    if receiver_id in cv2x_perceived_non_cv2x_vehicles:
                        # Sender does not believed the receiver sees the object cause of the score
                        # However, it is seen by the receiver
                        perceived_non_cv2x_vehicles = cv2x_perceived_non_cv2x_vehicles[receiver_id]

                        if perceived_non_cv2x_vehicle.vehicle_id in perceived_non_cv2x_vehicles:
                            perceived_but_sent_by_BS += 1
                            perceived = True

                    if receiver_id in cv2x_vehicles_perception_visible and not perceived:

                        visible_non_cv2x_vehicles = cv2x_vehicles_perception_visible[receiver_id]

                        if perceived_non_cv2x_vehicle.vehicle_id in visible_non_cv2x_vehicles:
                            # Sender does not believed the receiver sees the objectit
                            # It is not perceived by the receiver
                            # However, it is visible to the receiver --> due to detection or gps error!
                            not_perceived_but_visible_sent_by_BS += 1

                    msg_found = True
                    break

                if msg_found:
                    break

        with open(save_path, 'a') as fw:
            fw.write("GNSS Errors:\n")

            for msg in selected_messages_requests:
                sender_id, (receiver_id, score, obj_pos) = msg
                #To do:
                # for each msg,
                # 1- Calculate the relative position of the object ==> rel_obj_pos and add noise for depth error
                # 2- Change the position of the sender based on error ==> sender_noisy_pos
                # 3- Calculate the new position of the object ==> abs_obj_noisy_pos
                # 4- Calculate the error between rel_obj_po and rel_obj_noisy_pos
                # 5- Add noise for detection error
                sender_pos = np.array(vehicles[sender_id]._pos)
                sender_noisy_pos = move_point(sender_pos, random.randint(0, 360), self.hyper_params["noise_distance"])
                abs_obj_noisy_pos = get_new_abs_pos(sender_pos, sender_noisy_pos, obj_pos)

                error = np.linalg.norm([np.array(abs_obj_noisy_pos) - np.array(obj_pos)])
                fw.write(f"{sender_id}, {receiver_id}, {score}, {obj_pos}, {error}\n")

            fw.write("\n")

        with open(save_path, 'a') as fw:
            fw.write("AVs: " + str(len(cv2x_vehicles))
                     + "\nNon-AVs: " + str(len(non_cv2x_vehicles))
                     + "\nTotal Duplicate Requests: " + str(total_requests_duplicated)
                     + "\nTotal Unique Requests: " + str(sent+unsent)
                     + "\nSent: " + str(sent)
                     + "\nUnsent: " + str(unsent)
                     + "\nPerceived Vehicles: " + str(tot_perceived_objects)
                     + "\nVisible Vehicles: " + str(tot_visible_objects)
                     + "\nPerceived But considered sending to BS: " + str(perceived_but_considered_to_send)
                     + "\nPerceived But Sent to BS: " + str(perceived_but_sent_to_BS)
                     + "\nPerceived But Sent by BS: " + str(perceived_but_sent_by_BS)
                     + "\nNot Perceived but visible and considered sending to BS: " + str(not_perceived_but_visible_considered_to_send)
                     + "\nNot Perceived but visible and Sent to BS: " + str(not_perceived_but_visible_sent_to_BS)
                     + "\nNot Perceived but visible and Sent by BS: " + str(not_perceived_but_visible_sent_by_BS)
                     + "\nCorrect LoS: " + str(los_statuses[0])
                     + "\nCorrect Nlos: " + str(los_statuses[1])
                     + "\nIncorrect Los: " + str(los_statuses[2])
                     + "\nIncorrect NLoS: " + str(los_statuses[3])
                     + "\nUnsure LoS: " + str(los_statuses[4])
                     + "\nUnsure NLoS: " + str(los_statuses[5])
                     )
        # [correct_los, correct_nlos, incorrect_los, incorrect_nlos, unsure_los, unsure_nlos]
        # if self.hyper_params["save_visual"]:
        #     viz.save_img()



if __name__ == '__main__':
    hyper_params = {}
    # hyper_params['scenario_path'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_test/test.net.xml"
    # hyper_params['scenario_map'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_test/net.sumo.cfg"
    # hyper_params['scenario_polys'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_test/map.poly.xml"
    basedir = '/media/bassel/Career/toronto_content_selection/toronto_dense/toronto_2/5/'
    basedir = '/home/bassel/toronto_AVpercentage_RBs/toronto_7/1/'
    # basedir = '/media/bassel/Career/toronto_content_selection/toronto_dense/toronto_1/0/'
    hyper_params['scenario_path'] = basedir + "test.net.xml"
    hyper_params['scenario_map'] =  basedir + "net.sumo.cfg"
    hyper_params['scenario_polys'] = basedir + "map.poly.xml"
    hyper_params["cv2x_N"] = 0.65
    hyper_params["fov"] = 120
    hyper_params["view_range"] = 75
    # hyper_params["base_station_position"] = 1600, 600
    hyper_params["base_station_position"] = (2034, 1712)
    hyper_params["num_RBs"] = 100
    hyper_params['message_size'] = 2000*8
    hyper_params['tot_num_vehicles'] = 100
    hyper_params['time_threshold'] = 10
    hyper_params['save_visual'] = True
    hyper_params['noise_distance'] = 0
    hyper_params['perception_probability'] = 1
    hyper_params['estimate_detection_error'] = False
    hyper_params['save_gnss'] = False
    hyper_params['continous_probability'] = False
    hyper_params["avg_speed_sec"] = 10

    # while True:
    sim = Simulation(hyper_params, "1_0")
    sim.run()
