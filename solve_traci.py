import os, sys
import pickle
import random
import time
import numpy as np
import sumolib
from solver import Solver


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


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

