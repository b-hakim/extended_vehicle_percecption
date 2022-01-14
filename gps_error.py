import multiprocessing
import pickle
import os
import random
import time
import numpy as np


class RunGnssSimulationProcess(multiprocessing.Process):
    def __init__(self, map, cv2x_percentage, fov, view_range, num_RBs, tot_num_vehicles, id,
                 perception_probability=1.0, estimate_detection_error=False, noise_value=0):
        # multiprocessing.Process.__init__(self)
        super(RunGnssSimulationProcess, self).__init__()

        if type(map) != list:
            sub_dirs = os.listdir(map)
            sub_dirs.sort()
            self.traffics = [os.path.join(map, dirname) for dirname in sub_dirs]
        else:
            self.traffics = map

        self.cv2x_percentage, self.fov, self.view_range, \
        self.num_RBs, self.tot_num_vehicles, self.perception_probability = cv2x_percentage, fov, view_range, num_RBs,\
                                                                          tot_num_vehicles, perception_probability
        self.sim_id = id
        self.estimate_detection_error = estimate_detection_error
        self.noise_distance = noise_value

    @staticmethod
    def move_point(point, angle, distance):
        return [point[0] + np.sin(np.deg2rad(angle)*distance),
                point[1] + np.cos(np.deg2rad(angle) * distance)]

    def run(self):
        print(f"Simulation Thread {str(self.sim_id)} started")

        for traffic in self.traffics:  # toronto_'i'/'j'
            tot_time_start = time.time()
            hyper_params = {}
            hyper_params['scenario_path'] = os.path.join(traffic, "test.net.xml")
            hyper_params["cv2x_N"] = self.cv2x_percentage
            hyper_params["fov"] = self.fov
            hyper_params["view_range"] = self.view_range
            hyper_params["num_RBs"] = self.num_RBs
            hyper_params['tot_num_vehicles'] = self.tot_num_vehicles
            hyper_params['time_threshold'] = 10
            hyper_params['perception_probability'] = self.perception_probability
            hyper_params["estimate_detection_error"] = self.estimate_detection_error

            gnss_path = os.path.join(os.path.dirname(hyper_params['scenario_path']),
                                        "GNSS_" + str(hyper_params['cv2x_N'])
                                        + "_" + str(hyper_params['fov'])
                                        + "_" + str(hyper_params["view_range"])
                                        + "_" + str(hyper_params["num_RBs"])
                                        + "_" + str(hyper_params["tot_num_vehicles"])
                                        + "_" + str(hyper_params['time_threshold'])
                                        + "_" + str(hyper_params['perception_probability'])
                                        + ("_ede" if hyper_params["estimate_detection_error"] else "_nede")
                                        + ".pkl")

            with open(gnss_path, 'rb') as fw:
                (vehicles_id_pos_dict, selected_messages_requests) = pickle.load(fw)

            for msg in selected_messages_requests:
                sender_id, (receiver_id, _, obj_pos) = msg
                #To do:
                # for each msg,
                # 1- Calculate the relative position of the object ==> rel_obj_pos
                # 2- Change the position of the sender based on error ==> sender_noisy_pos
                # 3- Calculate the new position of the object ==> abs_obj_noisy_pos
                # 4- Calculate the error between rel_obj_po and rel_obj_noisy_pos
                # 5- Save it in file
                sender_pos = np.array(vehicles_id_pos_dict[sender_id])
                receiver_pos = np.array(vehicles_id_pos_dict[receiver_id])
                rel_obj_pos = receiver_pos - sender_pos
                sender_noisy_pos = RunGnssSimulationProcess.move_point(sender_pos, random.randint(0, 360), self.noise_distance)
                abs_obj_noisy_pos = rel_obj_pos + sender_noisy_pos
                error = np.linalg.norm(abs_obj_noisy_pos, obj_pos)

                # for each calc

            print(f"Simulation Thread {str(self.sim_id)} ended with {time.time()-tot_time_start} seconds")

if __name__ == '__main__':
    p = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_test"
    simulation_thread = RunGnssSimulationProcess([p], cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
                                                 tot_num_vehicles=100, id=0, perception_probability=1,
                                                 estimate_detection_error=False, noise_value=0.1)
    simulation_thread.run()
    simulation_thread = RunGnssSimulationProcess([p], cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
                                                 tot_num_vehicles=100, id=0, perception_probability=1,
                                                 estimate_detection_error=False, noise_value=0.5)
    simulation_thread.run()
    simulation_thread = RunGnssSimulationProcess([p], cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
                                                 tot_num_vehicles=100, id=0, perception_probability=1,
                                                 estimate_detection_error=False, noise_value=2)
    simulation_thread.run()
    simulation_thread = RunGnssSimulationProcess([p], cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
                                                 tot_num_vehicles=100, id=0, perception_probability=1,
                                                 estimate_detection_error=False, noise_value=5)
    simulation_thread.run()