import glob
import multiprocessing
import os
import shutil
import threading
import time
from ast import literal_eval
from enum import Enum
from traCI import Simulation


class RunSimulationProcess(multiprocessing.Process):
    def __init__(self, map, cv2x_percentage, fov, view_range, num_RBs, tot_num_vehicles, id,
                 perception_probability=1.0, estimate_detection_error=False, use_saved_seed=False):
        # multiprocessing.Process.__init__(self)
        super(RunSimulationProcess, self).__init__()

        if type(map) != list:
            sub_dirs = os.listdir(map)
            sub_dirs.sort()
            self.traffics = [os.path.join(map, dirname) for dirname in sub_dirs]
        else:
            self.traffics = map

        self.cv2x_percentage, self.fov, self.view_range, \
        self.num_RBs, self.tot_num_vehicles, self.perception_probability = cv2x_percentage, fov, view_range, num_RBs,\
                                                                          tot_num_vehicles, perception_probability
        self.repeat = use_saved_seed
        self.sim_id = id
        self.estimate_detection_error = estimate_detection_error

    def run(self):
        for traffic in self.traffics:  # toronto_'i'/'j'
            hyper_params = {}
            hyper_params['scenario_path'] = os.path.join(traffic, "test.net.xml")
            hyper_params['scenario_map'] = os.path.join(traffic, "net.sumo.cfg")
            hyper_params['scenario_polys'] = os.path.join(traffic, "map.poly.xml")
            hyper_params["cv2x_N"] = self.cv2x_percentage
            hyper_params["fov"] = self.fov
            hyper_params["view_range"] = self.view_range
            hyper_params["num_RBs"] = self.num_RBs
            hyper_params['message_size'] = 2000 * 8
            hyper_params['tot_num_vehicles'] = self.tot_num_vehicles
            hyper_params['time_threshold'] = 10
            hyper_params['save_visual'] = True
            hyper_params['perception_probability'] = self.perception_probability
            hyper_params["estimate_detection_error"] = self.estimate_detection_error

            with open(os.path.join(traffic,"basestation_pos.txt"), 'r') as fr:
                hyper_params["base_station_position"] = literal_eval(fr.readline())

            results_path = os.path.join(os.path.dirname(hyper_params['scenario_path']),
                                     "results_" + str(hyper_params['cv2x_N'])
                                     + "_" + str(hyper_params['fov'])
                                     + "_" + str(hyper_params["view_range"])
                                     + "_" + str(hyper_params["num_RBs"])
                                     + "_" + str(hyper_params["tot_num_vehicles"])
                                     + "_" + str(hyper_params['time_threshold'])
                                     + "_" + str(hyper_params['perception_probability'])
                                     + ("_ede" if hyper_params["estimate_detection_error"] else "_nede")
                                     + ".txt")

            if os.path.isfile(results_path):
                continue

            print(f"Simulation Thread {str(self.sim_id)} Scenario: {traffic[traffic.index('toronto_'):]} launched")
            sim = Simulation(hyper_params, str(self.sim_id)+"_"+os.path.basename(traffic))
            sim.run(self.repeat)
        print(f"Simulation Thread {str(self.sim_id)} Scenario: {traffic[traffic.index('toronto_'):]} ended")


def run_simulation(cv2x_percentage, fov, view_range, num_RBs, tot_num_vehicles,
                   perception_probability=1, estimate_detection_error=False, use_saved_seed=False):
    s = time.time()
    n_scenarios = 10

    for i in range(n_scenarios):
        run_simulation_one_scenario(cv2x_percentage, fov, view_range, num_RBs,
                                    tot_num_vehicles, i, perception_probability, estimate_detection_error, use_saved_seed)
        print("#######################################################################################################")

    e = time.time()

    print(f"Scenario "
          f"cv2x_percentage={cv2x_percentage}, fov={fov}, num_RBs={num_RBs},"
          f" toronto_0-9 took {e-s} seconds")


def run_simulation_one_scenario(cv2x_percentage, fov, view_range, num_RBs, tot_num_vehicles,
                                scenario_num, perception_probability, estimate_detection_error, repeat=False):
    n_threads = 12
    path = f"/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_{scenario_num}/"
    maps = os.listdir(path)
    maps = [path + map for map in maps]
    maps.sort(key=lambda x:int(x.split('/')[-1]))

    # maps = maps[99:]

    n = n_threads if n_threads<len(maps) else len(maps)
    block_Size = len(maps)//n
    list_threads = []

    for i in range(n_threads):
        start = i * block_Size
        end = start + block_Size

        if i == n-1:
            end = len(maps)

        # print(maps[i])
        simulation_thread = RunSimulationProcess(maps[start:end], cv2x_percentage=cv2x_percentage, fov=fov,
                                                 view_range=view_range, num_RBs=num_RBs,
                                                 tot_num_vehicles=tot_num_vehicles, id=i,
                                                 perception_probability=perception_probability,
                                                 estimate_detection_error=estimate_detection_error,
                                                 use_saved_seed=repeat)
        simulation_thread.start()
        list_threads.append(simulation_thread)

    for i in range(n):
        list_threads[i].join(timeout=len(maps[start:end]) * 4 * 60)
        print(f"Thread {i} done!")


class SIMULATION_TYPE(Enum):
    FIRST_PAPER=0,
    SECOND_PAPER=1


if __name__ == '__main__':
    sim_type = SIMULATION_TYPE.SECOND_PAPER
    delete_all_results = True
    # delete_all_results = False

    if sim_type == SIMULATION_TYPE.SECOND_PAPER:
        # delete all results
        if delete_all_results:
            answer = input("Are you sure to delete ALL the results.txt and maps.png??")
            if answer == 'y':
                n_scenarios = 10

                for i in range(n_scenarios):
                    path = f"/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_{i}/"
                    maps = os.listdir(path)
                    maps = [path + map for map in maps]
                    maps.sort(key=lambda x: int(x.split('/')[-1]))
                    for p in maps:
                        paths = glob.glob(p+"/result*.txt")
                        for p in paths:
                            os.remove(p)
                        paths = glob.glob(p+"/map*.png")
                        for p in paths:
                            os.remove(p)
                        paths = glob.glob(p + "/nede*")
                        for p in paths:
                            os.remove(p)

        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
                       tot_num_vehicles=100, perception_probability=1, estimate_detection_error=False, use_saved_seed=False)
        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
                       tot_num_vehicles=100, perception_probability=0.9, estimate_detection_error=False, use_saved_seed=True)
        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
                       tot_num_vehicles=100, perception_probability=0.9, estimate_detection_error=True, use_saved_seed=True)
        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
                       tot_num_vehicles=100, perception_probability=0.85, estimate_detection_error=False, use_saved_seed=True)
        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
                       tot_num_vehicles=100, perception_probability=0.85, estimate_detection_error=True, use_saved_seed=True)
        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
                       tot_num_vehicles=100, perception_probability=1, estimate_detection_error=True, use_saved_seed=True)
        # p = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_test"
        s = time.time()
        # simulation_thread = RunSimulationProcess([p], cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
        #                tot_num_vehicles=100, id=0, perception_probability=1, estimate_detection_error=False, use_saved_seed=True)
        # simulation_thread.run()
        # simulation_thread = RunSimulationProcess([p], cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
        #                tot_num_vehicles=100, id=1, perception_probability=1, estimate_detection_error=True, use_saved_seed=True)
        # simulation_thread.run()
        # simulation_thread = RunSimulationProcess([p], cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
        #                tot_num_vehicles=100, id=2, perception_probability=0.85, estimate_detection_error=False, use_saved_seed=True)
        # simulation_thread.run()
        # simulation_thread = RunSimulationProcess([p], cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
        #                tot_num_vehicles=100, id=3, perception_probability=0.85, estimate_detection_error=True, use_saved_seed=True)
        # simulation_thread.run()
        print("time taken:",time.time()-s)
    elif sim_type == SIMULATION_TYPE.FIRST_PAPER:
        ###################################################################################################################
        # Change CV2X percentage
        # Change #PRB
        run_simulation(cv2x_percentage=0.25, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.25, fov=120, view_range=75, num_RBs=30, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.25, fov=120, view_range=75, num_RBs=40, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.25, fov=120, view_range=75, num_RBs=50, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.25, fov=120, view_range=75, num_RBs=60, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.25, fov=120, view_range=75, num_RBs=70, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.25, fov=120, view_range=75, num_RBs=80, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.25, fov=120, view_range=75, num_RBs=90, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.25, fov=120, view_range=75, num_RBs=100, tot_num_vehicles=100)
        print("*******************************************************************************************")
        print("Done #1 cv2x_percentage=0.25, fov=120, view_range=75, num_RBs=20-100, tot_num_vehicles=100")

        run_simulation(cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=30, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=40, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=50, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=60, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=70, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=80, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=90, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=100, tot_num_vehicles=100)
        print("Done #2 cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=20-100, tot_num_vehicles=100")

        run_simulation(cv2x_percentage=0.45, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.45, fov=120, view_range=75, num_RBs=30, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.45, fov=120, view_range=75, num_RBs=40, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.45, fov=120, view_range=75, num_RBs=50, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.45, fov=120, view_range=75, num_RBs=60, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.45, fov=120, view_range=75, num_RBs=70, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.45, fov=120, view_range=75, num_RBs=80, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.45, fov=120, view_range=75, num_RBs=90, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.45, fov=120, view_range=75, num_RBs=100, tot_num_vehicles=100)
        print("Done #3 cv2x_percentage=0.45, fov=120, view_range=75, num_RBs=20-100, tot_num_vehicles=100")

        run_simulation(cv2x_percentage=0.55, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.55, fov=120, view_range=75, num_RBs=30, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.55, fov=120, view_range=75, num_RBs=40, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.55, fov=120, view_range=75, num_RBs=50, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.55, fov=120, view_range=75, num_RBs=60, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.55, fov=120, view_range=75, num_RBs=70, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.55, fov=120, view_range=75, num_RBs=80, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.55, fov=120, view_range=75, num_RBs=90, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.55, fov=120, view_range=75, num_RBs=100, tot_num_vehicles=100)
        print("Done #3 cv2x_percentage=0.55, fov=120, view_range=75, num_RBs=20-100, tot_num_vehicles=100")

        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=30, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=40, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=50, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=60, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=70, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=80, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=90, tot_num_vehicles=100)
        print("*******************************************************************************************")
        run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100, tot_num_vehicles=100)
        print("Done #5 cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=20-100, tot_num_vehicles=100")

        ####################################################################################################################
        run_simulation(cv2x_percentage=0.65, fov=60, view_range=75, num_RBs=100, tot_num_vehicles=100)
        print("*******************************************************************************************")
        print("Done #6 cv2x_percentage=0.65, fov=60, view_range=75, num_RBs=90, tot_num_vehicles=100")
        run_simulation(cv2x_percentage=0.65, fov=90, view_range=75, num_RBs=100, tot_num_vehicles=100)
        print("Done #7 cv2x_percentage=0.65, fov=90, view_range=75, num_RBs=90, tot_num_vehicles=100")

        ###################################################################################################################
        # s = time.time()
        # run_simulation_one_scenario(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=90, tot_num_vehicles=100,
        #                             scenario_num=0, repeat=True)
        # e = time.time()
        # print(f"Done #1 cv2x_percentage=0.25, fov=120, view_range=75, num_RBs=75, tot_num_vehicles=100, total_time={e-s}")


print("All Simulations are done! Happy Integrating 9: )")
