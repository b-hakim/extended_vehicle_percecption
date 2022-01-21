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
    def __init__(self, map, cv2x_percentage, fov, view_range, num_RBs, tot_num_vehicles, id, time_threshold,
                 perception_probability=1.0, estimate_detection_error=False, use_saved_seed=False, save_gnss=False,
                 noise_distance=0, repeat=False):
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
        self.use_saved_seed = use_saved_seed
        self.sim_id = id
        self.time_threshold = time_threshold
        self.estimate_detection_error = estimate_detection_error
        self.save_gnss = save_gnss
        self.noise_distance = noise_distance
        self.repeat = repeat


    def run(self):
        print(f"Simulation Thread {str(self.sim_id)} started with a batchsize={len(self.traffics)} ")
        tot_time_start = time.time()

        for traffic in self.traffics:  # toronto_'i'/'j'
            sc_time_start = time.time()
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
            hyper_params['time_threshold'] = self.time_threshold
            hyper_params['save_visual'] = True
            hyper_params['perception_probability'] = self.perception_probability
            hyper_params["estimate_detection_error"] = self.estimate_detection_error
            hyper_params["save_gnss"] = self.save_gnss
            hyper_params["noise_distance"] = self.noise_distance

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
                                        + "_" + str(hyper_params["noise_distance"])
                                        + ("_egps" if hyper_params["noise_distance"] else "")
                                        + ".txt")

            if os.path.isfile(results_path) and not self.repeat:
                continue

            print(f"Scenario: {traffic[traffic.index('toronto_'):]} started")

            sim = Simulation(hyper_params, str(self.sim_id)+"_"+os.path.basename(traffic))
            sim.run(self.use_saved_seed)

            print(f"Scenario {traffic[traffic.index('toronto_'):]} ended with {time.time()-sc_time_start} seconds")

        print(f"Simulation Thread {str(self.sim_id)} ended with {time.time()-tot_time_start} seconds")


def run_simulation(base_dir, cv2x_percentage, fov, view_range, num_RBs, tot_num_vehicles,
                   perception_probability=1, estimate_detection_error=False, use_saved_seed=False, save_gnss=False,
                   noise_distance=0, repeat=False):
    n_scenarios = 3
    s = time.time()

    for i in range(n_scenarios):
        print(f"Scenario: toronto_{i}")
        run_simulation_one_scenario(base_dir, cv2x_percentage, fov, view_range, num_RBs,
                                    tot_num_vehicles, i, perception_probability, estimate_detection_error,
                                    use_saved_seed, save_gnss, noise_distance, repeat)

        if i != n_scenarios-1:
            print("#######################################################################################################")

    print(f"Scenario toronto_0 - toronto_2 took {time.time() - s} seconds")
    print("#######################################################################################################\n")


def run_simulation_one_scenario(base_dir, cv2x_percentage, fov, view_range, num_RBs, tot_num_vehicles,
                                scenario_num, perception_probability, estimate_detection_error,
                                use_saved_seed=False, save_gnss=False, noise_distance=0, repeat=False):
    time_threshold = 10
    n_threads = 12
    path = f"{base_dir}/toronto_{scenario_num}/"
    maps = os.listdir(path)
    maps = [path + map for map in maps]
    maps.sort(key=lambda x:int(x.split('/')[-1]))

    if not repeat:
        filtered_maps = []
        for traffic in maps:
            scenario_path = os.path.join(traffic, "test.net.xml")
            results_path = os.path.join(os.path.dirname(scenario_path),
                                        "results_" + str(cv2x_percentage)
                                        + "_" + str(fov)
                                        + "_" + str(view_range)
                                        + "_" + str(num_RBs)
                                        + "_" + str(tot_num_vehicles)
                                        + "_" + str(time_threshold)
                                        + "_" + str(perception_probability)
                                        + ("_ede" if estimate_detection_error else "_nede")
                                        + "_" + str(noise_distance)
                                        + ("_egps" if noise_distance else "")
                                        + ".txt")

            if not os.path.isfile(results_path):
                filtered_maps.append(traffic)

        maps = filtered_maps

    if len(maps) == 0:
        print("No maps to do in this scenario! Going to the next scenario : )")
        return

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
                                                 tot_num_vehicles=tot_num_vehicles, id=i, time_threshold=time_threshold,
                                                 perception_probability=perception_probability,
                                                 estimate_detection_error=estimate_detection_error,
                                                 use_saved_seed=use_saved_seed, save_gnss=save_gnss,
                                                 noise_distance=noise_distance, repeat=repeat)
        simulation_thread.start()
        list_threads.append(simulation_thread)

    timeout = (len(maps) - (n - 1) * block_Size) * 5 * 60
    print(f"Gonna wait {timeout} seconds for all threads")
    # timeout = block_Size * 1 * 60
    running_time = time.time()

    while True:
        time_taken = time.time() - running_time
        remaining_time = timeout - time_taken

        if remaining_time < 0:
            for i in range(n):
                try:
                    list_threads[i].join(1)
                    list_threads[i].terminate()
                except:
                    print(f"error ending thread {i}")

            break

        time.sleep(remaining_time)
        print(f"Thread {i} done!")

    cmd = 'pkill sumo'
    os.system(cmd)
    os.system(cmd)


class SIMULATION_TYPE(Enum):
    FIRST_PAPER=0,
    SECOND_PAPER=1


if __name__ == '__main__':
    sim_type = SIMULATION_TYPE.SECOND_PAPER
    # delete_all_results = True
    delete_all_results = False

    if sim_type == SIMULATION_TYPE.SECOND_PAPER:
        # delete all results
        if delete_all_results:
            answer = input("Are you sure to delete ALL the results.txt and maps.png??")
            if answer == 'y':
                n_scenarios = 10

                for i in range(n_scenarios):
                    path = f"/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_gps/toronto/toronto_{i}/"
                    # path = f"/media/bassel/E256341D5633F0C1/toronto_fov/toronto/toronto_{i}/"
                    maps = os.listdir(path)
                    maps = [path + map for map in maps]
                    maps.sort(key=lambda x: int(x.split('/')[-1]))
                    for p in maps:
                        paths = glob.glob(p+"/result*.txt")
                        for p2 in paths:
                            os.remove(p2)
                        paths = glob.glob(p+"/map*.png")
                        for p2 in paths:
                            os.remove(p2)
                        paths = glob.glob(p+"/GNSS*.pkl")
                        for p2 in paths:
                            os.remove(p2)

        start_time = time.time()

        ################################################   GPS   #######################################################
        base_dir = '/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_gps/toronto'
        for noise in [0, 0.1, 0.5, 2, 5]:
            print(f"Simulation noise: {noise}")
            run_simulation(base_dir, cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
                           tot_num_vehicles=100, perception_probability=1,
                           estimate_detection_error=False, use_saved_seed=True, noise_distance=noise, repeat=False)

        #########################################   Error Detection   ##################################################
        # base_dir = '/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_gps/toronto'
        # for perc_porb in [0.9, 0.85]:
        #     for ede in [False, True]:
        #         print(f"Simulation perception probability: {perc_porb} and estimate detection error {ede}")
        #         run_simulation(base_dir=base_dir, cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100, tot_num_vehicles=100,
        #                        perception_probability=perc_porb, estimate_detection_error=ede, use_saved_seed=True)

        # #############################################    FOV     #####################################################
        # for fov in [60, 90, 120, 240, 360]:
        # # for fov in [240, 360, 60, 90, 120]:
        #     print(f"Simulation fov: {fov}")
        #     run_simulation(base_dir="/media/bassel/E256341D5633F0C1/toronto_fov/toronto",
        #                    cv2x_percentage=0.65, fov=fov, view_range=75, num_RBs=100, tot_num_vehicles=100,
        #                    perception_probability=1, estimate_detection_error=False, use_saved_seed=True,
        #                    save_gnss=False, noise_distance=0)
        #
        #     e = time.time()
        #
        #     print(f"Scenario toronto_0-9 took {e - s} seconds")
        #     print("")

        # ######################################   Test    ###############################################################
        # p = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_test"
        # simulation_thread = RunSimulationProcess([p], cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
        #                tot_num_vehicles=100, id=0, time_threshold=10, perception_probability=1, estimate_detection_error=False,
        #                                          use_saved_seed=True, save_gnss=True, noise_distance=0)
        # simulation_thread.run()

        print("time taken:",time.time()-start_time)
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
