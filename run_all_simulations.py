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
                 noise_distance=0, repeat=False, cont_prob=False, avg_speed_meter_per_sec=40000 / 3600):
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
        self.cont_prob = cont_prob
        self.avg_speed_meter_per_sec = avg_speed_meter_per_sec

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
            # hyper_params['message_size'] = 2000 * 8 # should be 490
            hyper_params['message_size'] = 490 # should be 490
            hyper_params['tot_num_vehicles'] = self.tot_num_vehicles
            hyper_params['time_threshold'] = self.time_threshold
            hyper_params['save_visual'] = True
            hyper_params['perception_probability'] = self.perception_probability
            hyper_params["estimate_detection_error"] = self.estimate_detection_error
            hyper_params["save_gnss"] = self.save_gnss
            hyper_params["noise_distance"] = self.noise_distance
            hyper_params["continous_probability"] = self.cont_prob
            hyper_params["avg_speed_meter_per_sec"] = self.avg_speed_meter_per_sec

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
                                        + ("_cont_prob" if hyper_params["continous_probability"] else "_discont_prob")
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
                   noise_distance=0, repeat=False, cont_prob=False, avg_speed_meter_per_sec=40000/3600):
    n_scenarios = 10
    s = time.time()

    for i in range(n_scenarios):
        print(f"Scenario: toronto_{i}")
        run_simulation_one_scenario(base_dir, cv2x_percentage, fov, view_range, num_RBs,
                                    tot_num_vehicles, i, perception_probability, estimate_detection_error,
                                    use_saved_seed, save_gnss, noise_distance, repeat, cont_prob, avg_speed_meter_per_sec)

        if i != n_scenarios-1:
            print("#######################################################################################################")

    print(f"Scenario toronto_0 - toronto_2 took {time.time() - s} seconds")
    print("#######################################################################################################\n")


def run_simulation_one_scenario(base_dir, cv2x_percentage, fov, view_range, num_RBs, tot_num_vehicles,
                                scenario_num, perception_probability, estimate_detection_error,
                                use_saved_seed=False, save_gnss=False, noise_distance=0, repeat=False, cont_prob=False,
                                avg_speed_meter_per_sec=40000 / 3600):
    import pickle

    time_threshold = 3
    n_threads = 12
    path = f"{base_dir}/toronto_{scenario_num}/"
    maps = os.listdir(path)
    maps = [path + map for map in maps if os.path.isdir(path+map)]
    maps.sort(key=lambda x:int(x.split('/')[-1]))

    if not repeat:
        filtered_maps = []
        for traffic in maps:
            scenario_path = os.path.join(traffic, "test.net.xml")

            state_path = os.path.join(os.path.dirname(scenario_path), "saved_state",
                                      "state_" + str(cv2x_percentage)
                                      + "_" + str(fov)
                                      + "_" + str(view_range)
                                      + "_" + str(num_RBs)
                                      + "_" + str(tot_num_vehicles)
                                      + "_" + str(time_threshold)
                                      + "_" + str(perception_probability)
                                      + ("_ede" if estimate_detection_error else "_nede")
                                      + "_" + str(noise_distance)
                                      + ("_egps" if noise_distance != 0 else "")
                                      + ("_cont_prob" if cont_prob else "_discont_prob")
                                      + ".pkl")


            if not os.path.isfile(state_path): # and not os.path.isfile(state_path):
                filtered_maps.append(traffic)
            else:
                b=False
                try:
                    with open(os.path.join(state_path), 'rb') as fw:
                        data = pickle.load(fw)
                        cv2x_vehicles, non_cv2x_vehicles, buildings, cv2x_perceived_non_cv2x_vehicles,\
                        scores_per_cv2x, los_statuses, vehicles, cv2x_perceived_non_cv2x_vehicles,\
                        cv2x_vehicles_perception_visible, tot_perceived_objects, tot_visible_objects = data
                except:
                    b=True

                if b:
                    filtered_maps.append(traffic)

        maps = filtered_maps

    if len(maps) == 0:
        print("No maps to do in this scenario! Going to the next scenario : )")
        return

    n = n_threads if n_threads<len(maps) else len(maps)
    block_size = len(maps)//n

    list_threads = []

    for i in range(n_threads):
        start = i * block_size
        end = start + block_size

        if i == n-1:
            end = len(maps)

        # print(maps[i])
        simulation_thread = RunSimulationProcess(maps[start:end], cv2x_percentage=cv2x_percentage, fov=fov,
                                                 view_range=view_range, num_RBs=num_RBs,
                                                 tot_num_vehicles=tot_num_vehicles, id=i, time_threshold=time_threshold,
                                                 perception_probability=perception_probability,
                                                 estimate_detection_error=estimate_detection_error,
                                                 use_saved_seed=use_saved_seed, save_gnss=save_gnss,
                                                 noise_distance=noise_distance, repeat=repeat, cont_prob=cont_prob,
                                                 avg_speed_meter_per_sec= avg_speed_meter_per_sec)

        simulation_thread.start()
        list_threads.append(simulation_thread)

    max_block_size = (len(maps) - block_size*(len(list_threads)-1)) # length of the last block
    max_block_size = max(block_size, max_block_size)
    timeout = 5 * 60 * max_block_size
    if timeout < 60:
        print("timeout was", timeout)
        timeout = 60

    print(f"Gonna wait {timeout} seconds for all threads")
    start_time = time.time()
    use_timeout = True

    while True:
        if use_timeout:
            time_taken = time.time() - start_time
            remaining_time = timeout - time_taken

            is_alive = False

            print("Waking up!")

            for i in range(n):
                if list_threads[i].is_alive():
                    is_alive = True
                    print(f"Thread {i} is alive!")
                else:
                    list_threads[i].join(0)
                    list_threads[i].terminate()
                    print(f"Thread {i} finished!")
                    # break

            if not is_alive:
                break

            if remaining_time < 0:
                print("Timeout! Killing remaining threads. . .")
                for i in range(n):
                    try:
                        list_threads[i].join(1)
                        list_threads[i].terminate()
                        print(f"Thread {i} killed")
                    except:
                        print(f"error ending thread {i}")

                break

            # print(f"Waiting threads, sleep {remaining_time} sec. . .")
            print(f"Waiting threads, sleep 1 min. . .")
            time.sleep(60)
            # time.sleep(remaining_time)
        else:
            for i in range(n):
                list_threads[i].join()
                list_threads[i].terminate()
                print(f"Thread {i} finished!")
            break
    cmd = 'pkill sumo'
    os.system(cmd)
    os.system(cmd)


class SIMULATION_TYPE(Enum):
    FIRST_PAPER=0,
    SECOND_PAPER=1,
    THIRD_PAPER=2,


if __name__ == '__main__':
    sim_type = SIMULATION_TYPE.THIRD_PAPER
    base_dir = "/home/bassel/toronto_AVpercentage_RBs"
    avg_speed_sec = 10
    min_num_vehicles = 100
    # base_dir = "/media/bassel/Career/toronto_content_selection/toronto_more_buses"
    # avg_speed_sec = 10
    min_num_vehicles = 100
    base_dir = "/media/bassel/Career/toronto_content_selection/toronto_dense"
    avg_speed_sec = 10
    min_num_vehicles = 400

    # delete_all_results = True
    delete_all_results = False
    # delete all results
    if delete_all_results:
        answer = input("Are you sure to delete ALL the results.txt and maps.png??")
        if answer == 'y' or answer == 'yes':
            n_scenarios = 10

            for i in range(n_scenarios):
                # path = f"/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_gps/toronto/toronto_{i}/"
                path = os.path.join(base_dir, f"toronto_{i}/")

                maps = os.listdir(path)
                maps = [path + map for map in maps]
                maps.sort(key=lambda x: int(x.split('/')[-1]))

                for p in maps:
                    # paths = glob.glob(p + "/result*.txt")
                    # for p2 in paths:
                    #     os.remove(p2)
                    paths = glob.glob(p + "/map*.png")

                    for p2 in paths:
                        os.remove(p2)
                    # paths = glob.glob(p + "/GNSS*.pkl")
                    # for p2 in paths:
                    #     os.remove(p2)
    exit(0)
    start_time = time.time()

    if sim_type == SIMULATION_TYPE.THIRD_PAPER:
        run_simulation(base_dir=base_dir,
                       cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100, tot_num_vehicles=min_num_vehicles,
                       perception_probability=1, estimate_detection_error=False, use_saved_seed=False,
                       repeat=False, save_gnss=False, noise_distance=0, cont_prob=False,
                       avg_speed_sec=avg_speed_sec)
        print("*******************************************************************************************")
    elif sim_type == SIMULATION_TYPE.SECOND_PAPER:

        ################################################   GPS   #######################################################
        # base_dir = '/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_gps/toronto'
        # for noise in [0, 0.1, 0.5, 2, 5]:
        #     print(f"Simulation noise: {noise}")
        #     run_simulation(base_dir, cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
        #                    tot_num_vehicles=100, perception_probability=1,
        #                    estimate_detection_error=False, use_saved_seed=True, noise_distance=noise, repeat=False,
        #                    continous_probability=False)

        #########################################   Error Detection   ##################################################
        # base_dir = '/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_gps/toronto'
        # for perc_porb in [0.9, 0.85]:
        #     for ede in [False, True]:
        #         print(f"Simulation perception probability: {perc_porb} and estimate detection error {ede}")
        #         run_simulation(base_dir=base_dir, cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100, tot_num_vehicles=100,
        #                        perception_probability=perc_porb, estimate_detection_error=ede, use_saved_seed=True)

        # #############################################    FOV     #####################################################
        for fov in [60, 90, 120, 240, 360]:
            for prob in [False]:
                s = time.time()
                print(f"Simulation fov: {fov}")
                # run_simulation(base_dir="/media/bassel/E256341D5633F0C1/toronto_fov/toronto",
                run_simulation(base_dir="/home/bassel/toronto_fov/toronto",
                               cv2x_percentage=0.65, fov=fov, view_range=75, num_RBs=100, tot_num_vehicles=100,
                               perception_probability=1, estimate_detection_error=False, use_saved_seed=True,
                               save_gnss=False, noise_distance=0, cont_prob=prob)

                e = time.time()

                print(f"Scenario toronto_0-9 took {e - s} seconds")
                print("")

        # ######################################   Test    ###############################################################
        # p = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_test"
        # simulation_thread = RunSimulationProcess([p], cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
        #                tot_num_vehicles=100, id=0, time_threshold=10, perception_probability=1, estimate_detection_error=False,
        #                                          use_saved_seed=True, save_gnss=True, noise_distance=0)
        # simulation_thread.run()
    elif sim_type == SIMULATION_TYPE.FIRST_PAPER:
        ###################################################################################################################
        for cv2_percentage in [0.25, 0.35, 0.45, 0.55, 0.65]:
            for num_RBs in [20, 30, 40, 50, 60, 70, 80, 90, 100]:
                print("cv2_percentage:", cv2_percentage)
                print("RB:", num_RBs)
                print("")
                run_simulation(base_dir=base_dir, cv2x_percentage=cv2_percentage, fov=120, view_range=75,
                               num_RBs=num_RBs, tot_num_vehicles=min_num_vehicles,
                               avg_speed_sec=avg_speed_sec, repeat=False)
        ###################################################################################################################
        # s = time.time()
        # run_simulation_one_scenario(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=90, tot_num_vehicles=100,
        #                             scenario_num=0, repeat=True)
        # e = time.time()
        # print(f"Done #1 cv2x_percentage=0.25, fov=120, view_range=75, num_RBs=75, tot_num_vehicles=100, total_time={e-s}")

    print("time taken:",time.time()-start_time)

print("All Simulations are done! Happy Integrating 9: )")
