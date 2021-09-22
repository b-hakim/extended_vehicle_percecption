import glob
import multiprocessing
import os
import shutil
import threading
from ast import literal_eval

from traCI import Simulation


class RunSimulationThread(multiprocessing.Process):
    def __init__(self, map, cv2x_percentage, fov, view_range, num_RBs, tot_num_vehicles, id):
        # multiprocessing.Process.__init__(self)
        super(RunSimulationThread, self).__init__()

        self.map = map
        sub_dirs = os.listdir(map)
        sub_dirs.sort()
        self.traffics = [os.path.join(map, dirname) for dirname in sub_dirs]
        self.cv2x_percentage, self.fov, self.view_range, self.num_RBs, self.tot_num_vehicles = cv2x_percentage, fov,\
                                                                                               view_range, num_RBs,\
                                                                                               tot_num_vehicles
        self.sim_id = id

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

            with open(os.path.join(traffic,"basestation_pos.txt"), 'r') as fr:
                hyper_params["base_station_position"] = literal_eval(fr.readline())

            print(f"Simulation Toronto_{str(self.sim_id)}_{os.path.basename(traffic)} launched")
            sim = Simulation(hyper_params, str(self.sim_id)+"_"+os.path.basename(traffic))
            sim.run()


def run_simulation(cv2x_percentage, fov, view_range, num_RBs, tot_num_vehicles):
    n_threads = 10
    path = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/"
    maps = os.listdir(path)
    maps = [path + map for map in maps]
    maps.sort()

    n = n_threads if n_threads<len(maps) else len(maps)
    block_Size = len(maps)//n
    list_threads = []

    for i in range(n_threads):
        # start = i * block_Size
        # end = start + block_Size

        # if i == n-1:
        #     end = len(maps)
        print(maps[i])
        simulation_thread = RunSimulationThread(maps[i], cv2x_percentage=cv2x_percentage, fov=fov, view_range=view_range,
                                                num_RBs=num_RBs, tot_num_vehicles=tot_num_vehicles, id=i)
        simulation_thread.start()
        list_threads.append(simulation_thread)

    for i in range(n):
        list_threads[i].join()


if __name__ == '__main__':
    run_simulation(cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=150)
    print("first one is done")
    run_simulation(cv2x_percentage=0.45, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=150)
    run_simulation(cv2x_percentage=0.55, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=150)
    run_simulation(cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=150)

    print("All Simulations are done! Happy Integrating 9: )")
