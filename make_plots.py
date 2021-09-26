import os
import numpy as np


def verify_results_exists(path, cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=150, time_threshold=10):
    # loop on all files
    # print un-existing results file

    scenarios_dirs = os.listdir(path)

    if len(scenarios_dirs) == 10:
        scenarios_dirs = [[path+"/"+p+"/"+pp for pp in os.listdir(path+"/"+p)] for p in scenarios_dirs]
        scenarios_dirs = np.array(scenarios_dirs).reshape(-1).tolist()

    scenarios_dirs.sort()

    for scenario_path in scenarios_dirs:
        results_path = os.path.join(path, scenario_path,
                                "results_" + str(cv2x_percentage)
                                     + "_" + str(fov)
                                     + "_" + str(view_range)
                                     + "_" + str(num_RBs)
                                     + "_" + str(tot_num_vehicles)
                                     + "_" + str(time_threshold)
                                     + ".txt")
        if not os.path.isfile(results_path):
            print("Missing:", results_path)

def save_plot(path, cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=150, time_threshold=10):
    # loop on all files
    # read results file
    # parse required data
    # save & show plots
    scenarios_dirs = os.listdir(path)

    if len(scenarios_dirs) == 10:
        scenarios_dirs = [[path+"/"+p+"/"+pp for pp in os.listdir(path+"/"+p)] for p in scenarios_dirs]
        scenarios_dirs = np.array(scenarios_dirs).reshape(-1).tolist()

    scenarios_dirs.sort()
    avg_sent = 0
    avg_unsent = 0
    avg_perceived = 0
    count = 0

    for scenario_path in scenarios_dirs:
        results_path = os.path.join(path, scenario_path,
                                "results_" + str(cv2x_percentage)
                                     + "_" + str(fov)
                                     + "_" + str(view_range)
                                     + "_" + str(num_RBs)
                                     + "_" + str(tot_num_vehicles)
                                     + "_" + str(time_threshold)
                                     + ".txt")
        # print(results_path, os.path.isfile(results_path))
        with open(results_path) as fr:
            lines = fr.readlines()

        for i, line in enumerate(lines):
            if line.find("Total Requests") != -1:
                avg_sent += int(lines[i+1].split(": ")[1])
                avg_unsent += int(lines[i+2].split(": ")[1])
                avg_perceived += int(lines[i+3].split(": ")[1])
                count += 1
                break

    print("avg sent: ", avg_sent/count)
    print("avg unsent", avg_unsent/count)
    print("avg perceived", avg_perceived/count)

if __name__ == '__main__':
    for i in range(10):
        verify_results_exists(f'/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_{i}',
                              cv2x_percentage=0.25, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=150)
        verify_results_exists(f'/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_{i}',
                              cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=150)
        verify_results_exists(f'/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_{i}',
                              cv2x_percentage=0.45, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=150)
        verify_results_exists(f'/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_{i}',
                              cv2x_percentage=0.55, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=150)
        verify_results_exists(f'/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_{i}',
                              cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=150)


    # save_plot('/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_0',
    #           cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=150, time_threshold=10)