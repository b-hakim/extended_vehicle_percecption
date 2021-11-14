import os
import numpy as np
import matplotlib.pyplot as plt


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


def save_plot(path, cv2x_percentage=0.35, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=150, time_threshold=10,
              perception_probability=1, estimate_detection_error=False, noise_distance=None):
    # loop on all files
    # read results file
    # parse required data
    # save & show plots
    scenarios_dirs = os.listdir(path)

    if len(scenarios_dirs) == 10:
        scenarios_dirs = scenarios_dirs[:3]
        scenarios_dirs = [[path+"/"+p+"/"+pp for pp in os.listdir(path+"/"+p)] for p in scenarios_dirs]
        scenarios_dirs = np.array(scenarios_dirs).reshape(-1).tolist()
    else:
        scenarios_dirs = [path + "/" + p for p in scenarios_dirs]

    scenarios_dirs.sort(key=lambda p: int(p.split("/")[-1]))
    avg_req = 0
    avg_uniq = 0
    avg_dup = 0
    avg_sent = 0
    avg_unsent = 0
    avg_perceived = 0
    avg_objective_value = 0
    avg_visible_objects = 0
    avg_perceived_sent = 0
    avg_visible_sent = 0
    count = 0

    sent_y = []
    visible_objects_y = []
    perceived_sent_y = []
    visible_sent_y = []
    percentage_sent_seen_y = []
    x = list(range(0, 300))

    for scenario_path in scenarios_dirs:
        results_path = os.path.join(path, scenario_path,
                                "results_" + str(cv2x_percentage)
                                     + "_" + str(fov)
                                     + "_" + str(view_range)
                                     + "_" + str(num_RBs)
                                     + "_" + str(tot_num_vehicles)
                                     + "_" + str(time_threshold)
                                     + "_" + str(perception_probability)
                                    + ("_ede" if estimate_detection_error else "_nede")
                                    + ".txt")

        # print(results_path, os.path.isfile(results_path))
        with open(results_path) as fr:
            lines = fr.readlines()

        sent = 0

        for i, line in enumerate(lines):
            if line.find("Objective value") != -1:
                avg_objective_value += float(line.split(" = ")[1])
            elif line.find("Total Duplicate Requests") != -1:
                avg_req += int(lines[i].split(": ")[1])
                avg_uniq += int(lines[i+1].split(": ")[1])
                avg_dup += int(lines[i].split(": ")[1]) - int(lines[i+1].split(": ")[1])

                avg_sent += int(lines[i+2].split(": ")[1])
                sent = int(lines[i+2].split(": ")[1])
                avg_unsent += int(lines[i+3].split(": ")[1])
                avg_perceived += int(lines[i+4].split(": ")[1])
                avg_visible_objects += int(lines[i+5].split(": ")[1])
                avg_perceived_sent += int(lines[i+6].split(": ")[1])
                avg_visible_sent += int(lines[i+7].split(": ")[1])

                count += 1
                break

        # sent_y.append(avg_sent/count)
        visible_objects_y.append(avg_visible_objects/count)
        perceived_sent_y.append(avg_perceived_sent/count)
        visible_sent_y.append(avg_visible_sent/count)
        # percentage_sent_seen_y.append(100*(avg_sent/count)/(avg_perceived/count))

    summary_name = f"results_summary/summary_{cv2x_percentage}_{num_RBs}_{perception_probability}" \
                   f"_{'_ede' if estimate_detection_error else '_nede'}.txt"


    # plt.figure()
    # plt.plot(x, sent_y)
    # plt.xlabel("Simulation number")
    # plt.ylabel("Average sent")
    # plt.title(f"AV: {cv2x_percentage}% RBs: {num_RBs}")
    # # plt.show()
    # plt.savefig(f"results_summary/avg_sent_{cv2x_percentage}_{num_RBs}.png")
    # plt.close()
    # plt.figure()
    # plt.plot(x, percentage_sent_seen_y)
    # plt.xlabel("Simulation number")
    # plt.ylabel("Average Added Information")
    # plt.title(f"AV: {cv2x_percentage}% RBs: {num_RBs}")
    # # plt.show()
    # plt.savefig(f"results_summary/avg_new_info_{cv2x_percentage}_{num_RBs}.png")
    plt.figure()
    plt.plot(x, visible_objects_y)
    plt.xlabel("Simulation number")
    plt.ylabel("visible_objects_y")
    plt.title(f"visible_objects_y")
    # plt.show()
    plt.savefig(summary_name.replace("txt", "png").replace("/summary", "/visible_objects"))
    plt.figure()
    plt.plot(x, perceived_sent_y)
    plt.xlabel("Simulation number")
    plt.ylabel("perceived_sent_y")
    plt.title(f"perceived_sent_y")
    # plt.show()
    plt.savefig(summary_name.replace("txt", "png").replace("/summary", "/perceived_sent"))
    plt.figure()
    plt.plot(x, visible_sent_y)
    plt.xlabel("Simulation number")
    plt.ylabel("visible_sent_y")
    plt.title(f"visible_sent_y")
    # plt.show()
    plt.savefig(summary_name.replace("txt", "png").replace("/summary", "/visible_sent"))

    print("***  " + str(cv2x_percentage) + " - " + str(num_RBs) + "  ***")
    print("avg_req: ", np.round(avg_req/count))
    print("avg_uniq: ", np.round(avg_uniq/count))
    print("avg_dup: ", np.round(avg_dup/count))
    print("avg_sent: ", np.round(avg_sent/count))
    print("avg unsent", np.round(avg_unsent/count))
    print("perc_duplicate/tot_req: " + str(np.round(100 * (avg_dup) / (avg_req), 1)))
    print("perc_sent/unique: " + str(np.round(avg_sent / avg_uniq, 1)))
    print("avg perceived", np.round(avg_perceived/count))
    print("avg sent/perceived", np.round(100*(avg_sent/count)/(avg_perceived/count)))
    print("avg_obj_value: ", np.round(avg_objective_value/count))
    print("avg_visible_objects: ", np.round(avg_visible_objects/count))
    print("avg_perceived_sent: ", np.round(avg_perceived_sent/count))
    print("avg_visible_sent: ", np.round(avg_visible_sent/count))
    print()

    plt.close()

    with open(summary_name, 'w') as fw:
        fw.writelines([
            "avg_req: " + str(np.round(avg_req / count, 2)),
            "\navg_uniq: " + str(np.round(avg_uniq / count, 2)),
            "\navg_dup: " + str(np.round(avg_dup / count, 2)),
            "\navg_sent: " + str(np.round(avg_sent / count, 2)),

            "\nperc_duplicate/tot_req: " + str(np.round(100*avg_dup/avg_req, 1)),
            "\nperc_sent/unique: " + str(np.round(100*avg_sent / avg_uniq, 1)),

            "\navg unsent: " + str(np.round(avg_unsent / count, 2)),
            "\navg perceived: " + str(np.round(avg_perceived / count, 2)),
            "\navg seen/perceived: " + str(np.round(100*(avg_sent / count) / (avg_perceived / count), 1)),
            "\navg objective value: " + str(np.round((avg_objective_value/ count) , 2)),
            "\navg perceived + visible objects: " + str(np.round((avg_visible_objects/ count - avg_perceived / count) , 2)),
            "\navg perceived and sent: " + str(np.round((avg_perceived_sent/ count) , 2)),
            "\navg not perceived but visible and sent: " + str(np.round((avg_visible_sent/ count) , 2)),
        ])


def plot_summary_sent_percentage():
    plt.figure()
    x = [20, 30, 40, 50, 60, 70, 80, 90, 100]

    for cv2x_percentage in [0.25, 0.35, 0.45, 0.55, 0.65]:
        y = []
        for rb in [20, 30, 40, 50, 60, 70, 80, 90, 100]:
            with open(f"results_summary/summary_{cv2x_percentage}_{rb}.txt") as fr:
                lines = fr.readlines()
            y.append(float(lines[5].split(":")[1]))
        plt.plot(x, y)

    plt.xlabel("Available number of RBs")
    plt.ylabel("Avg. Percentage Sent Messages")
    plt.legend(["25%", "35%", "45%", "55%", "65%"])
    plt.savefig("results_summary/summary_sent_percentage.png")


def plot_summary_sent_messages():
    plt.figure()
    x = [20, 30, 40, 50, 60, 70, 80, 90, 100]

    for cv2x_percentage in [0.25, 0.35, 0.45, 0.55, 0.65]:
        y = []
        for rb in [20, 30, 40, 50, 60, 70, 80, 90, 100]:
            with open(f"results_summary/summary_{cv2x_percentage}_{rb}.txt") as fr:
                lines = fr.readlines()
            y.append(float(lines[3].split(":")[1]))
        plt.plot(x, y)

    plt.xlabel("Available number of RBs")
    plt.ylabel("Avg number Sent Messages")
    plt.legend(["25%", "35%", "45%", "55%", "65%"])
    plt.savefig("results_summary/summary_sent_messages.png")
    plt.show()


def plot_summary_obj_function_performed():
    x = [20, 30, 40, 50, 60, 70, 80, 90, 100]

    for cv2x_percentage in [0.25, 0.35, 0.45, 0.55, 0.65]:
        y = []
        for rb in [20, 30, 40, 50, 60, 70, 80, 90, 100]:
            count = 0
            avg_total_obj_value = 0
            avg_executed_obj_value = 0
            for i in range(10):
                for j in range(100):
                    with open(f"D:/sumo_traffic/sumo_map/toronto/toronto_{i}/{j}/results_{cv2x_percentage}_120_75_{rb}_100_10.txt") as fr:
                        lines = fr.readlines()
                    avg_total_obj_value += float(lines[2].split(" = ")[1])

                    # print(f"toronto_{i}/{j}/results_{cv2x_percentage}_120_75_{rb}_100_10.txt")
                    avg_executed_obj_value += float(lines[7 if lines[6] == "Solution:\n" else 8].split(" = ")[1])
                    count += 1

            avg_total_obj_value /= count
            avg_executed_obj_value /= count

            v = np.round(100 * avg_executed_obj_value / avg_total_obj_value, 2)
            y.append(v)

            print(cv2x_percentage, rb, v)
        plt.plot(x, y)

    plt.xlabel("Available number of RBs")
    plt.ylabel("Percentage Executed Objective Value")
    plt.legend(["25%", "35%", "45%", "55%", "65%"])
    plt.savefig("results_summary/summary_executed_objective_value_percentage.png")
    plt.show()


    print(list(zip([0.25, 0.35, 0.45, 0.55, 0.65], y)))


def plot_summary_avg_objective_value():
    plt.figure()
    x = [20, 30, 40, 50, 60, 70, 80, 90, 100]

    for cv2x_percentage in [0.25, 0.35, 0.45, 0.55, 0.65]:
        y = []
        for rb in [20, 30, 40, 50, 60, 70, 80, 90, 100]:
            with open(f"results_summary/summary_{cv2x_percentage}_{rb}.txt") as fr:
                lines = fr.readlines()
            y.append(float(lines[9].split(":")[1]))
        plt.plot(x, y)

    plt.xlabel("Available number of RBs")
    plt.ylabel("Avg Objective Value")
    plt.legend(["25%", "35%", "45%", "55%", "65%"])
    plt.savefig("results_summary/summary_objective_values.png")
    plt.show()


def plot_summary_additional_perceived_information():
    plt.figure()
    x = [20, 30, 40, 50, 60, 70, 80, 90, 100]

    for cv2x_percentage in [0.25, 0.35, 0.45, 0.55, 0.65]:
        y = []
        for rb in [20, 30, 40, 50, 60, 70, 80, 90, 100]:
            with open(f"results_summary/summary_{cv2x_percentage}_{rb}.txt") as fr:
                lines = fr.readlines()
            y.append(float(lines[8].split(":")[1]))
        plt.plot(x, y)

    plt.xlabel("Available number of RBs")
    plt.ylabel("Avg Sent-Perceived Percentage")
    plt.legend(["25%", "35%", "45%", "55%", "65%"])
    plt.savefig("results_summary/summary_sent_perceived_percentage.png")
    plt.show()


def plot_summary_sent_percentage():
    plt.figure()
    x = [20, 30, 40, 50, 60, 70, 80, 90, 100]

    for cv2x_percentage in [0.25, 0.35, 0.45, 0.55, 0.65]:
        y = []
        for rb in [20, 30, 40, 50, 60, 70, 80, 90, 100]:
            with open(f"results_summary/summary_{cv2x_percentage}_{rb}.txt") as fr:
                lines = fr.readlines()
            y.append(float(lines[5].split(":")[1]))
        plt.plot(x, y)

    plt.xlabel("Available number of RBs")
    plt.ylabel("Avg. Percentage Sent Messages")
    plt.legend(["25%", "35%", "45%", "55%", "65%"])
    plt.savefig("results_summary/summary_sent_percentage.png")


if __name__ == '__main__':
    # for i in range(10):
    #     for cv2x_percentage in [0.25, 0.35, 0.45, 0.55, 0.65]:
    #         for rb in [20, 30, 40, 50, 60, 70, 80, 90, 100]:
    #             verify_results_exists(f'/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_{i}',
    #                           cv2x_percentage=cv2x_percentage, fov=120, view_range=75,
    #                                   num_RBs=rb, tot_num_vehicles=100)

    # for cv2x_percentage in [0.25, 0.35, 0.45, 0.55, 0.65]:
    #     for rb in [20, 30, 40, 50, 60, 70, 80, 90, 100]:
    #         save_plot('/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto' if os.name != 'nt' else r"D:\sumo_traffic\sumo_map\toronto",
    #                   cv2x_percentage=cv2x_percentage, fov=120, view_range=75, num_RBs=rb,
    #                   tot_num_vehicles=100, time_threshold=10)

    # plot_summary_sent_percentage()
    # plot_summary_obj_function_performed()
    # plot_summary_sent_messages()
    # plot_summary_avg_objective_value()
    # plot_summary_additional_perceived_information()
    # for i in range(3):
    for perception_probability in [1, 0.9, 0.85]:
        for estimate_detection_error in [True, False]:
            save_plot(f'/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/',
                          cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100,
                          tot_num_vehicles=100, time_threshold=10, perception_probability=perception_probability,
                          estimate_detection_error=estimate_detection_error, noise_distance=None)
    # save_plot('/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_0',
    #           cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=20, tot_num_vehicles=100, time_threshold=10)

    # save_plot('/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/toronto_0',
    #           cv2x_percentage=0.65, fov=120, view_range=75, num_RBs=100, tot_num_vehicles=100, time_threshold=10)

    '''
    Comparison between hyper-parameters: 
    
        a) for a given % of vehicles what is the best minimum RB to be allocated. 
        That is do a plot with x axis being % of vehicles and y axis the avg number of unique messages sent among 100 
        and 1000 scenarioes (2 plots). 
        
        For each number, check the convergence, that is how many out of the 100 or 1000 are needed till there is 
        no big change in the number of unique messages 
        
        b) Comparison between several FOV
    '''

    # avg total objective function
    # avg sent objective function
    # repeat for 0.23, 0.35 etc..