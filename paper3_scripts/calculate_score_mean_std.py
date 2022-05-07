import os
import pickle
import numpy as np
import matplotlib.pyplot as plt


def calculate_stats(basedir):
    scenarios = []
    all_scores = []

    for i in range(3):
        scenario_trials = []
        for j in range(100):
            path = os.path.join(basedir, "toronto_"+str(i), str(j), 'saved_state/cv2x_non_buildings.pkl')

            with open(path, 'rb') as fr:
                trial = pickle.load(fr) # cv2x_vehicles, non_cv2x_vehicles, buildings, av_perceiving_nav, scores_per_cv2x, los_statuses

            scenario_trials.append(trial)

        scenarios.append(scenario_trials)

    for scenario_id, scenario in enumerate(scenarios):
        lst_max_scores = []

        for trial_id, scenario_trial in enumerate(scenario):
            for sender, scores in scenario_trial[4].items():
                scores = sorted (scores, key=lambda x: x[1], reverse=True)
                for i, score in enumerate(scores):
                    if score[1] == 0:
                        break

                    all_scores += [s[1] for s in scores if s[1]!=0]

                    if i == len(lst_max_scores):
                        lst_max_scores.append([])

                    lst_max_scores[i].append(score[1])

        lst_mean_sigma=[]

        for score_lst in lst_max_scores:
            lst_mean_sigma.append([np.mean(score_lst), np.std(score_lst)])

        path = os.path.join(basedir, "toronto_"+str(scenario_id),  'map_scores_distribution.npy')
        np.save(path, lst_mean_sigma)
        print(f"scenario {scenario_id} u,o:", lst_mean_sigma)

    all_scores = np.array(all_scores)
    path = os.path.join(basedir, "all_scores.npy")
    np.save(path, all_scores)


def calculate_vehicles_perceived_more_than_one_AV(basedir):
    num_vehicles = [0, 0]

    for i in range(3):
        for j in range(100):
            path = os.path.join(basedir, "toronto_"+str(i), str(j), 'saved_state/cv2x_non_buildings.pkl')

            with open(path, 'rb') as fr:
                _, _, _, av_perceiving_nav, scores_per_cv2x, _ = pickle.load(fr)

            nav_seeer = {}
            c=0
            matched_before = {}

            for sender_av, scores in scores_per_cv2x.items():
                scores = sorted(scores, key=lambda x: x[1], reverse=True)
                scores_per_cv2x[sender_av] = scores

                for score in scores:
                    id = score[2].vehicle_id

                    if score[1] == 0:
                        break

                    if id in matched_before:
                        continue

                    for sender2_av, sender2_perceived_navs in av_perceiving_nav.items():
                        if score[2].vehicle_id in sender2_perceived_navs:
                            if id in nav_seeer:
                                nav_seeer[id] += 1
                            else:
                                nav_seeer[id] = 1

                    matched_before[id] = True
            avg = 0
            max = 0

            for id in nav_seeer:
                if nav_seeer[id] > max:
                    max = nav_seeer[id]
                avg += nav_seeer[id]

            avg /= len(nav_seeer.keys())
            num_vehicles = [num_vehicles[0]+avg, num_vehicles[1] if num_vehicles[1]>max else max]

    num_vehicles[0] /= 300
    # num_vehicles[1] /= 300
    print("avg, max", num_vehicles)

def plot_scores_hist(basedir):
    path = os.path.join(basedir, "all_scores.npy")
    all_scores = np.load(path)
    print(all_scores.shape)
    all_scores = all_scores[all_scores<1]
    print(all_scores.shape)
    min_val, max_val = all_scores.min(), all_scores.max()
    n = 100
    step = (max_val-min_val)/n
    print(min_val, max_val, step)
    a = min_val
    bins = list()

    for i in range(n+1):
        bins.append(a)
        a += step

    _ = plt.hist(all_scores, bins=bins, edgecolor='black')  # arguments are passed to np.histogram
    plt.title("Scores Frequencies")
    plt.xlabel("Score")
    plt.ylabel("Frequency")
    plt.show()


if __name__ == '__main__':
    # calculate_stats(basedir="/media/bassel/Career/toronto_content_selection/toronto")
    # plot_scores_hist(basedir="/media/bassel/Career/toronto_content_selection/toronto")
    # calculate_vehicles_perceived_more_than_one_AV(basedir="/media/bassel/Career/toronto_content_selection/toronto")
    # basedir="/media/bassel/Career/toronto_content_selection/toronto"
    basedir="/media/bassel/Career/toronto_content_selection/toronto_dense"
    # basedir="/media/bassel/Career/toronto_content_selection/toronto_dense (speed 40)"
    # basedir="/media/bassel/Career/toronto_content_selection/toronto_more_buses"
    calculate_stats(basedir)
    calculate_vehicles_perceived_more_than_one_AV(basedir)
    # plot_scores_hist(basedir)