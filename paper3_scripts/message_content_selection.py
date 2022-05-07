import multiprocessing
import os
import pickle
import time
from ast import literal_eval

import numpy as np
from paper3_scripts.GameTheory import GameTheory
from paper3_scripts.results import Stats

"""
todo: Do not send messages about objects that are being sent by another vehicle
tasks: 
1- for each message, check other vehicles perceiving it.
2- generate payoff table and fill missing values based on the map probability distribution
3- based on the solution (nash or not) if one NE then do it, if not then do the mixed solution
4- repeat for all vehicles
5- find the duplicate submitted messages, aka received by basestation
6- Find missed objects not sent to bs
7- repeat considering occlusion
"""
# use_seed = True
# if use_seed:
#     with open("./npstate.npy") as fr:
#         np_rand_state = (
#         fr.readline().strip(), np.array(literal_eval(fr.readline().strip())), int(fr.readline().strip()),
#         int(fr.readline().strip()), float(fr.readline().strip()))
#
#     np.random.set_state(np_rand_state)
# #######################################################################################################
# Save Rand State
# with open("./npstate.npy", 'w') as fw:
#     fw.write(str(np.random.get_state()[0]).replace("\n", "") + "\n")
#     fw.write(str(np.random.get_state()[1].tolist()).replace("\n", "") + "\n")
#     fw.write(str(np.random.get_state()[2]).replace("\n", "") + "\n")
#     fw.write(str(np.random.get_state()[3]).replace("\n", "") + "\n")
#     fw.write(str(np.random.get_state()[4]).replace("\n", "") + "\n")

class MessageContentSelection():
    def __init__(self, basedir):
        self.basedir = basedir
        state_path = os.path.join(basedir, "saved_state", "cv2x_non_buildings.pkl")

        with open(state_path, 'rb') as fw:
            self.cv2x_vehicles, self.non_cv2x_vehicles, self.buildings, \
            self.av_perceiving_nav, self.scores_per_cv2x, los_statuses = pickle.load(fw)

        self.map_scores_distributions = np.load(os.path.join(basedir,"../map_scores_distribution.npy"))
        self.stats = Stats()

    def player_sends_obj(self, sender_av, action=0):
        if action == 0:
            ret = [list(self.scores_per_cv2x[sender_av][0]), list(self.scores_per_cv2x[sender_av][0])]
        else:
            ret = [list(self.scores_per_cv2x[sender_av][1]), list(self.scores_per_cv2x[sender_av][0])]

        ret[0][2] = ret[0][2].vehicle_id
        ret[1][2] = ret[1][2].vehicle_id
        return tuple(ret)

    @staticmethod
    def adjust_scores(scores):
        for i in range(len(scores)):
            scores[i] = list(scores[i])
            if scores[i][1] > 1:
                scores[i][1] = 0

        return scores

    def run(self):
        # self.test_payoff()
        # return
        # 1- get potential senders
        all_potential_senders = {}

        for sender_av, scores in self.scores_per_cv2x.items():
            scores = MessageContentSelection.adjust_scores(scores)
            scores = sorted(scores, key=lambda x: x[1], reverse=True)
            self.scores_per_cv2x[sender_av] = scores

            score = scores[0]  # max

            if score[1] == 0:
                continue

            # find AV vehicles that are 'seeing' this NAV vehicle
            potential_senders = []

            for sender2_av, sender2_perceived_navs in self.av_perceiving_nav.items():
                if sender2_av == sender_av:
                    continue

                if score[2].vehicle_id in sender2_perceived_navs:
                    potential_senders.append(sender2_av)

            all_potential_senders[sender_av] = potential_senders

        ## 2- Get decision for each sender
        # 2.1 Fill the payoff matrix based on the number of players
        #   2.1.1 Get the distribution of all players
        #   2.1.1 Draw a number for the max score to check the actual participating player's
        #   2.1.2 Fill the payoff matrix
        # 2.2 Solve the game
        # 2.3 Fill the decision dictionary on which object to be sent

        decision_object_to_send = {} # d[sender_av] = (score_obj_sent, original_score_obj)
        used_gt_approach = {"dominant" : 0, "dominant - other" : 0, "ms" : 0}

        vmax_mean, vmax_std = self.map_scores_distributions[0]
        v2max_mean, v2max_std = self.map_scores_distributions[1]
        dominant_approach = 0
        ne_approach = 0
        ms_approach = 0

        for sender_av, potential_senders in all_potential_senders.items():
            n = len(potential_senders)
            used_gt_approach[sender_av] = "no"

            self.players_scores = [[self.scores_per_cv2x[sender_av][0], self.scores_per_cv2x[sender_av][1]]]

            # in case v1 and v2 are equal, then make a difference!
            if self.scores_per_cv2x[sender_av][0][1] == self.scores_per_cv2x[sender_av][1][1]:
                self.players_scores[0][1] = list(self.players_scores[0][1])
                self.players_scores[0][1][1] -= self.players_scores[0][1][1] * 0.001
                self.players_scores[0][1] = tuple(self.players_scores[0][1])

            for i in range(n):
                v1_dash = np.random.normal(loc=vmax_mean, scale=vmax_std)

                if v1_dash > self.scores_per_cv2x[sender_av][0][1] + 0.001:
                    n -= 1
                else:
                    v2_dash = -1

                    # while self.scores_per_cv2x[sender_av][0][1] <= v2_dash or v2_dash < 0:
                    #     v2_dash = np.random.normal(loc=v2max_mean, scale=v2max_std)
                    v2_dash = self.scores_per_cv2x[sender_av][1][1]

                    if False: # occlusion
                        self.players_scores.append([self.scores_per_cv2x[sender_av][0], v2_dash])*0.5 # incorrect: need to multiply the score itself
                    else:
                        self.players_scores.append([self.scores_per_cv2x[sender_av][0], (None, v2_dash, None)])

            if n == 0:
                # all other potential players will probably send smthg else, solution is to just send this object to bs.
                decision_object_to_send[sender_av] = self.player_sends_obj(sender_av, 0)
            else:
                # Game Theory Approach!
                n += 1
                shape = n * [2] + [n]
                self.payoff = np.zeros(shape)
                self.fill_payoff_matrix([])
                g = GameTheory(self.payoff)
                d = g.dominant_solutions()
                is_dominant = False

                if d[0] is not None:
                    is_dominant = True
                    used_gt_approach[sender_av] = "dominant"
                    used_gt_approach["dominant"] += 1
                    # dominant_approach += 1
                    decision_object_to_send[sender_av] = self.player_sends_obj(sender_av, d[0])
                else:
                    for p, action in d.items():
                        if action is not None:
                            is_dominant = True
                            decision_object_to_send[sender_av] = self.player_sends_obj(sender_av, 1-action)
                            used_gt_approach[sender_av] = "dominant - other"
                            used_gt_approach["dominant - other"] += 1
                            break

                if not is_dominant:
                    # print("NE Solution:")
                    ne = g.nash_equilibrium_solutions()
                    # print(ne)

                    if len(ne) == 1:
                        cell_idx = ne[0][0]
                        p = n-1
                        player_action = (cell_idx & (1 << p)) >> p
                        decision_object_to_send[sender_av] = self.player_sends_obj(sender_av, player_action)
                        ne_approach += 1
                    else:
                        ms = g.mixed_strategy_solution()
                        used_gt_approach[sender_av] = "ms"
                        used_gt_approach["ms"] += 1
                        ma = ms[0]
                        for m in ms:
                            if m != ma:
                                print("ne")
                            if m<0 or m>1:
                                if g.num_players < 4:
                                    print(self.basedir)

                        # assert ms[0] < 1
                        r = np.random.random()
                        decision_object_to_send[sender_av] = self.player_sends_obj(sender_av, 0 if r < ms[0] else 1)

        # for sender, decision in decision_object_to_send.items():
        #     print(f"Sender ID: {sender}, Receiver1 ID: {decision[0][0]}, Object ID: {decision[0][2]}, Score: {np.round(decision[0][1], 2)},"
        #           f" ::Original:: Receiver1 ID: {decision[1][0]}, Object ID: {decision[1][2]}, Score: {np.round(decision[1][1], 2)}")

        # self.stats.approaches_counts = [dominant_approach, ms_approach]

        self.evaluate_decision(decision_object_to_send, used_gt_approach)

        stats_path = os.path.join(self.basedir, "stats.pkl")

        with open(stats_path, 'wb') as fw:
            pickle.dump(self.stats, fw)

        # self.stats.print()

    def evaluate_decision(self, decision_object_to_send, used_gt_approach):
        gt_receiver_obj_dict = {}
        self.stats.approaches_counts = [used_gt_approach["dominant"],
                                        used_gt_approach["dominant - other"],
                                        used_gt_approach["ms"]]

        for sender, decision in decision_object_to_send.items():
            if decision[0][1] == 0:
            # if decision[0][1] == 0 or used_gt_approach[sender] == "no":
                continue

            id = decision[0][0]+"_"+decision[0][2]

            if id in gt_receiver_obj_dict:
                self.stats.gt_number_duplicate += 1
                self.stats.gt_total_duplicate_value += decision[0][1]
                gt_receiver_obj_dict[id] += 1
            else:
                gt_receiver_obj_dict[id] = 1
                self.stats.gt_total_sent_value += decision[0][1]

        self.stats.gt_number_sent_unique = len(gt_receiver_obj_dict.keys())

        max_receiver_obj_dict = {}
        gt_objs_sent = list(gt_receiver_obj_dict.keys())

        for sender, decision in self.scores_per_cv2x.items():
            if decision[0][1] == 0:
            # if decision[0][1] == 0 or used_gt_approach[sender] == "no":
                continue

            id = decision[0][0] + "_" + decision[0][2].vehicle_id

            if id not in gt_objs_sent:
                self.stats.gt_num_not_sent_msgs += 1
                self.stats.gt_total_not_sent_value += decision[0][1]

            if id in max_receiver_obj_dict:
                self.stats.max_number_duplicate += 1
                self.stats.max_total_duplicate_value += decision[0][1]
                max_receiver_obj_dict[id] += 1
            else:
                max_receiver_obj_dict[id] = 1
                self.stats.max_total_sent_value += decision[0][1]

        self.stats.max_number_sent_unique = len(max_receiver_obj_dict.keys())

        # How many used NE, dominant, or Mixed
        # self.stats.print()

    def test_payoff(self):
        #region Game 1

        self.players_scores = [[[None, 55, None], [None, 50, None]],
                               [[None, 55, None], [None, 50, None]]]
        self.payoff = np.zeros((2, 2, 2))
        self.fill_payoff_matrix([])

        g = GameTheory(self.payoff)
        # print(g.payoff)
        # print("Dominant Solution:")
        #
        # d = g.dominant_solutions()
        # print(d)
        #
        # print("NE Solution:")
        # ne = g.nash_equilibrium_solutions()
        # print(ne)
        #
        print("MS Solution:")
        ms = g.mixed_strategy_solution()
        print(ms)
        #endregion

        #region Game 2
        # self.payoff = np.array([[[[7, 7, 7], [11, 11, 14]], [[11, 10, 11], [22, 10, 14]]],
        #                         [[[22, 11, 11], [22, 22, 14]], [[22, 10, 22], [14, 2, 7]]]])
        #
        # g = GameTheory(self.payoff)
        # print(g.payoff)
        # print("Dominant Solution:")
        #
        # d = g.dominant_solutions()
        # print(d)
        #
        # print("NE Solution:")
        # ne = g.nash_equilibrium_solutions()
        # print(ne)

        print("MS Solution:")
        ms = g.mixed_strategy_solution()
        print(ms)
        #endrgion

    def fill_payoff_matrix(self, actions):
        num_players = self.payoff.shape[-1]

        if len(actions) == num_players:
            scores = []

            # count number of senders
            num_senders = 0

            for action in actions:
                if action == 0: # action is to send
                    num_senders += 1

            # if no player sends --> use s2 for all players and - s/n
            if num_senders == 0:
                for player_idx, action in enumerate(actions):
                    scores.append(self.players_scores[player_idx][1][1] - self.players_scores[player_idx][0][1]/num_players)

            # if all players send --> divide the score among them
            elif num_senders == num_players:
                for player_idx, action in enumerate(actions):
                    # if True or player_idx == 0:
                    #     v1 = self.players_scores[player_idx][0][1]
                    #     v2 = self.players_scores[player_idx][1][1]
                    #     scores.append(self.players_scores[player_idx][action][1] * (0.5*v2/v1))
                    # else:
                    #     scores.append(0)
                    #
                    scores.append(self.players_scores[player_idx][action][1]/num_players)

            # if one player sends all do not send, take s for that player but s2 for others
            elif num_senders == 1:
                for player_idx, action in enumerate(actions):
                    scores.append(self.players_scores[player_idx][action][1])

            # if more than one player sends (but not all) then those who send are punished
            elif num_senders > 1:
                for player_idx, action in enumerate(actions):
                    if action == 0:
                        scores.append(self.players_scores[player_idx][action][1]/num_senders)
                    else:
                        scores.append(self.players_scores[player_idx][action][1])

            obj = self.payoff

            for a in actions:
                obj = obj[a]

            obj[:] = scores

            return

        # for player in range(len(actions), num_players):
        for action in range(2):
            self.fill_payoff_matrix(actions+[action])

    def fill_payoff_matrix_new(self, actions):
        num_players = self.payoff.shape[-1]

        if len(actions) == num_players:
            scores = []

            # count number of senders
            num_senders = 0

            for action in actions:
                if action == 0: # action is to send
                    num_senders += 1

            # if no player sends --> use s2 for all players and - s/n
            if num_senders == 0:
                for player_idx, action in enumerate(actions):
                    scores.append(self.players_scores[player_idx][1][1] - self.players_scores[player_idx][0][1]/num_players)

            # if all players send --> divide the score among them
            elif num_senders == num_players:
                for player_idx, action in enumerate(actions):
                    scores.append(self.players_scores[player_idx][action][1]/num_players)

            # if one player sends all do not send, take s for that player but s2 for others
            elif num_senders == 1:
                for player_idx, action in enumerate(actions):
                    scores.append(self.players_scores[player_idx][action][1])

            # if more than one player sends (but not all) then those who send are punished
            elif num_senders > 1:
                for player_idx, action in enumerate(actions):
                    if action == 0:
                        scores.append(self.players_scores[player_idx][action][1]/num_senders)
                    else:
                        scores.append(self.players_scores[player_idx][action][1])

            obj = self.payoff

            for a in actions:
                obj = obj[a]

            obj[:] = scores

            return
        # for player in range(len(actions), num_players):
        for action in range(2):
            self.fill_payoff_matrix_new(actions+[action])


class MCSThread(multiprocessing.Process):
    def __init__(self, lst_paths, process_id):
        super(MCSThread, self).__init__()
        self.lst_paths = lst_paths
        self.process_id = process_id

    def run(self):
        for path in self.lst_paths:
            avg_stats = Stats()
            N = 10

            for i in range(N):
                # if self.process_id == 6:# and i%10 == 0:
                #     print(path)

                mcs = MessageContentSelection(path)
                mcs.run()
                avg_stats += mcs.stats

            avg_stats /= N
            stats_path = os.path.join(path, "stats.pkl")

            with open(stats_path, 'wb') as fw:
                pickle.dump(avg_stats, fw)

        print(f"Process {self.process_id} finished!")


def run_simulation(path):
    paths = []

    for i in range(3):
        for j in range(100):
            paths.append(f'{path}/toronto_{i}/{j}')

    p = 12

    block_size = len(paths)//p
    thread_pool = []


    for i in range(p):
        start = block_size*i
        end = start + block_size

        if i == p - 1:
            end = len(paths)

        mcs = MCSThread(paths[start:end], i)
        mcs.start()
        thread_pool.append(mcs)

    for p in thread_pool:
        p.join()

    print("threads done calculations. . . aggregating!")
    tot_stats = Stats()

    for i in range(3):
        for j in range(100):

            with open(f'{path}/toronto_{i}/{j}/stats.pkl', "rb") as fr:
                stats = pickle.load(fr)
                tot_stats += stats

    tot_stats /= 300
    tot_stats.print()


if __name__ == '__main__':
    print('/media/bassel/Career/toronto_content_selection/toronto')
    run_simulation('/media/bassel/Career/toronto_content_selection/toronto')

    print("\n***********************************************************************\n")

    # print('/media/bassel/Career/toronto_content_selection/toronto_more_buses')
    # run_simulation('/media/bassel/Career/toronto_content_selection/toronto_more_buses')

    print("\n***********************************************************************\n")

    # print('/media/bassel/Career/toronto_content_selection/toronto_dense')
    # run_simulation('/media/bassel/Career/toronto_content_selection/toronto_dense')
    # run_simulation('/media/bassel/Career/toronto_content_selection/toronto_dense (speed 40)')

    # run_simulation('/media/bassel/Career/toronto_content_selection/toronto_dense')
    # s = time.time()
    # msg = MessageContentSelection('/media/bassel/Career/toronto_content_selection/toronto/toronto_1/3')
    # msg.run()
    # print("time taken:", time.time()-s)