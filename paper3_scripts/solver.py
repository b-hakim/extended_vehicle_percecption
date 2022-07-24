import time

from ortools.linear_solver import pywraplp
import numpy as np
from scipy.optimize import minimize


class Solver:
    def __init__(self, num_players, payoff_matrix, ego_player_idx):
        # Create the two variables and let them take on any non-negative value.
        self.num_players = num_players
        self.payoff_matrix = payoff_matrix
        self.ego_player_idx = ego_player_idx

    def get_payoff_value(self, players_actions):
        obj = self.payoff_matrix

        for a in players_actions:
            obj = obj[a]

        return obj.tolist()

    def get_all_scores(self, actions=[]):
        if len(actions) == self.num_players:
            return self.get_payoff_value(actions)

        lst_scores = []

        for action in range(2):
            l = self.get_all_scores(actions + [action])
            lst_scores += l

        return lst_scores

    def find_probabilities_old(self):
        solver = pywraplp.Solver.CreateSolver('GLOP')
        probs = [0]*self.num_players

        for i in range(self.num_players):
            probs[i] = solver.NumVar(0, 1, f'p_{i}')

        all_scores = np.array(self.get_all_scores([])).reshape((-1, self.num_players)).tolist()

        for eq_i in range(self.num_players):
            eq = 0
            for score_idx, score in enumerate(all_scores):
                bits = self.int2bits(score_idx, self.num_players)
                term = score[eq_i]
                for bit_idx, b in enumerate(bits):
                    if bit_idx == eq_i:
                        if b == 1:
                            term = term * (-1)
                    else:
                        if b == 0:
                            try:
                                term = term * probs[bit_idx]
                            except:
                                xc = 0
                        else:
                            term = term * (1-probs[bit_idx])
                eq += term

            solver.Add(eq == 0)

        # solver.Add(sum(probs) - 1 == 0)

        sum_other_players_prob = 0

        for i in range(self.num_players):
            # if i == self.ego_player_idx:
            #     sum_other_players_prob += probs[i]
            # else:
            sum_other_players_prob += probs[i]

        solver.Maximize(sum_other_players_prob)
        status = solver.Solve()

        # if status == pywraplp.Solver.OPTIMAL:
        #     print('Solution:')
        #     print('Objective value =', solver.Objective().Value())
        #     for i, p in enumerate(probs):
        #         print(f'p_{i} =', p.solution_value())
        # else:
        #     print('The problem does not have an optimal solution.')
        #     print('Solution:')
        #     print('Objective value =', solver.Objective().Value())
        #     for i, p in enumerate(probs):
        #         print(f'p_{i} =', p.solution_value())

        # print('\nAdvanced usage:')
        # print('Problem solved in %f milliseconds' % solver.wall_time())
        # print('Problem solved in %d iterations' % solver.iterations())
        return [p.solution_value() for p in probs]

    def find_probabilities(self):
        def equations(probs):
            eqs = []
            all_scores = np.array(self.get_all_scores([])).reshape((-1, self.num_players)).tolist()

            for eq_i in range(self.num_players):
                eq = 0
                for score_idx, score in enumerate(all_scores):
                    bits = self.int2bits(score_idx, self.num_players)
                    term = score[eq_i]
                    for bit_idx, b in enumerate(bits):
                        if bit_idx == eq_i:
                            if b == 1:
                                term = term * (-1)
                        else:
                            if b == 0:
                                try:
                                    term = term * probs[bit_idx]
                                except:
                                    xc = 0
                            else:
                                term = term * (1 - probs[bit_idx])
                    eq += term

                eqs.append(eq)

            return np.dot(eqs,eqs)

        # def constraints(probs):
        #     l = []
        #     for p in probs:
        #         f.append()

        bnds = []

        for i in range(self.num_players):
            bnds.append((0, 1))

        propabilities_values = minimize(equations, (1,)*self.num_players, method='SLSQP', bounds=tuple(bnds))

        return propabilities_values.x
        # probs = [0] * self.num_players
        #
        # for i in range(self.num_players):
        #     probs[i] = solver.NumVar(0, 1, f'p_{i}')
        #
        # all_scores = np.array(self.get_all_scores([])).reshape((-1, self.num_players)).tolist()
        #
        # for eq_i in range(self.num_players):
        #     eq = 0
        #     for score_idx, score in enumerate(all_scores):
        #         bits = self.int2bits(score_idx, self.num_players)
        #         term = score[eq_i]
        #         for bit_idx, b in enumerate(bits):
        #             if bit_idx == eq_i:
        #                 if b == 1:
        #                     term = term * (-1)
        #             else:
        #                 if b == 0:
        #                     try:
        #                         term = term * probs[bit_idx]
        #                     except:
        #                         xc = 0
        #                 else:
        #                     term = term * (1 - probs[bit_idx])
        #         eq += term
        #
        #     solver.Add(eq == 0)
        #
        # sum_other_players_prob = 0
        #
        # for i in range(self.num_players):
        #     # if i == self.ego_player_idx:
        #     #     sum_other_players_prob += probs[i]
        #     # else:
        #     sum_other_players_prob += probs[i]
        #
        # solver.Maximize(sum_other_players_prob)
        # status = solver.Solve()

        # if status == pywraplp.Solver.OPTIMAL:
        #     print('Solution:')
        #     print('Objective value =', solver.Objective().Value())
        #     for i, p in enumerate(probs):
        #         print(f'p_{i} =', p.solution_value())
        # else:
        #     print('The problem does not have an optimal solution.')
        #     print('Solution:')
        #     print('Objective value =', solver.Objective().Value())
        #     for i, p in enumerate(probs):
        #         print(f'p_{i} =', p.solution_value())

        # print('\nAdvanced usage:')
        # print('Problem solved in %f milliseconds' % solver.wall_time())
        # print('Problem solved in %d iterations' % solver.iterations())
        # return [p.solution_value() for p in probs]

    def val_in_mask(self, mask: int, num_bits: int, val: int) -> int:
        val_freq = 0

        for i in range(num_bits):
            if (mask & 1) == val:
                val_freq += 1
            mask = mask >> 1

        return val_freq

    def int2bits(self, mask: int, num_bits: int) -> list:
        ret = []

        while mask != 0:
            ret.append(mask&1)
            mask = mask >> 1

        while len(ret) < num_bits:
            ret.append(0)

        ret.reverse()

        return ret