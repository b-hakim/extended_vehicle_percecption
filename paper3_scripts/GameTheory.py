import numpy as np
from scipy.optimize import fsolve


class GameTheory:
    def __init__(self, payoff_matrix:np.ndarray):
        self.num_players = payoff_matrix.shape[-1]
        self.strategies = 2
        self.payoff = payoff_matrix

    def dominant_solutions(self):
        player_dominant_strategies = {}

        for p in range(self.num_players):
            self.player = p
            action, dominant = self.__find_dominant_strategy([])

            if dominant:
                player_dominant_strategies[p] = action
            else:
                player_dominant_strategies[p] = None

        return player_dominant_strategies

    def get_payoff_value(self, players_actions):
        obj = self.payoff

        for a in players_actions:
            obj = obj[a]

        return obj.tolist()
    # if no dominant --> check for dominated
    # if no dominated and no dominant then we r done
    # otherwsie repeat checking for dominated and dominated
    # if found any (not the current vehicle), then remove that player's action and repeat

    def __find_dominant_strategy(self, actions:list=[]):
        # take the max --> dominant
        # take the min --> dominated

        if len(actions) == self.num_players:
            a1 = self.get_payoff_value(actions)[self.player]
            actions[self.player] = 1
            a2 = self.get_payoff_value(actions)[self.player]
            return (0, True) if a1 > a2 else (1, True)

        action_taken = None

        # dominanet if the self.player has one same action for all players
        for p2_action in range(self.strategies):
            if self.player == len(actions):
                action, dominant = self.__find_dominant_strategy(actions + [0])
                if dominant is False:
                    return None, False

                if action_taken is None:
                    action_taken = action
                elif action_taken != action:
                    return None, False
                break
            else:
                action, dominant = self.__find_dominant_strategy(actions + [p2_action])

            if dominant is False:
                return None, False

            if action_taken is None:
                action_taken = action
            elif action_taken != action:
                return None, False

        return action_taken, True

    def nash_equilibrium_solutions(self):
        all_cells = np.array(self.get_all_scores([])).reshape((-1, self.num_players)).tolist()
        all_ne = []

        for i, cell_score in enumerate(all_cells):
            is_ne = True
            for p in range(self.num_players):
                for a in range(self.strategies):
                    player_action = (i & (1<<a))>>a

                    if player_action == 1:
                        other_action = i & ~(1 << a)
                    else:
                        other_action = i | (1 << a)

                    is_ne = all_cells[other_action][p] <= cell_score[p]

                    if not is_ne:
                        break

            if is_ne:
                all_ne.append((i, cell_score))

        return all_ne

    def mask_to_array(self, mask: int, num_bits: int) -> list:
        ret = []

        while mask != 0:
            ret.append(mask&1)
            mask = mask >> 1

        while len(ret) < num_bits:
            ret.append(0)

        ret.reverse()

        return ret

    def val_in_mask(self, mask: int, num_bits: int, val: int) -> int:
        val_freq = 0

        for i in range(num_bits):
            if (mask & 1) == val:
                val_freq += 1
            mask = mask >> 1

        return val_freq

    # def mixed_strategy_solution(self):
    #     self.propabilities = (1,) * self.num_players
    #     # loop all cells in the matrix and multiply each by the corresponding prob: p if action == 0
    #     #                                                                     and (-1) if action is 1
    #     # then sum all
    #
    #     all_scores = np.array(self.get_all_scores([])).reshape((-1, self.num_players)).tolist()
    #
    #     def equations(propabilities):
    #         lst_eq = []
    #
    #         for player_id in range(self.num_players):
    #             eq = 0
    #             for i, score in enumerate(all_scores):
    #                 v = score[player_id]
    #                 player_actions = self.mask_to_array(i, self.num_players)
    #                 term = v
    #
    #                 for j, a in enumerate(player_actions):
    #                     if j == player_id:
    #                         if a == 0:
    #                             continue
    #                         else:
    #                             term *= -1
    #                     else:
    #                         if a == 0:
    #                             term *= propabilities[j]
    #                         else:
    #                             term *= (1 - propabilities[j])
    #                 eq += term
    #
    #             lst_eq.append(eq)
    #
    #         return tuple(lst_eq)
    #
    #     propabilities_values = fsolve(equations, self.propabilities)
    #     return propabilities_values

    def mixed_strategy_solution_old(self):
        def equations(propabilities):
            lst_eq = []
            for i in range(self.num_players):
                pos = self.get_half_mixed_strategy_sol_all_prob((i, 0), propabilities)
                neg = self.get_half_mixed_strategy_sol_all_prob((i, 1), propabilities)
                lst_eq.append(pos - neg)
            return lst_eq

        propabilities_values = fsolve(equations, (1,)*self.num_players, factor=10000)

        return propabilities_values

    def get_half_mixed_strategy_sol_all_prob(self, player_action, probs):
        # try:
        all_player_scores = np.array(self.get_all_scores_based_on_player_action([], player_action))\
                                                              .reshape((-1, 2, self.num_players)).tolist()
        # except:
        #     x = 0

        sum = 0

        for score in all_player_scores:
            # players_actions = self.mask_to_array(i, self.num_players)
            score , players_actions = score
            term = score[player_action[0]]

            for player_idx, a in enumerate(players_actions):
                if player_action[0] == player_idx:
                    continue

                if a == 0:
                    term *= probs[player_idx]
                else:
                    term *= 1 - probs[player_idx]

            sum += term

        return sum

    def mixed_strategy_solution(self):
        def equations(propabilities):
            pos = self.get_half_mixed_strategy_sol(0, propabilities[0])
            neg = self.get_half_mixed_strategy_sol(1, propabilities[0])
            return pos - neg

        propabilities_values = fsolve(equations, (1,))

        return propabilities_values

    def get_half_mixed_strategy_sol(self, starting_action, prob):
        all_scores = np.array(self.get_all_scores([starting_action])).reshape((-1, self.num_players)).tolist()

        sum = 0

        for i, score in enumerate(all_scores):
            s = score[0] # first player score
            zeros = self.val_in_mask(i, self.num_players, 0)
            ones = self.val_in_mask(i, self.num_players, 1)
            term = pow(prob, zeros) * pow((1-prob), ones)
            sum += s * term

        return sum

    def get_all_action_combinations(self, actions:list):
        if len(actions) == self.num_players:
            return actions

        lst = []

        for action in range(self.strategies):
            lst.append(self.get_all_action_combinations(actions + [action]))

            return lst

    def get_all_scores_based_on_player_action(self, actions, player_action):
        if len(actions) == self.num_players:
            return self.get_payoff_value(actions) + actions

        if len(actions) == player_action[0]:
            return self.get_all_scores_based_on_player_action(actions + [player_action[1]], player_action)

        lst_scores = []

        for action in range(0, self.strategies):
            l = self.get_all_scores_based_on_player_action(actions + [action], player_action)
            lst_scores += l

        return lst_scores

    def get_all_scores(self, actions=[]):
        if len(actions) == self.num_players:
            return self.get_payoff_value(actions)

        lst_scores = []

        for action in range(0, self.strategies):
            l = self.get_all_scores(actions + [action])
            lst_scores += l

        return lst_scores


if __name__ == '__main__':
    g = GameTheory(np.zeros((2,2,2)))
    a = g.mask_to_array(2,3)
    print(a)