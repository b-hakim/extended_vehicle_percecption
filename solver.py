import time

from ortools.linear_solver import pywraplp
import numpy as np


class Solver:
    def __init__(self, s_n, h_n_k, K, M):
        self.s_n = s_n
        self.h_n_k = h_n_k
        self.K = K
        self.M = M

    @staticmethod
    def power_dbm_to_mwatt(p_dbm):
        return 10**(p_dbm/10)

    def find_optimal_assignment(self, save_path=None):
        solver = pywraplp.Solver.CreateSolver('SCIP')

        # Decision Variables definitions
        alpha_n = {n:solver.IntVar(0.0, 1, 'alphas[%s]' % n) for n in self.s_n.keys()}

        eita_n_k = {}
        sum_scores = 0

        for n in self.s_n:
            tmp = []
            for k in range(self.K):
                tmp.append(solver.IntVar(0.0, 1, 'eita[%s, %i]' % (n, k)))
            eita_n_k[n] = tmp
            sum_scores += self.s_n[n][1]

        c_n = {}
        t=0.001
        # t=1
        P = Solver.power_dbm_to_mwatt(21)
        # print("P:", P)
        w = 180000
        n0 = Solver.power_dbm_to_mwatt(-174)
        # print("N0", n0)

        c_n_ks = {}

        for n in self.s_n:
            sum_n = 0

            for k in range(self.K):
                c_n_k = int( ( t * w * np.log2( 1 + (P * self.h_n_k[n][k])/(n0*w) ) ) )

                if n not in c_n_ks:
                    c_n_ks[n] = [c_n_k]
                else:
                    c_n_ks[n].append(c_n_k)

                sum_n += eita_n_k[n][k] * c_n_k

            c_n[n] = sum_n
        '''
        for all k, sum_n eita_n_k <= 1 
        '''

        for k in range(self.K):
            sum_n = 0
            for n in eita_n_k.keys():
                sum_n += eita_n_k[n][k]

            solver.Add(sum_n <= 1)

        for n in c_n.keys():
            solver.Add(c_n[n]-self.M*alpha_n[n] >= 0)

        for n in alpha_n.keys():
            a = alpha_n[n]
            solver.Add(self.K * a - solver.Sum(eita_n_k[n]) >= 0)

        tot_sum_eita_n_k = 0

        for n in self.s_n:
            for k in range(self.K):
                tot_sum_eita_n_k += eita_n_k[n][k]

        solver.Add(tot_sum_eita_n_k <= self.K)

        sum_objective = 0

        for n in self.s_n:
            x = n
            sum_objective += self.s_n[n][1] * alpha_n[n]

        solver.Maximize(sum_objective)

        start = time.time()
        status = solver.Solve()
        end = time.time()

        if save_path is not None:
            with open(save_path, 'w') as fw:
                fw.write('Number of variables = ' + str(solver.NumVariables())
                         + "\nNumber of constraints = " + str(solver.NumConstraints())
                         + "\nSum Requests scores = " + str(sum_scores)
                         + "\nLength Requests = " + str(len(list(self.s_n.keys())))
                         + "\nTime to solve: " + str (end - start) + "\n\n")

            if status != pywraplp.Solver.OPTIMAL:
                with open(save_path, 'a') as fw:
                    fw.write('The problem does not have an optimal solution.\n')

            selected_messages_requests = []

            with open(save_path, 'a') as fw:
                fw.write('Solution:'
                     + '\nObjective value = ' + str(solver.Objective().Value())
                     + "\n\nRBs = " + str(self.K) + "\n\n")

                s = ""
                for n in alpha_n.keys():
                    if s!="":
                        s+= " + "
                    s+= str(np.round(self.s_n[n][1], 2)) + "*" + str(abs(alpha_n[n].solution_value()))
                    if alpha_n[n].solution_value() == 1:
                        selected_messages_requests.append((self.s_n[n][0], [n, self.s_n[n][1], self.s_n[n][2].get_pos()]))

                fw.write('alpha: \n' + s + "\n\n")

                s = ""
                for n in c_n_ks.keys():
                    s += str(c_n_ks[n])
                    s += "\n"

                fw.write('c_n_k: \n' + s + "\n\n")

                fw.write('eita:\n')

                for n in eita_n_k.keys():
                    s = "["
                    for k in range(self.K):
                        s += str(abs(eita_n_k[n][k].solution_value())) + ", "
                    fw.write(s + "]\n")

            with open(save_path, 'a') as fw:
                fw.writelines('\nAdvanced usage:')
                fw.writelines(f'\nProblem solved in {solver.wall_time()} milliseconds' )
                fw.writelines(f'\nProblem solved in {solver.iterations()} iterations')
                fw.writelines(f'\nProblem solved in {solver.nodes()} branch-and-bound nodes\n\n')

        sent = np.count_nonzero([alpha_n[n].solution_value() for n in alpha_n.keys()])
        not_sent = len(list(alpha_n.keys())) - sent

        return sent, not_sent, selected_messages_requests

if __name__ == '__main__':
    N = 20 # number of vehicles
    M = 320 # message size in bits

    RBs = 100 # number of available resource blocks (i.e. frequencies)
    s_n = [1]*N
    c_n = [1]*N
    solver = Solver(s_n, c_n, RBs, M)
    res = solver.find_optimal_assignment()
    print(res)