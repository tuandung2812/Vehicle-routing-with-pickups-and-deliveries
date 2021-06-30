from __future__ import print_function
from ortools.linear_solver import pywraplp
import numpy as np
import time


# File reading
def input_reading(file):
    with open(file, 'r') as f:
        first_line = f.readline().split()
        N, M, K = int(first_line[0]), int(first_line[1]), int(first_line[2])
        second_line = f.readline().split()
        q = []
        for ele in second_line:
            q.append(int(ele))
        third_line = f.readline().split()
        capacity = []
        for ele in third_line:
            capacity.append(int(ele))
        d = np.array([[int(num) for num in line.split()] for line in f])
    return [N, M, K, q, capacity, d]


i = input_reading('test3')
N = i[0]
M = i[1]
K = i[2]
q = [0 for i in range(N)] + i[3] + [0 for i in range(N)] + [(-ele) for ele in i[3]]
capacity = i[4]
d = i[5]
person = [1 for i in range(N)] + [0 for i in range(N + 1, N + M + 1)] + [-1 for i in
                                                                         range(N + M + 1, 2 * N + M + 1)] + [0 for i in
                                                                                                             range(
                                                                                                                 2 * N + M + 1,
                                                                                                                 2 * N + 2 * M + 1)]


def MIP():
    global N, M, K, q, capacity, d
    solver = pywraplp.Solver.CreateSolver('SCIP')
    infinity = solver.infinity()
    # initialize variables

    x = [[[0 for i in range(2 * N + 2 * M + 2)] for i in range(2 * N + 2 * M + 2)] for k in range(K + 1)]

    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            for j in range(2 * N + 2 * M + 1):
                x[k][i][j] = solver.IntVar(0, 1, 'x[' + str(k) + '][' + str(i) + ']' + '[' + str(j) + ']')

    # u[k][i] denotes the order of point i in vehicle k's route,if k doesn't go through i than u[k][i] = 0
    u = [[0 for i in range(2 * N + 2 * M + 2)] for k in range(K + 1)]
    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            u[k][i] = solver.IntVar(0, infinity, 'u[' + str(k) + '][' + str(i) + ']')

    # w[k][i] denotes the weight in vehicle k after passing point i
    w = [[0 for i in range(2 * N + 2 * M + 2)] for k in range(K + 1)]
    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            w[k][i] = solver.IntVar(0, infinity, 'u[' + str(k) + '][' + str(i) + ']')

    # p[k][i] denotes the number of people in vehicle k after passing point i (either 0 or 1)
    p = [[0 for i in range(2 * N + 2 * M + 2)] for k in range(K + 1)]
    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            p[k][i] = solver.IntVar(0, 1, 'p[' + str(k) + '][' + str(i) + ']')

    x[0] = 0
    u[0] = 0
    w[0] = 0
    p[0] = 0

    # Exactly 1 vehicle goes in
    for j in range(1, 2 * N + 2 * M + 1):
        c = 0
        for k in range(1, K + 1):
            for i in range(2 * N + 2 * M + 1):
                if i != j:
                    c += x[k][i][j]
        solver.Add(c == 1)
    # Exactly 1 vehicle goes out
    for i in range(1, 2 * N + 2 * M + 1):
        c = 0
        for k in range(1, K + 1):
            for j in range(2 * N + 2 * M + 1):
                if i != j:
                    c += x[k][i][j]
        solver.Add(c == 1)

    # x[k][h][h] = 0
    for k in range(1, K + 1):
        for h in range(2 * N + 2 * M + 1):
            solver.Add(x[k][h][h] == 0)

    # If vehicle k passes through h it has to go into h
    for k in range(1, K + 1):
        for h in range(2 * N + 2 * M + 1):
            c1 = 0
            for i in range(2 * N + 2 * M + 1):
                c1 += x[k][i][h]
            c2 = 0
            for j in range(2 * N + 2 * M + 1):
                c2 += x[k][h][j]
            solver.Add(c1 - c2 == 0)
    # Vehicle k has to start at point 0
    for k in range(1, K + 1):
        c = 0
        for j in range(1, N + M + 1):
            c += x[k][0][j]
        solver.Add(c == 1)
    # Vehicle k has to end at point 0
    for k in range(1, K + 1):
        c = 0
        for i in range(N + M + 1, 2 * N + 2 * M + 1):
            c += x[k][i][0]
        solver.Add(c == 1)

    # If vehicle k passes through point i (the pick up point), it has to go through point (i+N+M) (the corresponding delivery point)
    for k in range(1, K + 1):
        for j in range(1, N + M + 1):
            c1 = 0
            for i in range(2 * N + 2 * M + 1):
                c1 += x[k][i][j] - x[k][i][j + N + M]
            solver.Add(c1 == 0)
    # all vehicles must start at point 0,so the order at point 0 with any vehicle k is u[k][i] = 0
    for k in range(1, K + 1):
        solver.Add(u[k][0] == 0)

    # u[k][j] = u[k][i] + 1 if x[k][i][j] = 1
    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            for j in range(1, 2 * N + 2 * M + 1):
                if i != j:
                    solver.Add(u[k][i] - u[k][j] + (2 * N + 2 * M + 1) * x[k][i][j] + (2 * N + 2 * M + 1 - 2) * x[k][j][
                        i] <= 2 * N + 2 * M + 1 - 1)

    # the vehicle must pass the pick up point i before passing the delivery point (i+N+M) (u[k][i] < u[k][i+N+M]
    for k in range(1, K + 1):
        for i in range(1, N + M + 1):
            solver.Add(u[k][i] - u[k][i + N + M] <= 0)
    # w[k][0] = 0 with every k
    for k in range(1, K + 1):
        solver.Add(w[k][0] == 0)
    # w[k][j] = w[k][i] + q[j] if x[k][i][j] == 1
    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            for j in range(1, 2 * N + 2 * M + 1):
                solver.Add(w[k][j] >= w[k][i] + q[j - 1] - 9999 * (1 - x[k][i][j]))
    """The constraint of weight(it has to be bigger than either 0,or q[i] if i is a package
            pick up point.And it has to be smaller than either capacity of vehicle k,or capacity[k] - qi
            if i is a package delivery point"""
    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            solver.Add(max(0, q[i - 1]) <= w[k][i] <= min(capacity[k - 1], capacity[k - 1] + q[i - 1]))

    # The number of person in vehicle k after passing point 0 is 0
    for k in range(1, K + 1):
        solver.Add(p[k][0] == 0)

    """p[k][j] = p[k][i] + person[j] if x[k][i][j] = 1,where
        person[j] = 1 if it's a person pick-up poiny
        person[j] = -1 if it's a person delivery point
        person[j] = 0 otherwise"""
    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            for j in range(1, 2 * N + 2 * M + 1):
                if i != j:
                    solver.Add(p[k][j] >= p[k][i] + person[j - 1] - 9999 * (1 - x[k][i][j]))

    # The same as the weight's constraint
    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            c = 0
            for j in range(1, 2 * N + 2 * M + 1):
                if i != j:
                    c += x[k][i][j]
            solver.Add(max(0, person[i - 1]) <= p[k][i] <= min(1, 1 + person[i - 1]))

    # The objective function
    c = 0
    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            for j in range(2 * N + 2 * M + 1):
                c += x[k][i][j] * d[i][j]
    solver.Minimize(c)

    status = solver.Solve()
    if status == pywraplp.Solver.OPTIMAL:
        def Print_Sol():
            for k in range(1, K + 1):
                print('Path of vehicle number ' + str(k) + ' is:')
                print(path[k][0],end='')
                print(' ==> ',end='')
                for value in path[k][1:len(path[k]) - 1]:
                    print(str(value), end=' ')
                    if 1 <= value <= N:
                        print('(person pick-up point)' + ' ==> ', end=' ')
                    if N + 1 <= value <= N + M:
                        print('(package pick-up point)' + ' ==> ', end='')
                    if N + M + 1 <= value <= 2 * N + M:
                        print('(person delivery point)' + ' ==> ' , end='')
                    if 2 * N + M + 1 <= value <= 2 * N + 2 * M:
                        print('(package delivery point)' + ' ==> ', end='')
                print(path[k][-1])
                print('The distance vehicle number ' + str(k) + ' travelled is: ', end=' ')
                print(find_distance(k))
                print()

        def find_distance(k):  # Hàm tìm xem xe thứ k đi tổng cộng được bao nhiêu
            global d
            distance = 0
            for i in range(len(path[k]) - 1):
                distance += d[path[k][i]][path[k][i + 1]]
            return distance

        path = {}
        for k in range(1, K + 1):
            path[k] = [0]
        print('Solution:')
        print('Objective value =', solver.Objective().Value())
        for k in range(1, K + 1):
            for j in range(1, 2 * N + 2 * M + 1):
                if x[k][0][j].solution_value() == 1:
                    path[k].append(j)
            while path[k][-1] != 0:
                for j in range(2 * N + 2 * M + 1):
                    if path[k][-1] != j:
                        if x[k][path[k][-1]][j].solution_value() == 1:
                            path[k].append(j)
                            if j == 0:
                                break
        Print_Sol()

    else:
        print('The problem does not have an optimal solution.')




if __name__ == '__main__':
    t0 = time.time()
    MIP()
    print(time.time() - t0)