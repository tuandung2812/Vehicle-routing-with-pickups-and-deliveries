import numpy as np
from ortools.sat.python import cp_model
import time
def input_reading(file):
    with open(file,'r') as f:
        first_line = f.readline().split()
        N , M , K = int(first_line[0]) , int(first_line[1]) , int(first_line[2])
        second_line = f.readline().split()
        q = []
        for ele in second_line:
            q.append(int(ele))
        third_line = f.readline().split()
        capacity = []
        for ele in third_line:
            capacity.append(int(ele))
        d = np.array([[int(num) for num in line.split()] for line in f])
    return [N,M,K,q,capacity,d]
i = input_reading('test3')
N = i[0]
M = i[1]
K = i[2]
q = [0 for i in range(N)] + i[3] + [0 for i in range(N)] + [(-ele) for ele in i[3]]
capacity = i[4]
d = i[5]
person = [1 for i in range(N)] + [0 for i in range(N+1,N+M+1)] + [-1 for i in range(N+M+1,2*N+M+1)] + [0 for i in range(2*N+M+1,2*N+2*M+1)]

def CP():
    global N, M, K, q, capacity, d
    model = cp_model.CpModel()

    # initialize variables

    x = [[[0 for i in range(2 * N + 2 * M + 2)] for i in range(2 * N + 2 * M + 2)] for k in range(K + 1)]

    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            for j in range(2 * N + 2 * M + 1):
                if i != j:
                    x[k][i][j] = model.NewIntVar(0, 1, 'x[' + str(k) + '][' + str(i) + ']' + '[' + str(j) + ']')

    u = [[0 for i in range(2 * N + 2 * M + 2)] for k in range(K + 1)]
    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            u[k][i] = model.NewIntVar(0, 2*N+2*M, 'u[' + str(k) + '][' + str(i) + ']')

    w = [[0 for i in range(2 * N + 2 * M + 2)] for k in range(K + 1)]
    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            w[k][i] = model.NewIntVar(0, capacity[k-1], 'u[' + str(k) + '][' + str(i) + ']')
    x[0] = 0
    u[0] = 0
    w[0] = 0
    # Exactly 1 vehicle goes in
    for j in range(1,2 * N + 2 * M + 1):
        c = 0
        for k in range(1, K + 1):
            for i in range(2 * N + 2 * M + 1):
                if i != j:
                    c += x[k][i][j]
        model.Add(c == 1)
    # Exactly 1 vehicle goes out
    for i in range(1,2 * N + 2 * M + 1):
        c = 0
        for k in range(1, K + 1):
            for j in range(2 * N + 2 * M + 1):
                if i != j:
                    c += x[k][i][j]
        model.Add(c == 1)


    # If vehicle k passes through h it has to go into h
    for k in range(1, K + 1):
        for h in range(1,2 * N + 2 * M + 1):
            c1 = 0
            for i in range(2 * N + 2 * M + 1):
                if i!= h:
                    c1 += x[k][i][h]
            c2 = 0
            for j in range(2 * N + 2 * M + 1):
                if j != h:
                    c2 += x[k][h][j]
            model.Add(c1 - c2 == 0)

    #All vehicles have to start at 0
    for k in range(1, K + 1):
        c = 0
        for j in range(1,N+M+1):
            c += x[k][0][j]
        model.Add(c == 1)
    #All vehicles have to end at 0
    for k in range(1, K + 1):
        c = 0
        for i in range(N+M+1,2 * N + 2 * M + 1):
            c += x[k][i][0]
        model.Add(c == 1)

    #If vehicle k goes through i(the pick up point),it has to go through i+N+M (the delivery point) later in its path
    for k in range(1, K + 1):
        for j in range(1, N + M + 1):
            c1 = 0
            for i in range(2 * N + 2 * M + 1):
                c1 += x[k][i][j] - x[k][i][j + N + M]
            model.Add(c1 == 0)

    #The order at point 0 is 0
    for k in range(1, K + 1):
        model.Add(u[k][0] == 0)

    #u[k][i] < u[k][i+N+M]
    for k in range(1,K+1):
        for k in range(1, K + 1):
            for i in range(1, N + M + 1):
                model.Add(u[k][i] - u[k][i + N + M]  < 0)
    # If x[k][i][j] == 1 then u[k][j] == u[k][i] + 1 (with j !=1 ,since we end at point 0)
    for k in range(1,K+1):
        for i in range(2*N+2*M+1):
            for j in range(1,2*N+2*M+1):
                if i != j:
                    b = model.NewBoolVar('b')
                    model.Add(x[k][i][j] == 1).OnlyEnforceIf(b)
                    model.Add(x[k][i][j] != 1).OnlyEnforceIf(b.Not())
                    model.Add(u[k][j] == u[k][i] + 1).OnlyEnforceIf(b)
    #The weight after passing point 0 has to be 0 for every k's
    for k in range(1, K + 1):
        model.Add(w[k][0] == 0)
    # If x[k][i][j] =1 then w[k][j] = w[k][i] + q[i]
    for k in range(1,K+1):
        for i in range(2*N+2*M+1):
            for j in range(1,2*N+2*M+1):
                if i !=j:
                    b = model.NewBoolVar('b')
                    model.Add(x[k][i][j] == 1).OnlyEnforceIf(b)
                    model.Add(x[k][i][j] != 1).OnlyEnforceIf(b.Not())
                    model.Add(w[k][j] == w[k][i] + q[j-1]).OnlyEnforceIf(b)

    """The constraint of weight(it has to be bigger than either 0,or q[i] if i is a package
    pick up point.And it has to be smaller than either capacity of vehicle k,or capacity[k] - qi
    if i is a package delivery point"""
    for k in range(1,K+1):
        for i in range(2*N+2*M+1):
            model.Add(max(0,q[i-1]) <= w[k][i] <= min(capacity[k-1],capacity[k-1]+q[i-1]))

    """This is the person's constraint.Basically,if vehicle k passes through 
    point i and point i+N+M (1 < i < N), which means it picks up a person in point i,
    then it can not pick up any other person until the person i has been delivered,
    which means if u[k][j] can't be anywhere in between u[k][i] and u[k][i+N+M]"""
    for k in range(1,K+1):
        for i in range(1,N+1):
            for j in range(1,N+1):
                if i != j:
                    b = model.NewBoolVar('b')
                    model.Add( u[k][j] > u[k][i]).OnlyEnforceIf(b)
                    model.Add(u[k][j] <= u[k][i]).OnlyEnforceIf(b.Not())
                    model.Add( u[k][j] > u[k][i + N + M] ).OnlyEnforceIf(b)
    #The objective function
    c = 0
    for k in range(1, K + 1):
        for i in range(2 * N + 2 * M + 1):
            for j in range(2 * N + 2 * M + 1):
                c += x[k][i][j] * d[i][j]
    model.Minimize(c)
    solver = cp_model.CpSolver()
    status = solver.Solve(model)
    if status == cp_model.OPTIMAL:
        #The computing and printing commands
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
        print('The optimized distance is: %i' % solver.ObjectiveValue())
        for k in range(1,K+1):
            for j in range(1,2*N+2*M+1):
                if solver.Value(x[k][0][j]) == 1:
                    path[k].append(j)
            while path[k][-1] != 0:
                for j in range(2 * N + 2 * M + 1):
                    if solver.Value(x[k][path[k][-1]][j]) == 1:
                        path[k].append(j)
                        if j == 0:
                            break
        Print_Sol()

    else:
        print('The problem does not have an optimal solution.')
if __name__ == '__main__':
    t = time.time()
    CP()
    print()
    print('The time taken is: ',end = ' ')
    print(time.time() - t)
