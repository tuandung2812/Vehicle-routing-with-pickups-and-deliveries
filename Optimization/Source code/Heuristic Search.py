import numpy as np
import time
t0 =time.time()
#File reading
with open('test3','r') as f:
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
current_weight = [0 for i in range(K+1)]    #current_weight[k]: Chỉ tải trọng tạm thời của xe
path = {}                                   #path: dict ghi lại quãng đường đã đi của các xe
last_position = [0 for i in range(K+1)]     #last_postion[k] : chỉ vị trí hiện tại của xe k
person_in_car = [False for i in range(K+1)] #person_in_car[k]: chỉ trạng thái người của xe k (xe k có người hay không)

# def Heuristic solution:
def Greedy_Sol():
    global  N,M,K,q,capacity,d,current_weight,path,last_position

    """Từ điểm 0 bắt đầu,xem điểm nào gần điểm 0 nhất => đưa xe đi đến điểm đấy
    candidate: array chỉ những điểm có thể đi được
    human_candidate: chỉ những người có thể đón được
    nếu đã đi qua điểm i (1<i<N+M+1) : xóa bỏ i khỏi candidate ,human_candidate(nếu i <= N) 
    hoặc package_candidate (N+1 <= i <= N+M) """
    candidate = [i for i in range(1,2*N+2*M+1)]
    human_candidate = [i for i in range(1,N+1)]
    package_candidate = [i for i in range(N+1,N+M+1)]
    near_distances = list(d[0][1:N+M])
    chosen_points = []
    for k in range(1,K+1):
        min = 9999
        for i in range(len(near_distances)):
            if near_distances[i] < min and i not in chosen_points :
                min = near_distances[i]
                last_position[k] = i+1
        chosen_points.append(last_position[k]-1)
        path[k] = [last_position[k]]
        candidate.remove(last_position[k])
        if last_position[k] <= N and last_position[k] in human_candidate:
            human_candidate.remove(last_position[k])
            person_in_car[k] = True
        elif last_position[k] in package_candidate:
            package_candidate.remove(last_position[k])
            current_weight[k] += q[last_position[k]-(N+1)]

    # Bước 2: Iterate đến khi nào không còn candidate nào nữa
    while len(candidate) > 0:
        for k in range(1, K + 1):
            sorted_candidate = sorted(candidate, key=lambda x: d[last_position[k]][x]) #sắp xếp các điểm chưa đi đến theo khoảng cách so với vị trí hiện tại
            if sorted_candidate == []:
                break
            for cand in sorted_candidate:
                if cand <= N:  # Nếu là người => bỏ khỏi candidate,cập nhật vị trí và trạng thái
                    if person_in_car[k] == False:
                       human_candidate.remove(cand)
                       person_in_car[k] = True
                       path[k].append(cand)
                       candidate.remove(cand)
                       sorted_candidate.remove(cand)
                       last_position[k] = cand
                elif (N < cand <= N + M):  # Nếu là vật,tương tự
                    if current_weight[k] + q[cand-(N+1)] <= capacity[k-1]:
                        package_candidate.remove(cand)
                        current_weight[k] += q[cand - (N + 1)]
                        path[k].append(cand)
                        candidate.remove(cand)
                        sorted_candidate.remove(cand)
                        last_position[k] = cand
                elif (N+M+1 <= cand <= 2*N+M):
                    if (cand-(N+M)) in path[k]:
                       candidate.remove(cand)
                       person_in_car[k] = False
                       path[k].append(cand)
                       last_position[k] = cand
                elif (2*N+M+1 <= cand <= 2*N+2*M):
                    if (cand - (N+M)) in path[k]:
                        candidate.remove(cand)
                        path[k].append(cand)
                        current_weight[k] -= q[(cand-(N+M)) - (N+1)]
                        last_position[k] = cand
    distance = 0
    for k in range(1,K+1):
        path[k].insert(0,0)
        path[k].append(0)
        for i in range(len(path[k] )-1):
            distance += d[path[k][i]][path[k][i+1]]
    return distance

def find_distance(k): #Hàm tìm xem xe thứ k đi tổng cộng được bao nhiêu
    global path,d
    distance = 0
    for i in range(len(path[k]) - 1):
        distance += d[path[k][i]][path[k][i + 1]]
    return distance

def Print_Sol():
    global path,d
    for k in path.keys():
        print('Path of vehicle number ' + str(k) + ' is:')
        print('0 ==>',end = '')
        for value in path[k][1:len(path[k])-1]:
            print(str(value) ,end = ' ')
            if 1<= value <= N:
                print('(person pick-up point)'+ ' ==>' ,end =' ')
            if N+1 <= value <= N + M:
                print('(package pick-up point)'+ ' ==>',end='')
            if N + M + 1 <= value <= 2*N+M:
                print('(person delivery point)'+ ' ==>',end='')
            if 2*N+M+1 <= value <= 2*N+2*M:
                print('(package delivery point)'+ ' ==>',end='')
        print(path[k][-1])
        print('The distance vehicle number '+ str(k) + ' travelled is: ',end= ' ')
        print()
        print(find_distance(k))
        print()

if __name__ == '__main__':
    print("The estimated optimized distance is: ", end=' ')
    print(Greedy_Sol())
    print()
    Print_Sol()
    print( )
    print('The time taken for the estimation is: ',end = ' ' )
    print(time.time() - t0)



