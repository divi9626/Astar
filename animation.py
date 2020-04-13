import numpy as np
import matplotlib.pyplot as plt
import time
import math
import sys
from matplotlib.animation import FuncAnimation


blank_list = []
for entries in range(4000000):
    entries = math.inf
    blank_list.append(entries)
cost_matrix = np.array(blank_list, dtype = object).reshape(200,200,100)

blank_list1 = []
for entries in range(4000000):
    entries = 0
    blank_list1.append(entries)
visited_mat = np.array(blank_list, dtype = object).reshape(200,200,100)

blank_list2 = []
for entries in range(4000000):
    entries = 0
    blank_list2.append(entries)
cost_2_come = np.array(blank_list, dtype = object).reshape(200,200,100)


def approximation(a):
    b = math.ceil(a)
    if b - a <= 0.75 and b - a >= 0.25:
        a = b - 0.5
    elif b - a > 0.75 and b - a <= 1:
        a = math.floor(a)
    elif b - a < 0.25 and b - a >=0:
        a = b
    else:
        return a 
    return a


def angle_approximation(a):
    a = int(a)
    a = a/10
    b = math.ceil(a)
    if b - a <= 0.75 and b - a >= 0.25:
        a = b - 0.5
        a = a*10
    elif b - a > 0.75 and b - a <= 1:
        a = math.floor(a)
        a = a*10
    elif b - a < 0.25 and b - a >= 0:
        a = b
        a = a*10
    else:
        a = a*10
        return a
    return a

orientation = 0        
k = angle_approximation(orientation)                   # real value 


# Everything in m.  obstacle space is 10*10 m. Converted in cm
R1 = 30
R2 = 60


#ind1 = 2  #int(input('enter x coordinate of starting node'))  # 5
#ind2 = 2  #int(input('enter y coordinate of starting node'))  # 5
ind = (5,5,k)

#goal1 = 20  #int(input('enter x coordinate of goal node'))  # 295
#goal2 = 20  #int(input('enter y coordinate of goal node'))  # 195 
goal = (50, 90)

#d = int(input())



cost_2_come[int(2*ind[0])][int(2*ind[1])][int((ind[2])/5)] = 0

r = 10
c = 10

b = r+c

def obstacle_here(i,j):
    
    if i >= 22.5+b and i<= 37.5+b and j >= 12.5+b and j <= 27.5+b:
        return True
    
    if i <= 2.5+b and i<= 17.5+b and j >= 37.5+b and j <= 57.5+b:
        return True
    
    if i >= 82.5+b and i <= 97.5+b and j >= 37.5+b and j >= 57.5+b:
        return True
    
    if (i-70)**2 + (j-20)**2 <= 100+b:
        return True
    
    if (i-50)**2 + (j-50)**2 <= 100+b:
        return True 
    
    if (i-30)**2 + (j-80)**2 <= 100+b:
        return True
    
    if (i-70)**2 + (j-80)**2 <= 100+b:
        return True
    
    

if obstacle_here(ind[0],ind[1]):
    print("obstacle in start node")
    sys.exit()
if obstacle_here(goal[0],goal[1]):
    print("obstacle in goal node")
    sys.exit()

    

def action_one(i,j,k,R1,R2):
    l = 1.6
    r = 0.5
    dt = 1
    u1 = (2*3.14*r*0)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*R1)/60    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j

        
    for iterations in range(10):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 + u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 + u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost
    

def action_two(i,j,k,R1,R2):
    l = 1.6
    r = 0.5
    dt = 1
    u1 = (2*3.14*r*R1)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*0)/60    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(10):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
    
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost



def action_three(i,j,k,R1,R2):
    l = 1.6
    r = 0.5
    dt = 1
    u1 = (2*3.14*r*R1)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*R1)/60    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(10):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost


def action_four(i,j,k,R1,R2):
    l = 1.6
    r = 0.5
    dt = 1
    u1 = (2*3.14*r*0)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*R2)/60    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(10):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost
    
def action_five(i,j,k,R1,R2):
    l = 1.6
    r = 0.5
    dt = 1
    u1 = (2*3.14*r*R2)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*0)/60    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(10):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost
    

def action_six(i,j,k,R1,R2):
    l = 1.6
    r = 0.5
    dt = 1
    u1 = (2*3.14*r*R2)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*R2)/60    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(10):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost



def action_seven(i,j,k,R1,R2):
    l = 1.6
    r = 0.5
    dt = 1
    u1 = (2*3.14*r*R1)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*R2)/60    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(10):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost


def action_eight(i,j,k,R1,R2):
    l = 1.6
    r = 0.5
    dt = 1
    u1 = (2*3.14*r*R2)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*R1)/60    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(10):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost




def get_neighbours(i,j,k):
    neighbours_cost = []
    index = []
    
    one = action_one(i,j,k,R1,R2)
    if one is not None:
        neighbours_cost.append(one[1])
        index.append(one[0])
        
    two = action_two(i,j,k,R1,R2)
    if two is not None:
        neighbours_cost.append(two[1])
        index.append(two[0]) 
        
    three = action_three(i,j,k,R1,R2)
    if three is not None:
        neighbours_cost.append(three[1])
        index.append(three[0])
        
    four = action_four(i,j,k,R1,R2)
    if four is not None:
        neighbours_cost.append(four[1])
        index.append(four[0]) 
        
    five = action_five(i,j,k,R1,R2)
    if five is not None:
        neighbours_cost.append(five[1])
        index.append(five[0])
        
    six = action_six(i,j,k,R1,R2)
    if six is not None:
        neighbours_cost.append(six[1])
        index.append(six[0])
        
    seven = action_seven(i,j,k,R1,R2)
    if seven is not None:
        neighbours_cost.append(seven[1])
        index.append(seven[0]) 
        
    eight = action_eight(i,j,k,R1,R2)
    if eight is not None:
        neighbours_cost.append(eight[1])
        index.append(eight[0])
                     
    return neighbours_cost, index


'''def get_current_value(i,j):
    current_value = cost_matrix[i][j]
    return current_value'''

'''def locate_value_index(mat):
    i,j = np.where(mat == node)
    i = int(i)
    j = int(j)
    return (i,j)'''


def sort_list1(list_a,list_b):
    list_b = [i[1] for i in sorted(zip(list_a, list_b))]
    
def sort_list2(list_a):
    list_a.sort()
    

node_cost = 0
cost_matrix[int(2*ind[0])][int(2*ind[1])][int(ind[2]/5)] = 0

explore_queue = []  
index_queue = []
index_queue.append(ind)
explore_queue.append(node_cost)

breakwhile = False

start_time = time.clock()

visited_list = []
parent_map = {}
parent_map[ind] = None

while len(index_queue) != 0 and not breakwhile:
    node = index_queue[0]
    visited_mat[int(2*node[0])][int(2*node[1])][int(node[2]/5)] = 1
    visited_list.append(node)

    print(index_queue[0], explore_queue[0])
    index_queue.pop(0)
    explore_queue.pop(0)
    pair = get_neighbours(node[0],node[1],node[2])
    #print('pair',pair)
    neighbours_cost = pair[0]
    index = pair[1]
    print(index)
    print(neighbours_cost)
    
    #######
    if math.sqrt((goal[0] - node[0])**2 + (goal[1] - node[1])**2) <= 15:
        print("goal reached")
        goal_ind = node
        breakwhile = True
    
    #######

    
    for i in range(len(index)):
        if not visited_mat[int(2*((index[i])[0]))][int(2*(index[i])[1])][int((index[i][2])/5)] == 1:    
                                        
            old_cost = cost_matrix[int(2*((index[i])[0]))][int(2*(index[i])[1])][int((index[i][2])/5)]
            if neighbours_cost[i] < old_cost:
                cost_matrix[int(2*((index[i])[0]))][int(2*(index[i])[1])][int((index[i][2])/5)] = neighbours_cost[i]
                #if old_cost != math.inf:
                    #ind_node = index_queue.index((index[i][0], index[i][1]))
                    #index_queue.pop(ind_node)
                    #explore_queue.pop(ind_node)
                    
                index_queue.append(index[i])
                explore_queue.append(cost_matrix[int(2*((index[i])[0]))][int(2*(index[i])[1])][int((index[i][2])/5)])  # cost_matrix getting updated
                parent_map[index[i]] = node


       
            
    sort_list1(explore_queue,index_queue)
    sort_list2(explore_queue)
    visited_mat[int(2*node[0])][int(2*node[1])][int(node[2]/5)] == 1
   

end_time = time.clock()
print(end_time - start_time)
 
path_list = []
parent = parent_map[goal_ind]
path_list.append(goal_ind)
while parent is not None:
    path_list.append(parent)
    parent = parent_map[parent]

path_list.reverse()
print(path_list)

########## Plotting ###########
path_final = np.asarray(path_list)

x_coord = path_final[:,0]
y_coord = path_final[:,1]


def plot_curve(ax, Xi,Yi,Thetai,UL,UR):
    t = 0
    r = 0.1
    L = 1.5
    dt = 0.05
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180

# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes

    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        ax.plot([Xs, Xn], [Ys, Yn], color="blue")
        plt.plot(x_coord, y_coord, color = 'black')

    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan


fig, ax = plt.subplots()

actions=[[0, R1],[R1,0],[R1,R1],[0,R2],[R2,0],[R2,R2],[R1,R2],[R2,R1]]





#animation = FuncAnimation(fig, func=plotfunction, frames=200, interval=10)   
ax.set_aspect('equal')

plt.xlim(0,100)
plt.ylim(0,100)

###### obstacle plotting #######

plt.scatter(goal[0], goal[1], s = 30, c ='red')
plt.scatter(ind[0], ind[1], s = 30, c ='red')

rectangle = plt.Rectangle((22.5, 72.5), 15, 15, fc='r')
ax.add_patch(rectangle)

rectangle = plt.Rectangle((2.5, 42.5), 15, 15, fc='r')
ax.add_patch(rectangle)

rectangle = plt.Rectangle((82.5, 42.5), 15, 15, fc='r')
ax.add_patch(rectangle)

circle = plt.Circle((30, 20), radius=10, fc='g')
ax.add_patch(circle)

circle = plt.Circle((70, 20), radius=10, fc='g')
ax.add_patch(circle)

circle = plt.Circle((50, 50), radius=10, fc='g')
ax.add_patch(circle)

circle = plt.Circle((70, 80), radius=10, fc='g')
ax.add_patch(circle)

circle = plt.Circle((goal), radius=15, color='black',fill=False)
ax.add_patch(circle)

plt.title('Vector - Path',fontsize=10)




def animate(point):
    print('current count', len(path_list))
    #point = 
#    for point in range(len(path_list)):
    if point < len(path_list):
        x = path_list[point][0]
        y = path_list[point][1]
        theta = path_list[point][2]
        print(x, y, theta)
        for action in actions:
                X1 = plot_curve(ax, x,y,theta,action[0],action[1])
                
    
        
animation = FuncAnimation(fig, func=animate, interval=0.01)
fig.show()
plt.draw()
plt.show()
