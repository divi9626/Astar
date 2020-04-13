Astar_phase_3

Astar algorithm is executed on differential drive constraints robot .
There are two codes submitted . One is for small step (astar_diff.py) size is accurate but takes some time to compile and works for every point with less threshold. (test case compilation time :- 191 sec)
Other code submitted is (Astar_diff.py) which has bigger step size. 
Matplotlib is used for the animation and obstacle space is modified
The values of RPM, radius, delta_t and wheel based are fixed and start and node goal node can be taken as user input (from lines 69-74 in all codes)
Quiver plot is used for visualization


Details of algorithm:-
Backtrack nodes , cost and velocities are outputs
Eight action sets are defined based on diff RPM values and accordingly linear velocity and angular velocity is taken as output. 
The nodes are incremented using steps with iterations to get a curve exactly how robot would move
actions=[[0, R1],[R1,0],[R1,R1],[0,R2],[R2,0],[R2,R2],[R1,R2],[R2,R1]]


