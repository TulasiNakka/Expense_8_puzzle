from collections import deque
import sys
from datetime import datetime
import heapq

#Initializing all the actions to 0
popped=0
expanded=0
generated=0
max_fringe=0

# I have mostly used my variable names to be the variable names that the sample dump file has.
class expense_8_puzzle:
    def __init__(self, state, cost=0, height=0, parent=None, move=None,heuristic=0):
        global popped,expanded,generated,max_fringe
        self.state = state
        self.cost = cost
        self.height = height
        self.parent = parent
        self.heuristic=heuristic
        self.move = move
        self.map=','.join(str(x) for x in state)
        generated=generated+1
    def __lt__(self, other):
        return (self.cost) < (other.cost)

# Dump file contents   
def dumpContent(expanded_node, fringe, ClosedSet=set(), method=None):
    file1.write("Command-Line Arguments : {}\n".format(sys.argv))
    file1.write("Method Selected: {}\n".format(method))
    file1.write("Running {}\n".format(method))

    file1.write("Generating successors to < state = {}, action = {}, g(n) = {}, d = {}, f(n) = {}, Parent = Pointer to {} >:\n".format(
        expanded_node.state, expanded_node.move, expanded_node.cost, expanded_node.height, expanded_node.heuristic, "None" if expanded_node.parent is None else "{}".format(expanded_node.parent.state)))

    # Counting the number of successors in the fringe
    successors_count = len(fringe)  
    file1.write("\t{} successors generated\n".format(successors_count))

    file1.write("\tClosed: {}\n".format(ClosedSet))
    file1.write("\tFringe: [\n")
    
    for x in fringe:
        state = x.state if not isinstance(x, tuple) else x[1].state
        action = x.move if not isinstance(x, tuple) else x[1].move
        g_n = x.cost if not isinstance(x, tuple) else x[1].cost
        d = x.height if not isinstance(x, tuple) else x[1].height
        f_n = x.heuristic if not isinstance(x, tuple) else x[1].heuristic
        parent = "None" if not isinstance(x, tuple) or x[1].parent is None else "{}".format(x[1].parent.state)
        file1.write("\t\t< state = {}, action = {}, g(n) = {}, d = {}, f(n) = {}, Parent = Pointer to {} >\n".format(state, action, g_n, d, f_n, parent))

    file1.write("\t]\n")


# Nodes Expanded
def expand(node):
    expanded_states = []
    for i in range(3):
        for j in range(3):
            if node.state[i][j] == 0:
                # moves format defined to be: (r_change, c_change, move_name)
                moves = [(1, 0, "Up"), (-1, 0, "Down"), (0, 1, "Left"), (0, -1, "Right")]
                
                for move in moves:
                    di, dj, move_name = move
                    new_i, new_j = i + di, j + dj
                    if 0 <= new_i < 3 and 0 <= new_j < 3:
                        new_state = [r[:] for r in node.state]
                        new_state[i][j], new_state[new_i][new_j] = new_state[new_i][new_j], new_state[i][j]
                        expanded_states.append(expense_8_puzzle(new_state, node.cost + new_state[i][j], node.height + 1, node, move_name))
                        
    return expanded_states

# Implementation of search strategies based on the algorithms

############################### Breadth First Search #####################################
def bfs():
    "Expense 8 puzzle using Breadth First Search strategy"
    global popped, expanded, generated, max_fringe, start_node, goal_node
    start_node=expense_8_puzzle(start_state)
    
    #Initialized variable queue to follow the FIFO order and start_node.
    queue = deque([start_node])

    # Keep track of visiting nodes so used set. 
    visited=set()

    while queue:

         # Tracks the maximum size of the stack during the search.
         max_fringe=max(max_fringe,len(queue))

         # remove the leftmost node from the queue to satisfy the FIFO order
         current_node = queue.popleft()
         popped += 1
         
         # If reached goal_state then terminate
         if current_node.state==goal_state:
             goal_node=current_node
             break
         
         # Nodes that are not visited will be expanded
         if current_node.map not in visited:
            expanded += 1
            queue.extend(expand(current_node))
            visited.add(current_node.map)   # To track visited Node
            if dump=='true':
                dumpContent(current_node,queue,visited,"bfs")

############################### Uniform Cost Search #####################################
def ucs(): 
    "Expense 8 puzzle using Uniformed Cost Search strategy"
    global popped, expanded, generated, max_fringe, start_node, goal_node
    start_node=expense_8_puzzle(start_state)

    # start_node added to fringe with cost O
    fringe = [(0, start_node)]

    # Keep track of visiting nodes so used set. 
    visited = set()

    while fringe:
        max_fringe=max(max_fringe,len(fringe))

        # Here popped nodes with lowest cost from fringe
        cost, current_node = heapq.heappop(fringe)
        popped += 1

        # If reached goal_state then terminate
        if current_node.state==goal_state:
             goal_node=current_node
             break
        
        # Nodes that are not visited will be expanded
        if current_node.map not in visited:
            expanded += 1
            for node in expand(current_node):
                heapq.heappush(fringe, (node.cost ,node))
            visited.add(current_node.map)   # To track visited Node
            if dump=='true':
                dumpContent(current_node,fringe,visited,"ucs")

############################### Depth First Search #####################################
def dfs():
    "Expense 8 puzzle using Depth First Search strategy"
    global popped, expanded, generated, max_fringe, start_node, goal_node
    start_node=expense_8_puzzle(start_state)

    # Implementing as a Stack
    queue = deque([start_node])

    # Keep track of visiting nodes so used set. 
    visited=set()

    while len(queue)>0:
         
         # Tracks the maximum size of the stack during the search.
         max_fringe=max(max_fringe,len(queue))

         # pops the rightmost node from the stack. 
         current_node = queue.pop()
         popped += 1

         # If reached goal_state then terminate
         if current_node.state==goal_state:
             goal_node=current_node
             break
         
         # Nodes that are not visited will be expanded
         if current_node.map not in visited:
            expanded += 1
            queue.extend(expand(current_node))
            visited.add(current_node.map)  # To track visited Node
            if dump=='true':
                dumpContent(current_node,queue,visited,"dfs")

############################### Depth Limited Search #####################################
#  (Note: Depth Limit will be obtained as a Console Input) 
def dls(limit,method_name):
    "Expense 8 puzzle using Depth Limited Search strategy"

    #same strategy as dfs with a depth limit l
    global popped, expanded, generated, max_fringe, start_node, goal_node
    start_node=expense_8_puzzle(start_state)
    queue = deque([start_node])
    visited=set()
    while len(queue)>0:
         max_fringe=max(max_fringe,len(queue))
         current_node = queue.pop()
         popped += 1
         if current_node.state==goal_state:
             goal_node=current_node
             return True
         if current_node.height < limit:
            expanded += 1
            queue.extend(expand(current_node))
            if dump=='true':
                dumpContent(current_node,queue,[],method_name)
    return False

############################### Iterative Deepening Search #####################################
def ids():
    "Expense 8 puzzle using Iterative Deepening Search strategy"
    global popped, expanded, generated, max_fringe, start_node, goal_node

    # Initialize the 
    height=0
    solution=False
    method_name = "ids"
    while not solution:
        solution = dls(height,method_name)
        height += 1

############################### Greedy Search #####################################
def greedy():
    "Expense 8 puzzle using Greedy strategy"
    global popped, expanded, generated, max_fringe, start_node, goal_node
    start_node=expense_8_puzzle(start_state)

    # The fringe is initialized as a priority queue and contains a heuristic initially 0 and a node. 
    fringe = [(0, start_node)]

    # Keep track of visiting nodes so used set. 
    visited = set()

    while fringe:
        max_fringe=max(max_fringe,len(fringe))

        # Here popped nodes with lowest heuristic value from fringe 
        heuristic, current_node = heapq.heappop(fringe)
        popped += 1

        # If reached goal_state then terminate
        if current_node.state==goal_state:
             goal_node = current_node
             break
        
        # Nodes that are not visited will be expanded
        if current_node.map not in visited:
            expanded += 1

            # Calculates manhattan_distance distance heuristic for successors
            for node in expand(current_node):
                sum=0
                for i in range(3):
                    for j in range(3):
                        sum += manhattan_distance(node.state[i][j],i,j)*node.state[i][j]
                node.heuristic = sum

                # pushes next node and heuristic value to fringe
                heapq.heappush(fringe, (node.heuristic ,node))
            visited.add(current_node.map)  # To track visited Node
            if dump=='true':
                dumpContent(current_node,fringe,visited,"greedy")

def manhattan_distance(val,r,c):
    for i in range(3):
        for j in range(3):
            if goal_state[i][j]==val:
                return abs(r-i)+abs(c-j)

############################### A* Search #####################################
def Astar():
    "Expense 8 puzzle using A* Search strategy"
    global popped, expanded, generated, max_fringe, start_node, goal_node
    start_node=expense_8_puzzle(start_state)

    # same like greedy the fringe is initialized as a priority queue and contains a heuristic initially 0 and a node.
    fringe = [(0, start_node)]
    visited = set()
    while fringe:
        max_fringe=max(max_fringe,len(fringe))

        # Here popped nodes with lowest heuristic value from fringe 
        heuristic, current_node = heapq.heappop(fringe)
        popped=popped+1

        # If reached goal_state then terminate
        if current_node.state==goal_state:
             goal_node=current_node
             break
        
        # Nodes that are not visited will be expanded
        if current_node.map not in visited:
            expanded += 1
            for node in expand(current_node):
                sum=0

                #calculates the heuristic value for each successor node based on the Manhattan distance heuristic.
                for i in range(3):
                    for j in range(3):
                        sum=sum+manhattan_distance(node.state[i][j],i,j)*node.state[i][j]
                node.heuristic= sum+node.cost   #Expected Cost: The calculated heuristic value along with the cost is assigned to the node.heuristic attribute.

                # pushes next node and heuristic value to fringe based on priority
                heapq.heappush(fringe, (node.heuristic,node))
            visited.add(current_node.map) #Track visited nodes
            if dump=='true':
                dumpContent(current_node,fringe,visited,"a*")

# Files operations to read start and goal files from CL arguments
with open(sys.argv[1], 'r') as f:
    start_state = list(map(str, f.read().split()))
with open(sys.argv[2], 'r') as f:
    goal_state = list(map(str, f.read().split()))

start_state = start_state[0:9]
x=0
start= [[0]*3 for i in range(3)]
for i in range(3):
    for j in range(3):
        start[i][j]=int(start_state[x])
        x=x+1
start_state=start
goal_state = goal_state[0:9]
x=0
goal= [[0]*3 for i in range(3)]
for i in range(3):
    for j in range(3):
        goal[i][j]=int(goal_state[x])
        x=x+1
goal_state=goal
start_node,goal_node=None,None

dump='false'
if len(sys.argv)>=5:
    dump=sys.argv[4]

# Checking for dump true and formatting the dump files as sample given
if dump=='true':
    filename = "trace-"+datetime.now().strftime("%m-%d-%Y-%I_%M_%S_%p")+".txt"
    file1 = open(filename,'w')


if len(sys.argv)>=4:
    if sys.argv[3] == 'bfs':
        bfs()
    elif sys.argv[3] == 'ucs':
        ucs()
    elif sys.argv[3] == 'dfs':
        dfs()
    elif sys.argv[3] == 'dls':
        print("Please Enter depth limit l=" )
        limit =  int(input())
        method_name = "dls"
        dls(limit,method_name)
    elif sys.argv[3] == 'ids':
        ids()
    elif sys.argv[3] == 'greedy':
        greedy()
    elif sys.argv[3] == 'a*':
        Astar()
    else:
        dump=sys.argv[3]
        if dump=='true':
            filename = "trace-" + datetime.now().strftime("%Y-%m-%d-%I_%M_%S_%p")+".txt"
            file1 = open(filename,'w')
        Astar()
else:
    Astar()

# printing output   
print("Nodes Popped:", popped)
print("Nodes Expanded:", expanded)
print("Nodes Generated:", generated)
print("Max Fringe Size:", max_fringe)
steps=[]
print("Solution Found at depth "+str(goal_node.height)+" with cost of "+str(goal_node.cost)+".")
while goal_node!=start_node:
    steps.append("MOVE "+str(goal_node.cost-goal_node.parent.cost)+" "+goal_node.move)
    goal_node=goal_node.parent
steps=steps[::-1]
print("Steps:")
print('\n'.join(map(str,steps)))

if dump=='true':
    file1.write("\nNodes Popped:"+str(popped))
    file1.write("\nNodes Expanded:"+str(expanded))
    file1.write("\nNodes Generated:"+str(generated))
    file1.write("\nMax Fringe Size:"+str(max_fringe))
    file1.close()


