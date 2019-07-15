# DEFINING HELPER FUNCTIONS to calculate f,g,h before calling the search on the given Map_40

def path_cost(path,M,cost=0):
    """
    Function to calculate actual Path Cost between 2 nodes.
    The PATH COST (g) is the actual euclidean distance for the path (g)
    in f = g + h
    It is calculated as sqrt((y2-y1)^2 + (x2-x1)^2) 
    for all the intersections in the path, aggregated
    """
    
    # To get x,y coordinates of the path
    M = M.intersections
    for i in range(len(path)-1):
        # Add up the cost of each road sqrt(x^2 + y^2) in the path to give path cost
        x_d = (M[path[i]][0]-M[path[i+1]][0])
        y_d = (M[path[i]][1]-M[path[i+1]][1])
        cost += ((y_d**2) + (x_d**2)) ** 0.5
    # Return the path cost of path
    return cost

def estimated_path_cost(n1,n2,M):
    """
    Function to calculate Estimated Path cost, which is our heuristic.
    The ESTIMATED COST OF THE PATH (h) is the Euclidean Heuristic function h(s)
    in f = g + h
    from the start node (n1) to the goal node (n2)
    Defined as airline distance sqrt((y2-y1)^2 + (x2-x1)^2) 
    Admissible since it is always lower than actual cost
    """

    # To get x,y coordinates of the path
    M = M.intersections 
    # Calculate y distance and x distance (coordinates)
    x_d = (M[n1][0]-M[n2][0])
    y_d = (M[n1][1]-M[n2][1])
    # Estimated cost is the sqrt()
    estimated_cost = ((y_d**2) + (x_d**2)) ** 0.5
    return estimated_cost

def minimum_path_cost(frontier,goal,M):
    """
    Function to calculate the minimum cost in A* search
    The MINIMUM COST OF THE PATH (f) in the A* search is defined by:
    f = g + h, here we calculate f as the total path cost(g) + Heuristic cost (h) 
    """
    
    # Create a dictionary of values storing the total costs
    minimum_path_cost = dict([(path_cost(frontier[node]+[node],M) + estimated_path_cost(node,goal,M),node) for node in frontier])  
    
    # Return the minimum cost of all the total costs from the dict
    minimum_path_cost = minimum_path_cost[min(minimum_path_cost)]
    return minimum_path_cost

def remove_node(frontier,M):
    """
    This Funtion removes a node from the anode can be removed from the frontier if that can be reached 
    through a shorter path from another node of the frontier, remove it 
    from frontier
    """
    for node in list(frontier):
        for i in [i for i in frontier if i!=node]:
            
            # If the node is not in the frontier, move ahead
            if node not in frontier:
                continue        
            
            # If the node is already seen, move ahead
            if frontier[i] == frontier[node]:
                continue
            
            # If the node not in the road yet, move ahead
            if i not in M.roads[node]:
                continue
            
            # If the path cost of the node from starting node 
            # is less than path cost from frontier to that node
            # remove the frontier node
            if path_cost(frontier[node] + [node] + [i], M) < path_cost(frontier[i] + [i], M):
                del frontier[i]
    return frontier


def shortest_path(M, start, goal):
    """
    MAIN: A* SEARCH = Algorithm for shortest path least cost search
    f = g + h
    Call f(minimum_cost_path) on the all the frontier nodes till we explore the goal node
    """
    
    # explored is a set of unique nodes, containing the start node first
    explored = set([start])
    # Initialize frontier dictonary containing
    frontier = dict([(i,[start]) for i in M.roads[start]]) if start!=goal else {start:[]}
    
    
    while frontier: # While we still have a node on the frontier to explore
        
        # Call f = g + h on the frontier node towards the goal
        explore = minimum_path_cost(frontier,goal,M)
        
        for i in [ i for i in M.roads[explore] if i not in frontier.keys() | explored ]:
            frontier[i] = frontier[explore] + [explore]
        
        # Remove the node from the frontier
        frontier = remove_node(frontier,M)
        
        # If we have explored the goal node
        if explore == goal:
            # Stop the search when goal is explored, break out
            path = frontier[explore] + [explore]
            return path
        else:
            # Else, add this node to the explored list and move on
            explored.add(explore)
            # Remove the node from the frontier 
            del frontier[explore]

