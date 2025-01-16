import queue
import math
from heapq import heappop, heappush

# checks if a point lies within a box (x1, x2, y1, y2)
def is_point_in_box(point, box):
    x1, x2, y1, y2 = box
    x, y = point
    return x1 < x < x2 and y1 < y < y2

# calculates euclidean distance between point 1 and point 2 (p1, p2)
def get_euclidean_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

# calculates valid destination point inside destination box and be reached from source point
# path stays within boundaries of boxes and follows rules and constraints
def legal_path_between(source_point, source_box, destination_box, detail_points):
    x1, x2 = source_box[0], source_box[1]
    y1, y2 = source_box[2], source_box[3]
    p_x, p_y = source_point

    # computes range for x and y coordinates where path can move following rules and constraints
    x_range = [max(x1, destination_box[0]), min(x2, destination_box[1])]
    y_range = [max(y1, destination_box[2]), min(y2, destination_box[3])]

    # finds the valid new coordinates within the ranges
    new_x = max(x_range[0], min(p_x, x_range[1]))
    new_y = max(y_range[0], min(p_y, y_range[1]))

    destination_point = (new_x, new_y)
    detail_points[source_box] = destination_point

    return destination_point

# reconstructs path by combining forward and backward path segments
def path_to(source_point, destination_point, source_box, destination_box, forward_path, backward_path, middle_box, middle_point, detail_points):
    # forward path const
    forward_path_points = [middle_point]
    current_box = middle_box
    dp = middle_point

    while current_box != source_box and forward_path.get(current_box) is not None:
        sp = legal_path_between(dp, forward_path[current_box], current_box, detail_points)
        forward_path_points.append(sp)
        dp = sp
        current_box = forward_path[current_box]
    
    forward_path_points.append(source_point)

    # backward path const
    backward_path_points = []
    current_box = middle_box
    dp = middle_point

    while current_box != destination_box and backward_path.get(current_box) is not None:
        sp = legal_path_between(dp, backward_path[current_box], current_box, detail_points)
        backward_path_points.append(sp)
        dp = sp
        current_box = backward_path[current_box]
    
    backward_path_points.append(destination_point)

    # combines both forward and backward paths
    return forward_path_points[::-1] + backward_path_points

# performs bidirectional A* search between the source and destination points
def bidirectional_a_star_search(source_point, destination_point, source_box, destination_box, adjacencies, visited_boxes, detail_points):
    forward_path = {}
    backward_path = {}
    forward_costs = {}
    backward_costs = {}
    forward_closed = set()
    backward_closed = set()
    q = []
    
    # initial heuristics for A*:
    # distance from source to destination
    # distance from destination to source
    forward_h = get_euclidean_distance(source_point, destination_point)
    backward_h = get_euclidean_distance(destination_point, source_point)
    
    forward_costs[source_box] = (0, forward_h) # g_cost = 0, h_cost = heuristic
    backward_costs[destination_box] = (0, backward_h)

    # pushes initial positions to priority queue for the forward and backward searches
    heappush(q, (forward_costs[source_box][1], source_box, destination_box, source_point))
    heappush(q, (backward_costs[destination_box][1], destination_box, source_box, destination_point))
    
    forward_path[source_box] = None
    backward_path[destination_box] = None

    while q:
        cost, current_box, current_goal, entry_point = heappop(q)
        visited_boxes.append(current_box)

        # if goal reached in forward direction
        if current_goal == destination_box:
            forward_closed.add(current_box)
            for new_box in adjacencies[current_box]:
                # calculates new cost and checks if forward search should continue
                new_entry = legal_path_between(entry_point, current_box, new_box, detail_points)
                new_g_cost = forward_costs[current_box][0] + get_euclidean_distance(new_entry, entry_point)
                new_h_cost = get_euclidean_distance(new_entry, destination_point)
                new_f_cost = new_g_cost + new_h_cost
                
                if new_box not in forward_path or new_f_cost < forward_costs[new_box][0] + forward_costs[new_box][1]:
                    forward_path[new_box] = current_box
                    forward_costs[new_box] = (new_g_cost, new_h_cost)
                    heappush(q, (new_f_cost, new_box, current_goal, new_entry))
            
            # if backward search has met, combine both paths
            if current_box in backward_closed:
                return path_to(source_point, destination_point, source_box, destination_box, forward_path, backward_path, current_box, entry_point, detail_points)
        
        # if goal reached in backward direction
        else:
            backward_closed.add(current_box)
            for new_box in adjacencies[current_box]:
                # calculates new cost and checks if backward search should continue
                new_entry = legal_path_between(entry_point, current_box, new_box, detail_points)
                new_g_cost = backward_costs[current_box][0] + get_euclidean_distance(new_entry, entry_point)
                new_h_cost = get_euclidean_distance(new_entry, source_point)
                new_f_cost = new_g_cost + new_h_cost
                
                if new_box not in backward_path or new_f_cost < backward_costs[new_box][0] + backward_costs[new_box][1]:
                    backward_path[new_box] = current_box
                    backward_costs[new_box] = (new_g_cost, new_h_cost)
                    heappush(q, (new_f_cost, new_box, current_goal, new_entry))
            
            # if forward search has met, combine both paths
            if current_box in forward_closed:
                return path_to(source_point, destination_point, source_box, destination_box, forward_path, backward_path, current_box, entry_point, detail_points)

    return [] # no path found

# performs A* search for path from source to destination
def a_star_search(source_point, destination_point, source_box, destination_box, adjacencies, visited_boxes, detail_points):
    came_from = {}
    g_costs = {}
    h_costs = {}
    f_costs = {}
    
    q = []
    heappush(q, (0, source_box)) # pushes source box into priority queue
    g_costs[source_box] = 0
    h_costs[source_box] = get_euclidean_distance(source_point, destination_point)
    f_costs[source_box] = g_costs[source_box] + h_costs[source_box]
    
    came_from[source_box] = None

    while q:
        f_cost, current_box = heappop(q)
        visited_boxes.append(current_box)

        # if destination box reached, reconstruct the path
        if current_box == destination_box:
            path = []
            while current_box is not None:
                path.append(current_box)
                current_box = came_from[current_box]
            return path[::-1]
        
        # process each neighbor (adjacent box) to explore further
        for new_box in adjacencies[current_box]:
            new_g_cost = g_costs[current_box] + get_euclidean_distance(current_box, new_box)
            new_h_cost = get_euclidean_distance(new_box, destination_point)
            new_f_cost = new_g_cost + new_h_cost

            if new_box not in came_from or new_f_cost < f_costs.get(new_box, float('inf')):
                came_from[new_box] = current_box
                g_costs[new_box] = new_g_cost
                h_costs[new_box] = new_h_cost
                f_costs[new_box] = new_f_cost
                heappush(q, (new_f_cost, new_box))

    return [] # no path found

# main function: finds a path from source to destination using mesh details
def find_path(source_point, destination_point, mesh):
    """
    Searches a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:
    
        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """
    source_box = None
    destination_box = None
    visited_boxes = []
    detail_points = {}

    # identifies the box each point (the source point and destination point) is inside
    for current_box in mesh["boxes"]:
        if is_point_in_box(source_point, current_box):
            source_box = current_box
        if is_point_in_box(destination_point, current_box):
            destination_box = current_box

    if source_box is not None and destination_box is not None:
        # if source and destination points are directly adjacent, return a simple path
        if destination_box in mesh["adj"][source_box] or source_box in mesh["adj"][destination_box]:
            mid_point = legal_path_between(source_point, destination_box, source_box, detail_points)
            return [source_point, mid_point, destination_point], list(visited_boxes)

        # if source and destination points are in the same box, return a simple path
        if destination_box == source_box:
            mid_point = legal_path_between(source_point, destination_box, source_box, detail_points)
            return [source_point, mid_point, destination_point], list(visited_boxes)

        # performs bidirectional A* search if direct adjacency doesn't hold
        result = bidirectional_a_star_search(source_point, destination_point, source_box, destination_box, mesh["adj"], visited_boxes, detail_points)
        if not result:
            print("No path found.")
            return [], list(visited_boxes)
        else:
            return result, list(visited_boxes)

    print("Source or destination points are out of bounds!")
    return [], list(visited_boxes) # if no valid source or destination, returns empty
