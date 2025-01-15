import math
import heapq

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    boxes_list = mesh['boxes']
    adjacency = mesh['adj']

    # finds boxes containing source/destination points
    source_box = get_box_for_point(mesh['boxes'], source_point)
    dest_box = get_box_for_point(mesh['boxes'], destination_point)

    
    path = []
    boxes = {}

    if(source_box is None) or (dest_box is None):
        print("No path! (Source or destination lies outside navigable ara.)")
        return path, boxes.keys()
    
    if source_box == dest_box:
        path = [source_point, destination_point]
        boxes[source_box] = True
        print("Already there")
        return path, boxes.keys()


    distance = {} #distance[box] is cost so far from source_box
    prevous = {} #prevous[box] which box from
    detailedPoints = {} #detailedPoints[box] is (x,y) coord used in 'box'

    distance[source_box] = 0.0
    detailedPoints[source_box] = source_point

    # Priority queue for (f_score, box)
    pq = []


    def heuristic(b):
        x, y = detailedPoints[b]
        dx, dy = destination_point
        return math.dist((x, y), (dx, dy))
    heapq.heappush(pq, (heuristic(source_box), source_box))

    while pq:
        curr_f, curr_box = heapq.heappop(pq)
        print(path)
        boxes[curr_box] = True

        if curr_box == dest_box:
            path = reconstruct_point_path(curr_box, prevous, detailedPoints, source_box, destination_point)
            print("got done")
            return path, boxes.keys()

        curr_g = distance[curr_box]
        currentPoint = detailedPoints[curr_box]


        for neighbor_box in adjacency[curr_box]:
            nextPoint = clamp_point_to_box(currentPoint, neighbor_box)
            edgeCost = math.dist(currentPoint, nextPoint)
            tentative_g = curr_g + edgeCost

            if(neighbor_box not in distance) or (tentative_g < distance[neighbor_box]):
                distance[neighbor_box] = tentative_g
                prevous[neighbor_box] = curr_box
                detailedPoints[neighbor_box] = nextPoint

                f_score = tentative_g + heuristic(neighbor_box)
                heapq.heappush(pq, (f_score, neighbor_box))
            
    print("No path available")
    return path, boxes.keys()

#Helper funcs

def get_box_for_point(boxes, point):
    x, y = point
    for b in boxes:
        x1, x2, y1, y2 = b
        if x1 <= x < x2 and y1 <= y < y2:
            return b
    return None

def clamp_point_to_box(point,box):
    x, y = point
    x1, x2, y1, y2 = box
    clampx = max(x1, min(x, x2 - 1))
    clampy = max(y1, min(y, y2 - 1))
    return(clampx, clampy)
def euclidean_dist(a,b):

    (ax,ay) = a
    (bx, by) = b
    return math.sqrt((ax-bx)**2 + (ay-by)**2)

def reconstruct_point_path(end_box, prev, detail_points, start_box, real_destination):
    #Reconstruct a 'point-level' path (list of (x,y)) from 'start_box' to 'end_box'
    #using the 'prev' dictionary and 'detail_points' dictionary.
    
    #rebuilding chain from backwards of end to start
    box_chain = []
    b = end_box
    while b != start_box:
        box_chain.append(b)
        b = prev[b]
    box_chain.append(start_box)
    box_chain.reverse()

    #convert each box in chain to its detail point
    points_path = []
    for box in box_chain:
        points_path.append(detail_points[box])
    
    #adds line to destination if line does not make it, starts from the end of the box
    if box_chain[-1] == end_box:

        final_clamped_pt = points_path[-1]
        if final_clamped_pt != real_destination:
            points_path.append(real_destination)

    #returns new path that has reconstructed points
    #Note: in implementation, replaces old one.
    return points_path