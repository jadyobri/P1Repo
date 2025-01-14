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

    detail_points = {}
    source_box = get_box_for_point(mesh['boxes'], source_point)
    dest_box = get_box_for_point(mesh['boxes'], destination_point)

    
    path = []
    boxes = {}
    if(source_box is None) or (dest_box is None):
        print("No path! (Source or destination lies outside navigable ara.)")
        return path, boxes.keys()
    


    return path, boxes.keys()

def get_box_for_point(boxes, point):
    x, y = point
    for b in boxes:
        x1, x2, y1, y2 = b
        if x1 <= x < x2 and y1 <= y < y2:
            return b
    return None