import math

def distance_2d(pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    cost = math.sqrt(
        (x1 - x2) * (x1 - x2)
        + (y1 - y2) * (y1 - y2) 
    )
    return cost

def distance_3d(pos1, pos2):
    x1, y1, z1 = pos1
    x2, y2, z2 = pos2
    cost = math.sqrt(
        (x1 - x2) * (x1 - x2)
        + (y1 - y2) * (y1 - y2) 
        + (z1 - z2) * (z1 - z2)
    )
    return cost