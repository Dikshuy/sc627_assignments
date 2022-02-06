import math 

def dist(p1,p2):
    return math.sqrt((p1[1]-p2[1])**2+(p1[0]-p2[0])**2)       # euclidean distance between two points

def computeLineThroughTwoPoints(p1,p2):
    norm = dist(p1,p2)
    if norm < 10**(-8):
        print("identical points")
        return None
    return [(p2[1]-p1[1])/norm,(p1[0]-p2[0])/norm,(p1[1]*p2[0]-p1[0]*p2[1])/norm]   # eq of plane passing through these points

def computeDistancePointToLine(q,p1,p2):
    [a,b,c] = computeLineThroughTwoPoints(p1,p2)
    return math.abs(a*q[0]+b*q[1]+c)

def computeDistancePointToSegment(q,p1,p2):
    # using the concept of vectors, we can compute the distance of the point from segment
    p1q = [[q[0]-p1[0]],[q[1]-p1[1]]]
    p2q = [[q[0]-p2[0]],[q[1]-p2[1]]]
    p1p2 = [[p2[0]-p1[0]],[p2[1]-p1[1]]]
    
    p1p2_p1q = p1p2[0]*p1q[0] + p1p2[1]*p1q[1]
    p1p2_p2q = p1p2[0]*p2q[0] + p1p2[1]*p2q[1]

    if p1p2_p1q < 0:
        return  dist(q,p1)
    
    if p1p2_p2q > 0:
        return  dist(q,p2)

    else:
        return computeDistancePointToLine(q,p1,p2)

def computeDistancePointToPolygon(q,P):
    closest = math.inf
    for i in range(len(P)):
        closest = min(closest, computeDistancePointToLine(P[(i)%len(P)],P[(i+1)%len(P)],q))
    return closest

def computeTangentVectorToPolygon(q,P):
    pass 
