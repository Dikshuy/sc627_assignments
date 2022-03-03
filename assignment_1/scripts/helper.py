import math 

def dist(p1,p2):
    # print(p1, p2)
    return math.sqrt((p1[1]-p2[1])**2+(p1[0]-p2[0])**2)       # euclidean distance between two points

def computeLineThroughTwoPoints(p1,p2):
    norm = dist(p1,p2)
    if norm < 10**(-8):
        print("identical points")
        return None
    return [(p2[1]-p1[1])/norm,(p1[0]-p2[0])/norm,(p1[1]*p2[0]-p1[0]*p2[1])/norm]   # eq of plane passing through these points

def computeDistancePointToLine(q,p1,p2):
    [a,b,c] = computeLineThroughTwoPoints(p1,p2)
    return abs(a*q[0]+b*q[1]+c)

def computeDistancePointToSegment(q,p1,p2):
    # using the concept of vectors dot product, we can compute the distance of the point from segment
    a,b,c = computeLineThroughTwoPoints(p1,p2)
    proj = (-b*(q[0]-p1[0]) + a*(q[1]-p1[1])) / dist(p1,p2)
    if proj <= 0:   return dist(p1,q), 1
    elif proj >= 1:   return dist(p2,q), 2
    else:   return computeDistancePointToLine(q,p1,p2), 0 

def computeDistancePointToPolygon(q,P):
    min_dist = math.inf 
    n = len(P)
    for i in range(len(P)):
        dist, wt = computeDistancePointToSegment(q,P[i%n],P[(i+1)%n])
        if dist < min_dist:
            min_dist, w, index = dist, wt, i
    return min_dist, w, index

def computeTangentVectorToPolygon(q,P):
    min_dist, w, index = computeDistancePointToPolygon(q,P)
    n = len(P)
    if w == 0:
        tot = 0
        for i in range(n):
            tot += (P[i%n][0]-P[(i+1)%n][0])*(P[i%n][1]-P[(i+1)%n][1])
        d = dist(P[index%n],P[(index+1)%n])
        if tot > 0: return  -((P[index%n][0]-P[(index+1)%n][0]))/d, -(P[index%n][1]-P[(index+1)%n][1])/d
        else: return  (P[index%n][0]-P[(index+1)%n][0])/d, (P[index%n][1]-P[(index+1)%n][1])/d
    if w == 1:
        a,b,c = computeLineThroughTwoPoints(q,P[index%n])
        return  a-0.3*b, b+0.3*a 
    else: 
        a,b,c = computeLineThroughTwoPoints(q,P[(index+1)%n])
        return  a-0.3*b, b+0.3*a 