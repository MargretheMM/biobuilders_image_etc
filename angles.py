import networkx as nx
import numpy as np
import scipy.spatial
import sys
import ntpath
import math

G = nx.read_gpickle(sys.argv[1])

def NumberOfJunctions(G):
    junctions = 0    
    for n in G.nodes():
        if type(G) == type(nx.DiGraph):
            if len(G.neighbors(n)) >= 2:
                junctions += 1
        else:
            if len(G.neighbors(n)) >= 3:
                junctions += 1 
                
    return junctions
    
def NumberOfTips(G):
    tips = 0  
    for n in G.nodes():
        if type(G) == type(nx.DiGraph):
            if len(G.neighbors(n)) == 0:
                tips += 1
        else:
            if len(G.neighbors(n)) == 1:
                tips += 1 
                
    return tips
    
def TotalLength(G):
    return np.asarray([e[2]['weight'] for e in G.edges_iter(data=True)]).sum()
    
def AverageEdgeLength(G):
    return np.asarray([e[2]['weight'] for e in G.edges_iter(data=True)]).mean()
    
def AverageEdgeRadius(G):
    return np.asarray([e[2]['conductivity'] for e in G.edges_iter(data=True)]).mean()
    
def TotalNetworkArea(G):
    return np.asarray([e[2]['weight']*e[2]['conductivity'] \
        for e in G.edges_iter(data=True)]).sum()
    
def AreaOfConvexHull(G):
    points = np.asarray([[n[1]['y'],n[1]['x']] for n in G.nodes(data=True)])   
    hull = scipy.spatial.ConvexHull(points)
    vertices = points[hull.vertices]
    vertices = np.vstack([vertices,vertices[0,0:]])
    lines = np.hstack([vertices,np.roll(vertices,-1,axis=0)])
    area = 0.5*abs(sum(x1*y2-x2*y1 for x1,y1,x2,y2 in lines))
    return area

def NumberOfCycles(G):
    return len(nx.cycle_basis(G))

# function to calculate angles for nodes with a specific number of neighbours
def Anglecalc(G,num):
    # creates a dictionary - each set of three nodes used at a key will give the angle between them
    # note that nodes are addressed by their identifier numbers, not their positions
    results={}
    nodes = dict(G.nodes(data=True))
    for node in G.nodes(data=True):
        neighbors = nx.neighbors(G,node[0])
        if len(neighbors) == num:
            continue
        for start, i in enumerate(neighbors):
            i_node = nodes[i]
            for j in neighbors[start+1:]:
                results[(node[0], i, j)] = angle(node[1], i_node, nodes[j])           
    return results


def angle(vertex,side1, side2):
    # use relative positions - corresponds to l an r having the coordinates of the v to l and v to r vectors    
    def coords(node, rel_pos=(0,0)):
        return (node["x"] - rel_pos[0], node["y"] - rel_pos[1])
    v = coords(vertex)
    l = coords(side1, v)
    r = coords(side2, v)
    # calculating the cos of the angle between the l and r vectors
    denom = (l[0]**2+l[1]**2)**.5 * (r[0]**2+r[1]**2)**.5    
    if (denom == 0):
        return 'no angle' 
    res = (l[0]*r[0]+l[1]*r[1]) / denom
    # catch floating point inaccuracy
    if res < -1:
        res = -1
    elif res > 1:
        res = 1
    # angle calculated in radians
    rad = math.acos(res)
    return rad


def prettify(angles):
    return "\n".join(["  %s: %s" % (k, v) for k, v in angles.iteritems()])

f = open(sys.argv[1] + '.txt', 'w')
f.write('*** Statistics ***\n\n')
f.write('Number of junctions:\t %d\n'%NumberOfJunctions(G))
f.write('Number of tips:\t\t %d\n'%NumberOfTips(G))
f.write('Total length:\t\t %f px\n'%TotalLength(G))
f.write('Average edge length:\t %f px\n'%AverageEdgeLength(G))
f.write('Average edge radius:\t %f px\n'%AverageEdgeRadius(G))
f.write('Total network area:\t %f px^2\n'%TotalNetworkArea(G))
f.write('Area of convex hull:\t %f px^2\n'%AreaOfConvexHull(G))
f.write('Number of cycles:\t %d\n'%NumberOfCycles(G))
for i in range(2, 5):
    f.write('Angles - nodes with %d neightbors:\n %s\n' % (i, prettify(Anglecalc(G,i))))
f.close()



