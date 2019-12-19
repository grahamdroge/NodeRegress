'''
Various utility functions for handling the network of nodes 
For node regression
'''
import osmnx as ox 
import math 
import numpy as np

def plot_route(net,route):
    '''
    Plot the given route on the network
    '''
    fig, ax = ox.plot_graph_route(net, route, node_color='w', node_size=20)

def plot_edge(net,u_,v_):
    '''
    Function to plot a specific edge in one color and 
    every other in another
    '''
    ec = ['b' if ((u==u_ and v==v_) or (u==v_ and v==u_)) else 'r' for u, v, k in net.edges(keys=True)]
    fig, ax = ox.plot_graph(net, node_color='w', node_edgecolor='k', node_size=20, node_zorder=3, edge_color=ec, edge_linewidth=2)


def get_path_len(net,route):
    '''
    Function to calculate the total length of the path
    '''
    total_len = 0

    for i in range(len(route)- 1):
        edge = (route[i],route[i+1],0)
        tmp_len = net.edges[edge]['length']

        total_len += tmp_len 

    return total_len


def get_angle(s,e):
    '''
    Function to get the angle of the edge in common x-y 
    coordinate system
    '''
    # e = G.node[end]['utm']
    # s = G.node[start]['utm']
    xdif = e[0] - s[0]
    ydif = e[1] - s[1]
    ang = math.atan(ydif/xdif) * 180/math.pi

    return ang

def encode_angles(net,route):
    '''
    Function to encode angle differences
    '''
    prev_ang = 0
    enc_val = 1

    for n in range(len(route)-1):
        start = G.nodes[route[n+1]]['utm']
        end = G.nodes[route[n]]['utm']

        ang = get_angle(start,end)
        diff = abs(round(ang - prev_ang))
        prev_ang = ang 

        if diff > 20:
            if n == 0:
                continue
            elif enc_val == 0:
                enc_val += 1
            else:
                enc_val = enc_val << 1
            print("[Turn] \t--> BP-{} = 1".format(n))
        else:
            print("[Srt] \t--> BP-{} = 0".format(n))

    print("")
    print("Enc Val: {}  # Turns: {}  Binary: {}".format(enc_val,np.log2(enc_val),bin(enc_val)))

    return enc_val


def sort_by_dist(net,start_node,nodes):
    '''
    Function to sort the nodes as distance from the start node 
    '''
    dist_diff = lambda end_node: ox.great_circle_vec(net.nodes[start_node]['y'], net.nodes[start_node]['x'],
                                net.nodes[end_node]['y'], net.nodes[end_node]['x'])
                                
    G_sort = sorted(nodes, key=dist_diff)

    return G_sort


def get_vec(net,s,e):
    '''
    Function to compute vector pointing from start to end in 
    utm
    '''
    return np.array([ (net.nodes[e]['x'] - net.nodes[s]['x']), (net.nodes[e]['y'] - net.nodes[s]['y']) ])

def unit_vector(vector):
    '''
     Returns the unit vector of the vector.
     '''
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    '''
    Returns the angle in degrees between vectors 'v1' and 'v2'
    '''
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)) * 180/math.pi


def get_turn_info(net,route,true,thresh):
    '''
    Function to compute number of turns in route.  Turn determined by 
    relative angle threshold
    '''
    total_len = true['total_len']
    turn_info = []
    num_turns = 0
    acc_len = 0
    last_edge_vec = np.array([0,0])

    for i in range(len(route)-1):
        edges = net.out_edges(route[i],data=True)
        for e in edges:
            if e[1] == route[i+1]:
                if not last_edge_vec.any():
                    last_edge_vec = e[2]['vec']
                    acc_len += e[2]['length']
                elif angle_between(e[2]['vec'],last_edge_vec) > thresh:
                    num_turns += 1
                    turn_info.append((num_turns,acc_len/total_len))
                    last_edge_vec = e[2]['vec']
                    acc_len += e[2]['length']
                    break
                else:
                    acc_len += e[2]['length']

    return turn_info   


def build_path(net,start_node,end_node,p):
    '''
    Function to build the next guess for path taken. Starting at start node
    the function *randomly* selects next node to take until the final node is reached
    '''
    node_list = [0,start_node]
    cur_node = start_node
    
    sort_dir = lambda e: -np.dot( unit_vector(e[2]['vec']) , unit_vector(get_vec(net,e[0],end_node)) )

    while ((cur_node != end_node) and (len(node_list) <  50)):
    
        edge_neighbors = net.edges(cur_node,data=True)
        edges_sort = sorted(edge_neighbors, key=sort_dir)

        if len(edges_sort) == 0:
            cur_node = node_list[-2]
            _ = node_list.pop()
            continue
        elif len(edges_sort) == 1:
            cur_node = edges_sort[0][1]
            if cur_node == node_list[-2]:
                cur_node = start_node
                node_list = [0,cur_node]
                continue 
            node_list.append(cur_node)
            continue
              
        best, others = edges_sort[0],edges_sort[1:]

        if np.random.choice([0,1], p=[(1-p),p]):
            if best[1] == node_list[-2]:
                cur_node = others[0][1]

            else:
                cur_node = best[1]
            
        else:
            next_index = np.random.randint(len(others))
            if others[next_index][1] == node_list[-2]:
                continue
            cur_node = others[next_index][1]

        node_list.append(cur_node)

    return node_list[1:]

