'''
Implementation of Simulated Annealing to solving network-node route mapping regression
'''
import osmnx as ox 
import geopandas as gp 
import numpy as np 
import networkx as nx 
import pandas as pd
import random 

from node_utils import *



def energy(net,route,true):
    '''
    Function that computes the energy of the state

    Parameters
        route: path estimate that we build
        true:  true path encoding from image/data 
    '''
    thresh = 20
    turn_info_est = get_turn_info(net,route,true,thresh)
    turn_info_true = true['turn_info']

    epsilon = .005
    n_iters = max(len(turn_info_est),len(turn_info_true))
    obj_fun = 0

    for i in range(n_iters):
        if i > (len(turn_info_true)-1) or i > (len(turn_info_est)-1):
            obj_fun += (n_iters - i)**2
        elif abs(turn_info_est[i][1] - turn_info_true[i][1]) > epsilon:
            obj_fun += (n_iters - i)**4

    return obj_fun 

def regress(net,true,start_node,end_node,n_iters,update_size,update_iter):
    '''
    Performs the regression to chain together nodes that minimize 
    the energy
    '''
    min_e = 1000000000000
    last_min = min_e
    min_route = []
    node_list = [] 
    start_node_ = start_node
    
    for i in range(n_iters):
        if i % update_iter == 0 and i != 0 and (last_min != min_e):
            last_min = min_e
            if len(min_route) < update_size:
                # node_list_test = node_list + min_route
                pass
            else:
                # node_list_test = node_list + min_route
                node_list.extend(min_route[:update_size])
                start_node_ = min_route[update_size]
                print("[{}] Current Min-e ==> {} {} {}".format(i,min_e,min_route[:update_size],start_node_))

        p = (.95 -  .35*(n_iters - i)/n_iters)
        path = build_path(net,start_node_,end_node,p)
        full_path = node_list + path
        e = energy(net,full_path,true)
        if e < min_e:
            min_e = e
            min_route = path
            if min_e == 0:
                print("[*] Found Min")
                break

    return (min_e,node_list + min_route)

# rand_int = np.random.randint
# p = .7

# Location for place in Minneapolis and size of 
# area to consider
lat, lon = 44.977, -93.265
distance = 750

# Get networkx graph of location only considering 
# drive locations and add edge bearings
G = ox.graph_from_point((lat,lon), distance=distance, network_type='drive')
# G = ox.graph_from_address('Omaha, Nebraska', network_type='drive', distance=2000)
G = ox.project_graph(G)
G = ox.add_edge_bearings(G)

# start_node = 582111787      # 33380473
# end_node = 757562572        # 33358310
start_node = np.random.choice(G.nodes())
end_node = np.random.choice(G.nodes())

# Add UTM coordinates to nodes
for n in G.nodes():
    # G.nodes[n]['utm'] = utm.from_latlon(G.nodes[n]['y'],G.nodes[n]['x'])
    G.nodes[n]['utm'] = (G.nodes[n]['x'],G.nodes[n]['y'])

# Get stats of network nodes
stats = ox.basic_stats(G)
avg_len = stats['street_length_avg']

# Base vector from start to end node in utm 
base_vec = np.array([ (G.nodes[end_node]['x'] - G.nodes[start_node]['x']), (G.nodes[end_node]['y'] - G.nodes[start_node]['y']) ])
for u,v,k,data in G.edges(keys=True,data=True):
    vec = get_vec(G,u,v)
    # ang = angle_between(base_vec,vec)
    dot = base_vec.dot(vec)
    data['dot_rel'] = -dot
    data['vec'] = vec


# Generate test path to match too
print("[*] Generating test path...")
while True:
    test_path = build_path(G, start_node, end_node, .85)
    if len(test_path) > 50:
        continue
    test_path_len = get_path_len(G, test_path)
    test_path_info = {'total_len': test_path_len, 'turn_info': get_turn_info(G, test_path, {'total_len': test_path_len}, 45)}
    break

plot_route(G, test_path)

# Go about solving
update_size =   3                       # number of nodes to lock after update iter
num_iters =     10000                   # number of iterations to run algorithm
update_iter =   300                     # Iteration to accept best energy path

minE, path = regress(G, test_path_info, start_node, end_node, num_iters, update_size, update_iter)
print("Min-E ==> ", minE)
plot_route(G, path)




