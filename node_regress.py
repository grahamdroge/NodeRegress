'''
Implementation of Simulated Annealing to solving network-node route mapping regression
'''
import osmnx as ox 
import geopandas as gp 
import numpy as np 
import networkx as nx 
import pandas as pd
import utm 
import random 

from simanneal import Annealer 


rand_int = np.random.randint

# Location for place in Minneapolis and size of 
# area to consider
lat, lon = 44.977, -93.265
distance = 750

# Get networkx graph of location only considering 
# drive locations and add edge bearings
G = ox.graph_from_point((lat,lon), distance=distance, network_type='drive')
G = ox.project_graph(G)
G = ox.add_edge_bearings(G)

start_node = 33358310

# Add UTM coordinates to nodes
for n in G.nodes():
    G.node[n]['utm'] = utm.from_latlon(G.node[n]['y'],G.node[n]['x'])

# Get stats of network nodes
stats = ox.basic_stats(G)
avg_len = stats['street_length_avg']


def plot_route(net,route):
    '''
    Plot the given route on the network
    '''
    fig, ax = ox.plot_graph_route(net, route, node_color='w', node_size=20)

def plot_edge(u_,v_):
    '''
    Function to plot a specific edge in one color and 
    every other in another
    '''
    ec = ['b' if ((u==u_ and v==v_) or (u==v_ and v==u_)) else 'r' for u, v, k in G.edges(keys=True)]
    fig, ax = ox.plot_graph(G, node_color='w', node_edgecolor='k', node_size=20, node_zorder=3, edge_color=ec, edge_linewidth=2)


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
        start = G.node[route[n+1]]['utm']
        end = G.node[route[n]]['utm']

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


def dist_diff(end_node):
    '''
    Function to calculate the distance between nodes (As the crow flies)
    '''
    return ox.great_circle_vec(G.node[start_node]['y'], G.node[start_node]['x'],
                                G.node[end_node]['y'], G.node[end_node]['x'])



def sort_by_dist(net):
    '''
    Function to sort the nodes as distance from the start node 
    '''
    G_sort = sorted(G, key=dist_diff)

    return G_sort


def build_path(net):
    '''
    Function to build the next guess for path taken. Starting at start node
    the function randomly selects next node to take until the final node is reached
    '''
    node_list = [0,start_node]
    total_nodes = len(net.nodes())
    nodes_in_path = 1
    cur_node = start_node
    
    edge_check = 0

    while cur_node != end_node:
        if nodes_in_path > total_nodes / 2:
            return None
        next_node = random.choice([n for n in net[cur_node]])
        if next_node == node_list[-2]:
            if edge_check < 4:
                edge_check += 1
                continue
            else:
                edge_check = 0
                cur_node = start_node 
                nodes_in_path = 0
                continue
        else:
            node_list.append(next_node)
            cur_node = next_node
            nodes_in_path += 1

    return node_list


