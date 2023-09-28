import copy
from types import SimpleNamespace
import pandas as pd
import vrplib
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import numpy.random as rnd

#from alns import ALNS
#from alns.accept import RecordToRecordTravel
#from alns.select import RouletteWheel
#from alns.stop import MaxRuntime

import yaml
import networkx as nx
import itertools
import math
from sklearn.cluster import KMeans
import random

from scipy.spatial import distance_matrix

#Loading the data from a YAML file

with open('C:/Users/ai_la/Repositories/UPF4ROS2/upf4ros2_demo/params/lookupTable.yaml') as file: 

    try:
        config = yaml.safe_load(file)
        print(config)
    except yaml.YAMLError as exc:
        print(exc)

#Creating a datafram from it

new_df= pd.DataFrame.from_dict(config['upf4ros2_navigation_action']['ros__parameters'])
new_df.index= ['Long','Lat']


#print(new_df)
#print(type(c_df),'\n',c_df)

#to create a distance matrix from the initial starting point to the target locations
#def create_Data_Model(no_of_drones,starting_point,df):
#    data={}
#    data['distance_matrix'] = df-data['depot'] #distances between locations --> this has to be fixed not correct right now
#    data["num_drones"] = no_of_drones
#    data["depot"] = starting_point #where all vehicles start and end their routes

#    return data

#def distance_Model(lat_from ,lat_to ,lon_from ,lon_to ): #calculates distance between two locations, will translate the input into array  

#    lon_from = radians(lon_from)
#    lon_to = radians(lon_to)
#    lat_from = radians(lat_from)
#    lat_to = radians(lat_to)
#
#    dlon = lon_to- lon_from
#    dlat = lat_to-lat_from
    
#    a = sin(dlat/2)**2 + cos(lat_from) *cos(lat_to)*sin(dlon/2)**2

#    c = a*asin(sqrt(a))

#    r = 6371 # radius of earth in kilometers

#    return(c*r) #calculate the result but this is the distance 

original_df = pd.DataFrame(new_df)

#Reshaping the original DataFrame

locations1= original_df.columns.tolist()

melted_df= pd.DataFrame()



for location in locations1:

    location_data= original_df[location]
    long,lat = location_data.tolist()
    location_df= pd.DataFrame({'Locations':[location],'Lat': [lat], 'Long': [long]}) 

    melted_df=pd.concat([melted_df,location_df])

#to reset the index
melted_df=melted_df.reset_index(drop=True)

print(melted_df)

#Creating a graph to represent locations and to calculate distances between them

G = nx.Graph()

#Adding nodes (locations) to the graph

for idx, row in melted_df.iterrows():
    G.add_node(row['Locations'], pos=(row['Lat'], row['Long']))

#Calculating the distancsas between nodes and add them as edges

for u, u_data in G.nodes(data=True):
    for v, v_data in G.nodes(data=True):
        if u != v:
            pos_u = u_data['pos']
            pos_v = v_data['pos']
            distance = math.sqrt((pos_u[0] - pos_v[0])**2 + (pos_u[1] - pos_v[1])**2)
            G.add_edge(u, v, weight=distance)

#Using Nearest Neighbor Algorithm to find a initial solution

def nearest_neighbor(graph, start_node):
    unvisited_nodes = set(graph.nodes)
    current_node = start_node
    unvisited_nodes.remove(current_node)
    tour = [current_node]
    total_distance = 0

    while unvisited_nodes:
        nearest_node = min(unvisited_nodes, key=lambda node: graph[current_node][node]['weight'])
        tour.append(nearest_node)
        total_distance += graph[current_node][nearest_node]['weight']
        current_node = nearest_node
        unvisited_nodes.remove(current_node)

    #Returning to the starting node to complete the cycle

    tour.append(start_node)
    total_distance += graph[current_node][start_node]['weight']

    return tour, total_distance

#to find the shortest path using Nearest Neighbor
shortest_distance = float('inf')
shortest_path = None

for node in G.nodes:
    tour, total_distance = nearest_neighbor(G, node)
    if total_distance < shortest_distance:
        shortest_distance = total_distance
        shortest_path = tour

# Print the result
#print("Shortest Path:", shortest_path)
#print("Total Distance (Euclidean distance):", shortest_distance)

#This part is for multiple vehicles shortest route calculation for the initial solution of ALNS, I clustered the locations within each vehicle to use TSP solver to find the shortest path

num_of_vehicles= 6 #this can also be taken from the yaml file/simulation

#Creating a new graph for the calculation of vehicle routes

g=nx.Graph()

for idx,row in melted_df.iterrows():
    G.add_node(row['Locations'], pos=(row['Lat'], row['Long']))

for u, u_data in G.nodes(data=True):
    for v, v_data in G.nodes(data=True):
        if u != v:
            pos_u = u_data['pos']
            pos_v = v_data['pos']
            distance = math.sqrt((pos_u[0] - pos_v[0])**2 + (pos_u[1] - pos_v[1])**2)
            G.add_edge(u, v, weight=distance)

#I used K-Means clustering to group locations into clusters for each vehicle

kmeans = KMeans(n_clusters=num_of_vehicles, random_state=0)

melted_df['Cluster'] = kmeans.fit_predict(melted_df[['Lat','Long']])

#we have the problem here -->PROBLEM SOLVED!

#Creating another list for storing paths of each vehicle

vehicle_paths= [[] for _ in range(num_of_vehicles)]

#To find the shortest path wihtin each cluster for each vehicle

for vehicle in range(num_of_vehicles):
    
    cluster_locations= melted_df[melted_df['Cluster']==vehicle]

    if len(cluster_locations)>0:

        subgraph = G.subgraph(cluster_locations['Locations'])

    #using tsp solver
        if subgraph.number_of_edges() >0:

            shortest_path1= nx.approximation.traveling_salesman_problem(subgraph,cycle=False)
            vehicle_paths[vehicle]= shortest_path1
        else:
            print(f"No edges found in subgraph for Vehicle {vehicle+1}")

else:
    print(f"No nodes found in cluster for vehicle {vehicle+1}")

#For printing the calculated routes for each vehicle

for vehicle, path in enumerate(vehicle_paths):

    print(f"Vehicle {vehicle+1} Route: {path}" ) 

#WORKS UNTIL HERE NO PROBLEMO :)
#Creating a destroy/repair operator for ALNS

#function to randomly select a vehicle --> later should be changed to the vehicle that has the highest path distance

def choose_vehicle(num_vehicles):
    return random.randint(0,num_of_vehicles-1)

#Destroy function that removes the nodes randomly

def destroy_nodes_random(vehicle_route,destroy_fraction=0.2):
    
    if not isinstance(vehicle_route,list):
        vehicle_route= [vehicle_route]

    num_nodes_to_destroy= max(1,int(len(vehicle_route)*destroy_fraction))

    if len(vehicle_route) == 0:

        print("Warning: Vehicle route is empty.")
        return [],[]

    destroyed_nodes= random.sample(vehicle_route,num_nodes_to_destroy)
    remaining_nodes=[node for node in vehicle_route if node not in destroyed_nodes]

    return destroyed_nodes,remaining_nodes


#Changed this from random removal to destroy the node which has highest distance from the starting point of the route

def destroy_nodes_distance(vehicle_route,graph,destroy_fraction=0.2):

    if not isinstance(vehicle_route,list):
        vehicle_route=[vehicle_route]

    num_nodes_to_destroy = max(1, int(len(vehicle_route) * destroy_fraction))  # Ensure at least 1 node is destroyed

    print("Vehicle Route: ", vehicle_route)

    if len(vehicle_route) ==0:
        print("Warning: Vehicle route is empty.")
        return [],[]
    
    starting_node= vehicle_route[0]
    print("Starting Node: ", starting_node)

    vehicle_route.sort(
        key=lambda node: graph[starting_node].get(node,{'weight': float('inf')})['weight']
    )


    #destroyed_nodes = random.sample(vehicle_route, num_nodes_to_destroy)

    print("Sorted Vehicle Route: ", vehicle_route)

    destroyed_nodes= vehicle_route[:num_nodes_to_destroy]
    remaining_nodes= vehicle_route[num_nodes_to_destroy:]

    #remove destroyed ones from route
    #for node in destroyed_nodes:
    #        vehicle_route.remove(node)

    return destroyed_nodes,remaining_nodes

#function to repair the solution by rerouting destroyed nodes using nearest

def repair_solution(destroyed_nodes, vehicle_routes, graph, selected_vehicle):

    vehicle_route = vehicle_routes.at[selected_vehicle, 'Route'][0]  # Extract the route from the DataFrame

        
    vehicle_route = vehicle_route.split('->')


    for node in destroyed_nodes:

        min_distance= float('inf')
        best_insertion_point = None

        for i in range(len(vehicle_route)+1):

            route_copy= vehicle_route[:]
            route_copy.insert(i,node)
            route_length= calculate_route_length(route_copy,graph)

            if route_length< min_distance:
                min_distance= route_length
                best_insertion_point=i

        vehicle_route.insert(best_insertion_point, node) 

    #To join the repaired route list back into a string

    repaired_route = '->'.join(vehicle_route)

    #Updating the vehicle routes with repaired route

    vehicle_routes.at[selected_vehicle, 'Route'] = [repaired_route] 
#to calculate total route length for the vehicle

def calculate_route_length(route,graph):
    
    total_length=0
    
    for i in range(len(route)-1):

        u,v = route[i], route[i+1]

        if u in graph and v in graph[u]:
            total_length += graph[u][v]['weight']
        #else:
        #    print(f"Edge ({u}, {v}) not found in the graph. ")

    return total_length


#to store vehicle routes

vehicle_routes = pd.DataFrame(columns=['Vehicle', 'Route'])

for vehicle, path in enumerate(vehicle_paths):
    vehicle_routes = pd.concat([vehicle_routes, pd.DataFrame({'Vehicle': [vehicle], 'Route': [path]})], ignore_index=True)



#to choose a random vehicle for the destroy and repair operations

selected_vehicle = choose_vehicle(num_of_vehicles)

#!!! chekck point

print(f"Selected Vehicle: {selected_vehicle+1}")

if selected_vehicle in vehicle_routes['Vehicle'].tolist():

    vehicle_index = vehicle_routes[vehicle_routes['Vehicle'] == selected_vehicle].index[0]
#to extract selected vehicle's route

    selected_vehicle_route = vehicle_routes.at[vehicle_index, 'Route']  # changed

#!!! check point

    print(f"Extracted Route: {selected_vehicle_route}")

#to check if the route exists for the selected vehicle

    if isinstance(selected_vehicle_route,list) and len(selected_vehicle_route) >0:
    
        if not isinstance(selected_vehicle_route,list):
            selected_vehicle_route= [selected_vehicle_route]
        print(f"Original Route for Vehicle {selected_vehicle+1}: {selected_vehicle_route}")

        print("Vehicle Index: ", vehicle_index)
        print("Selected Vehicle Route: ", selected_vehicle_route)

        destroyed_nodes, remaining_nodes= destroy_nodes_distance(selected_vehicle_route,G)

        print(f"Destroyed nodes: {destroyed_nodes}")

        repair_solution(destroyed_nodes, vehicle_routes, G, selected_vehicle)

        repaired_vehicle_route = vehicle_routes.loc[vehicle_routes['Vehicle'] == selected_vehicle, 'Route'].values[0]

        if len(repaired_vehicle_route)>0:
            if isinstance(repaired_vehicle_route,list):
                repaired_vehicle_route=repaired_vehicle_route[0]
            if not isinstance(repaired_vehicle_route,list):
                repaired_vehicle_route=[repaired_vehicle_route]

            print(f"Repaired Route for Vehicle {selected_vehicle+1}: {repaired_vehicle_route}")

        else:
            print(f"No repaired route found for Vehicle {selected_vehicle+1}")

    else:
        print(f"No route found for Vehicle {selected_vehicle+1}")


#Printing the calculated routes after the destroy/repair operations -->just to be sure 

for vehicle, path in enumerate(vehicle_paths):

    print(f"New Route for Vehicle {vehicle+1} Route: {path}" ) 

#++to visualize the routes and location points to be sure

def visualize_routes_and_locations(graph, vehicle_paths, locations_df):
    plt.figure(figsize=(10, 8))

    #Plotted the location points
    for idx, row in locations_df.iterrows():
        plt.scatter(row['Long'], row['Lat'], color='blue', s=100) #plotting locations as blue points

        plt.text(row['Long'], row['Lat'], row['Locations'], fontsize=12, ha='left', va='bottom') #Adding location names

    #Plotted the routes
    for vehicle, path in enumerate(vehicle_paths):
        route_coordinates = [graph.nodes[node]['pos'] for node in path]
        route_coordinates = list(zip(*route_coordinates)) 

        plt.plot(route_coordinates[1], route_coordinates[0], linestyle='-', marker='o', markersize=6) #0 contains the latitude, 1 contains the longitude

    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('VRP Routes and Locations')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.show()


#Just calling the visualization function
visualize_routes_and_locations(G, vehicle_paths, melted_df)
