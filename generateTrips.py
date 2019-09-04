#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import print_function

import argparse
import math
import os
import sys
import json
import optparse
import subprocess
import random
import time
import threading
from pprint import pprint
import networkx as nx
from collections import defaultdict

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME"), "tools"))
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
import sumolib


class SumoMain:
    def __init__(self, sim_type):
        # build a separate graph for each transit type
        self.sim_type = sim_type
        self.ped_graph = nx.DiGraph()
        self.car_graph = nx.DiGraph()
        self.bike_graph = nx.DiGraph()
        self.bus_graph = nx.DiGraph()
        self.tram_graph = nx.DiGraph()
        self.start_nodes = defaultdict(set)

        # map of vClass to transit graph
        self.graph_map = {
            "pedestrian": self.ped_graph,
            "passenger": self.car_graph,
            "bicycle": self.bike_graph,
            "bus": self.bus_graph,
            "tram": self.tram_graph
        }

        self.type_map = {
            "passenger": "car",
            "bus": "bus",
            "pedestrian": "ped",
            "bicycle": "bike",
            "tram": "tram"
        }

        self.heavy_traffic = ['B_NS_1', 'B_NS_5', 'B_D1', 'B_EW_out']


        self.edge_weights = {
            'l': 0.94,    # laneway
            'a': 1,       # accessway
            't': 0.95,    # transitway
            'b': 0.9      # boulevard
        } 

        # (start, end) pairs
        self.tram_routes = [ 
            ('B_NS_1', 'B_NS_5'), # NS
            ('B_NS_5', 'B_NS_1'),

            ('B_EW_0', 'B_EW_out'), # WE boulevard
            ('B_EW_out', 'B_EW_0'),

            ('B_D1', 'B_EW_out'), # Diag
            ('B_EW_out', 'B_D1'),
        ]

        self.bus_routes = [
            ('T_EW_0', 'B_EW_out'), # WE transitway down
            ('B_EW_out', 'T_EW_0'),

            ('B_NS_1', 'B_EW_out'), # NSE
            ('B_EW_out', 'B_NS_1'),

            ('T_EW_0', 'B_D1'), # WE transitway up
            ('B_D1', 'T_EW_0'),
        ]

        self.PERSONS_PER_CAR = 1.6




    def load_graphs(self):
        net = sumolib.net.readNet(self.sim_type + '.net.xml')
        
        # Don't add these to our node list to avoid generating it as a start/destination
        # TODO: find a better way of doing this

        for node in net._nodes:
            nid = node.getID()
            neighbors = set()
            allowed_types = set()

            for edge in node.getIncoming() + node.getOutgoing():
                from_node, to_node = edge.getFromNode().getID(), edge.getToNode().getID()

                # add neighbors to neighbors set
                if to_node != nid: neighbors.add(to_node)
                if from_node != nid: neighbors.add(from_node)

                # add this node/edge to appropriate transit graphs
                for vehicle_type, graph in self.graph_map.items(): 
                    if edge.allows(vehicle_type):
                        allowed_types.add(vehicle_type)
                        graph.add_node(nid, id=nid)
                        edge_id = edge.getID()
                        graph.add_edge(from_node, to_node, id=edge_id, weight=self.edge_weights[edge_id[0]])

            # if node only has one neighbor, eligible as a "start node"
            if len(neighbors) == 1:
                for T in allowed_types:
                    self.start_nodes[T].add(nid)

        # temp get-around
        self.start_nodes["pedestrian"].add("B_NS_3")
        if self.sim_type == "proposed":
            self.start_nodes["pedestrian"].add("L_EW_4") # in bau, this street is one-way, can't start here

        # import pdb
        # pdb.set_trace()



    def weighted_start_nodes(self, vehicle_type):
        if vehicle_type != "pedestrian":
            return list(self.start_nodes[vehicle_type]) + self.heavy_traffic
        else:
            return list(self.start_nodes[vehicle_type])


#     <!--

#           B_NS_1         A_NS_1         A_NS_2         A_NS_3   B_D1
#             ||             ||             ||             ||      ||
#             ||             ||             ||             ||        ||
# T_EW_0 == B_NS_2 ======= T_EW_1 ======= T_EW_2 ======= T_EW_3 === T_EW_4
#             ||             ||             ||             ||          ||            
#             ||             ||             ||             ||            ||   
#           B_NS_3 ======= L_EW_1 ======= L_EW_2 ======= L_EW_3 ====== L_EW_4
#             ||             ||             ||             ||              ||
#             ||             ||             ||             ||                ||
# B_EW_0 == B_NS_4 ======= B_EW_1 ======= B_EW_2 ======= B_EW_3 ========== B_EW_4 == B_EW_out
#             ||             ||             ||             ||
#             ||             ||             ||             ||
#           B_NS_5         A_NS_1_X       A_NS_2_X       A_NS_3_X

# -->
    

    def generate_route_from_graph(self, vehicle_type, start_node=None, end_node=None):
        try:
            graph = self.graph_map[vehicle_type]
        except:
            raise("Vehicle type not currently supported in simulation.")

        if not start_node or not end_node:
            start_node = random.sample(self.weighted_start_nodes(vehicle_type), 1)[0]
            while True:
                end_node = random.sample(self.weighted_start_nodes(vehicle_type), 1)[0]
                if end_node != start_node:
                    break
            print(start_node, end_node)

        paths = [p for p in nx.all_shortest_paths(graph, source=start_node, target=end_node, weight='weight')]
        start_to_end = random.choice(paths)


        # with small random chance in BAU, force a car to take the local route
        if self.sim_type == "bau" and random.random() < 0.03:
            first_nodes = random.choice([["B_NS_1", "B_NS_2"], ["T_EW_0", "B_NS_2"], ["B_EW_0", "B_NS_4"], ["B_NS_5", "B_NS_4"]])
            local = nx.shortest_path(graph, source="B_NS_3", target="L_EW_4")
            print('GENERATING forced local path')
            if vehicle_type == "pedestrian":
                print('is pedestrian')
                start_to_end = first_nodes + local
            else:
                print('\t' + vehicle_type)
                start_to_end = first_nodes + local + random.choice([["T_EW_4", "B_D1"], ["B_EW_4", "B_EW_out"]])

        # TODO: fix (for some reason in BAU, the junction T_EW_4 does not generate any pedestrian crossings)
        # temp get-around: end all pedestrian trips at T_EW_4 
        if self.sim_type == "bau" and vehicle_type == "pedestrian":
            for err_node in ["T_EW_4", "L_EW_4"]:
                try:
                    err_idx = start_to_end.index(err_node)
                    start_to_end = start_to_end[:err_idx+1] # end at T_EW_4
                except ValueError:
                    pass

        return self.nodes_to_edge_path(graph, start_to_end)


    def nodes_to_edge_path(self, graph, node_path):    
        edge_path = []
        for i in range(1, len(node_path)):
            src = node_path[i-1]
            dest = node_path[i]
            edge_id = graph.get_edge_data(src, dest)["id"]
            edge_path.append(edge_id)
        return edge_path

    def add_planned_vehicle(self, v_type, route_id, count):
        if v_type == "tram":
            graph = self.graph_map["tram"]
            endpoints = self.tram_routes[route_id]
        elif v_type == "bus":
            graph = self.graph_map["bus"]
            endpoints = self.bus_routes[route_id]
        path = nx.shortest_path(graph, source=endpoints[0], target=endpoints[1], weight='weight')
        edges = self.nodes_to_edge_path(graph, path)
        vehicle_id = v_type + str(route_id) + "_" + str(count)
        traci_route_id = "route_" + v_type + str(route_id) + "_" + str(count)
        traci.route.add(routeID=traci_route_id, edges=edges)
        traci.vehicle.add(vehID=vehicle_id, typeID=v_type, routeID=traci_route_id) # typeID=self.type_map[v_type]


# BAU: how many more cars?

# number of people who are driving cars in BAU who would have taken tram in proposed:
# each tram runs every 5min for one hour = 12 trips per tram, 6 tram routes, ~80 person/tram = 5760
# say in BAU, 20% will take bus (PERSONS_PER_BUS += 16), 80% take car: (0.8 * 5760)/1.67 = 2760 more cars
# so at each second of BAU, there is a ~76% of generating another car


    def run(self):
        self.spawnrate=2
        simlen = 3600 # one hour

        # while traci.simulation.getMinExpectedNumber() > 0:
        for count in range(0,simlen):

            # add tram/bus if it's time to add tram/bus
            # timing: evenly space out 6 buses and 6 trams over 5min (in proposed), in BAU only buses:
            if self.sim_type == "proposed" and (count+10) % 50 == 0: 
                route_id = (count // 50) % 6
                self.add_planned_vehicle("tram", route_id, count)
            elif (count+35) % 50 == 0:
                route_id = (count // 50) % 6
                self.add_planned_vehicle("bus", route_id, count)

            # Spawn 1-3 vehicles
            if count % self.spawnrate == 0:
                is_bau = 1 if self.sim_type == "bau" else 0
                num_to_generate = random.randint(1,3)
                if self.sim_type == "bau" and random.random() < 0.76: # see calculation above
                    num_to_generate += 1

                # if proposed, show we can fill laneway with a lot more pedestrians at no cost to other modes - these guys just hanging around
                if self.sim_type == "proposed" and random.random() < 0.15:
                    trip_id = str(count) + "_lane"
                    route_id = "route" + trip_id
                    endpoints = ["B_NS_3", "L_EW_4"] if random.random() < 0.5 else ["L_EW_4", "B_NS_3"]
                    edges = self.generate_route_from_graph(v_type, start_node=endpoints[0], end_node=endpoints[1])
                    person_id = "person" + trip_id
                    traci.person.add(personID=person_id, edgeID=edges[0], pos=0)
                    traci.person.appendWalkingStage(personID=person_id, edges=edges[1:], arrivalPos=0)

                for i in range(num_to_generate):

                    r = random.random()
                    if self.sim_type == "bau":
                        v_type = "passenger" if r < 0.75 else "pedestrian"
                    else:
                        if r < 0.55:
                            v_type = "passenger"
                        elif r < 0.7:
                            v_type = "pedestrian"
                        else:
                            v_type = "bicycle"

                    if random.random() < (1/1.6)

                    trip_id = str(count) + "_" + str(i)
                    route_id = "route" + trip_id
                    edges = self.generate_route_from_graph(v_type)

                    if v_type != "pedestrian":
                        vehicle_id = "vehicle" + trip_id
                        traci.route.add(routeID=route_id, edges=edges)
                        traci.vehicle.add(vehID=vehicle_id, typeID=self.type_map[v_type], routeID=route_id) # typeID=self.type_map[v_type]
                    else:
                        if len(edges) > 1:
                            person_id = "person" + trip_id
                            traci.person.add(personID=person_id, edgeID=edges[0], pos=0)
                            traci.person.appendWalkingStage(personID=person_id, edges=edges[1:], arrivalPos=0)

            traci.simulationStep()
        traci.close()
        sys.stdout.flush()


# this is the main entry point of this script
if __name__ == "__main__":
    random.seed(4)

    parser = argparse.ArgumentParser()
    parser.add_argument("sim_type", help="proposed or bau")
    args = parser.parse_args()

    if args.sim_type not in ["proposed", "bau"]:
        raise Error("Sim type must be 'proposed' or 'bau'.")

    m = SumoMain(args.sim_type)
    m.load_graphs()

    sumo_binary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumo_binary, "-c", args.sim_type + ".sumocfg"])                   
    m.run()


