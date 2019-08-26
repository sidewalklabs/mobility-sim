#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import print_function

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
    def __init__(self):
        # build a separate graph for each transit type
        self.ped_graph = nx.DiGraph()
        self.car_graph = nx.DiGraph()
        self.bike_graph = nx.DiGraph()
        self.tram_graph = nx.DiGraph()
        self.start_nodes = defaultdict(set)

        # map of vClass to transit graph
        self.graph_map = {
            "pedestrian": self.ped_graph,
            "passenger": self.car_graph,
            "bicycle": self.bike_graph,
            "tram": self.tram_graph
        }

        self.type_map = {
            "passenger": "car",
            "pedestrian": "ped",
            "bicycle": "bike",
            "tram": "tram"
        }


# for edge in net._edges:
#             self.graph.add_edge(edge._from._id, edge._to._id, id=edge._id)


    def load_graphs(self):
        net = sumolib.net.readNet('proposed.net.xml')
        
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
                        graph.add_edge(from_node, to_node, id=edge.getID()) # TODO: lots of repeats?? 

            # if node only has one neighbor, eligible as a "start node"
            if len(neighbors) == 1:
                for T in allowed_types:
                    self.start_nodes[T].add(nid)

        # temp get-around
        self.start_nodes["pedestrian"].add("B_NS_3")
        self.start_nodes["pedestrian"].add("L_EW_4")

        # for edge in net._edges:
        #     to_node, from_node = edge.getFromNode().getID(), edge.getToNode().getID()
        #     for vehicle_type, graph in self.graph_map.items():
        #         if edge.allows(vehicle_type):
        #             graph.add_edge(from_node, to_node, id=edge.getID())

        print(self.graph_map["pedestrian"].nodes)
        print(self.graph_map["passenger"].nodes)
        print(self.graph_map["bicycle"].nodes)
        print(self.graph_map["tram"].nodes)


    def generate_route_from_graph(self, vehicle_type):
        try:
            graph = self.graph_map[vehicle_type]
        except:
            raise("Vehicle type not currently supported in simulation.")

        start_node = random.sample(self.start_nodes[vehicle_type], 1)[0]
        while True:
            end_node = random.sample(self.start_nodes[vehicle_type], 1)[0]
            if end_node != start_node:
                break
        start_to_end = nx.shortest_path(graph, source=start_node, target=end_node)

        return self.nodes_to_edge_path(graph, start_to_end)


    def nodes_to_edge_path(self, graph, node_path):
        edge_path = []
        for i in range(1, len(node_path)):
            src = node_path[i-1]
            dest = node_path[i]
            edge_id = graph.get_edge_data(src, dest)["id"]
            edge_path.append(edge_id)
        return edge_path



# <routes>
#    <vType id="type1" accel="0.8" decel="4.5" sigma="0.5" length="5" maxSpeed="70"/>

#    <vehicle id="0" type="type1" depart="0" color="1,0,0">
#       <route edges="beg middle end rend"/>
#    </vehicle>

# </routes>


    def run(self):
        self.spawnrate=2
        simlen = 5000

        # while traci.simulation.getMinExpectedNumber() > 0:
        for count in range(0,simlen):
            # FOR PEDS
            # https://sumo.dlr.de/pydoc/traci._person.html

            # Spawn a vehicle
            if self.spawnrate > 0 and count%self.spawnrate == 0:
                # hack: make it more likely to spawn cars/peds than trams
                v_type = random.choice(["passenger", "passenger", "passenger",
                                       "pedestrian", "pedestrian", "pedestrian",
                                       "bicycle", "bicycle",
                                       "tram"])

                route_id = "route" + str(count)
                edges = self.generate_route_from_graph(v_type)

                if v_type != "pedestrian":
                    vehicle_id = "vehicle" + str(count)
                    traci.route.add(routeID=route_id, edges=edges)
                    traci.vehicle.add(vehID=vehicle_id, typeID=self.type_map[v_type], routeID=route_id) # typeID=self.type_map[v_type]
                else:
                    person_id = "person" + str(count)
                    traci.person.add(personID=person_id, edgeID=edges[1], pos=0)
                    traci.person.appendWalkingStage(personID=person_id, edges=edges[1:], arrivalPos=0)

            traci.simulationStep()
        traci.close()
        sys.stdout.flush()


# this is the main entry point of this script
if __name__ == "__main__":
    random.seed(4)

    m = SumoMain()
    m.load_graphs()

    sumo_binary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumo_binary, "-c", "proposed.sumocfg"])                   
    m.run()


