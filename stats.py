import numpy as np
import xml.etree.ElementTree as ET
import json
from collections import defaultdict
import argparse

# maybe generally useful? https://ops.fhwa.dot.gov/congestion_report/chapter2.htm

vCount = {}
stats = {}

PERSONS_PER_CAR = 1.67
PERSONS_PER_TRAM = 80 # max capacity per car: 125


def initStats():
    for vType in ["all_vehicles", "car", "bus", "bike", "ped", "tram"]:
        vCount[vType] = 0
        stats[vType] = {
            'waitingTime': [], # s, none for peds 
            'waitPercent': [],
            'timeLoss': [],    # s
            'aveSpeed': [],    # m/s
            'duration': []     # s   
        }

def parseStats():
    for node in root:
        if node.tag == 'tripinfo':
            # is vehicle, node.attrib['id']

            # <tripinfo id="vehicle732" 
            #         depart="732.00"
            #         departLane="a2_ns_12_2"
            #         departPos="1.70"
            #         departSpeed="0.00"
            #         departDelay="0.00"
            #         arrival="877.00"
            #         arrivalLane="b_diag_21_1"
            #         arrivalPos="126.97"
            #         arrivalSpeed="5.75"
            #         duration="145.00"
            #         routeLength="691.39"
            #         waitingTime="9.00"
            #         waitingCount="2"
            #         stopTime="0.00"
            #         timeLoss="26.68"
            #         rerouteNo="0"
            #         devices="tripinfo_vehicle732"
            #         vType="bike"
            #         speedFactor="0.86"
            #         vaporized=""/>

            vType = node.attrib['vType']
            aveSpeed = float(node.attrib['routeLength']) / float(node.attrib['duration'])

            for category in ['all_vehicles', vType]:
                vCount[category] += 1
                stats[category]['waitingTime'].append(float(node.attrib['waitingTime']))
                stats[category]['waitPercent'].append(float(node.attrib['waitingTime']) / float(node.attrib['duration']))
                stats[category]['timeLoss'].append(float(node.attrib['timeLoss']))
                stats[category]['aveSpeed'].append(aveSpeed)
                stats[category]['duration'].append(float(node.attrib['duration']))


        elif node.tag == 'personinfo':
            # is person

            # <personinfo id="person267" depart="267.00" type="DEFAULT_PEDTYPE">
         #        <walk depart="267.00"
         #            departPos="0.00"
         #            arrival="954.00"
         #            arrivalPos="0.00"
         #            duration="687.00"
         #            routeLength="847.13"
         #            timeLoss="77.07"
         #            maxSpeed="1.39"/>
         #    </personinfo>

            vCount['ped'] += 1
            vCount['all_vehicles'] += 1
            walk = node.find('walk')
            aveSpeed = float(walk.attrib['routeLength']) / float(walk.attrib['duration'])
            stats['ped']['timeLoss'].append(float(walk.attrib['timeLoss']))
            stats['ped']['aveSpeed'].append(aveSpeed)
            stats['ped']['duration'].append(float(walk.attrib['duration']))

def getCumulative():
    cumulativeStats = {
        'all_vehicles': defaultdict(dict),
        'car': defaultdict(dict),
        'bus': defaultdict(dict),
        'bike': defaultdict(dict),
        'ped': defaultdict(dict),
        'tram': defaultdict(dict)
    }
    for vType, info in stats.items():
        for statName, statList in info.items(): # waitingTime, timeLoss, etc. 
            print(statName)
            cumulativeStats[vType][statName]['mean'] = np.mean(statList)
            cumulativeStats[vType][statName]['std'] = np.std(statList)
            # cumulativeStats[vType][statName]['all'] = statList
    return cumulativeStats

def printStats(sim_type):
    with open(sim_type + '_stats.txt', 'w') as f:
        throughput = {
            'car': vCount['car']*PERSONS_PER_CAR,
            'bus': vCount['bus']*PERSONS_PER_BUS,
            'bike': vCount['bike'],
            'tram': vCount['tram']*PERSONS_PER_TRAM,
            'ped': vCount['ped'],
            'total': vCount['car']*PERSONS_PER_CAR +
                     vCount['bus']*PERSONS_PER_BUS +
                     vCount['bike'] +
                     vCount['tram']*PERSONS_PER_TRAM +
                     vCount['ped']
        }

        all_stats = {
            'vehicle_counts': vCount,
            'throughput': throughput,
            'cumulative_stats': getCumulative()
        }
        f.write(json.dumps(all_stats, indent=2))


parser = argparse.ArgumentParser()
parser.add_argument("sim_type", help="proposed or bau")
args = parser.parse_args()

if args.sim_type not in ["proposed", "bau"]:
    raise Error("Sim type must be 'proposed' or 'bau'.")


tree = ET.parse(args.sim_type + "-trips.xml")
root = tree.getroot()

PERSONS_PER_BUS = 66 if args.sim_type == "bau" else 40 # full capacity = ~70

initStats()
parseStats()
printStats(args.sim_type)

