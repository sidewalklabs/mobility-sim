import numpy as np
import xml.etree.ElementTree as ET
import json
from collections import defaultdict

tree = ET.parse('toronto-trips.xml')
root = tree.getroot()


vCount = {}
stats = {}

PERSONS_PER_CAR = 1.67
PERSONS_PER_TRAM = 50 # ???? ask willa

def initStats():
    for vType in ["car", "bike", "ped", "tram"]:
        vCount[vType] = 0
        stats[vType] = {
            'waitingTime': [], # s, none for peds 
            'waitPercent': [],
            'timeLoss': [],    # s
            'speedFactor': [], # none for peds
            'aveSpeed': []     # m/s
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

            # get count (all, per vtype) 
            # collect (all) 'depart' times - plot as graph later
            # collect (all, per vtype) 'waitingTime', 'waitingCount', 'timeLoss'
            # collect (all, per vtype) 'speedFactor' - queuelength? 
            # collect (all, per vtype) speed: 'routeLength'/'duration'

            vType = node.attrib['vType']
            vCount[vType] += 1
            aveSpeed = float(node.attrib['routeLength']) / float(node.attrib['duration'])
            stats[vType]['waitingTime'].append(float(node.attrib['waitingTime']))
            stats[vType]['waitPercent'].append(float(node.attrib['waitingTime']) / float(node.attrib['duration']))
            stats[vType]['timeLoss'].append(float(node.attrib['timeLoss']))
            stats[vType]['aveSpeed'].append(aveSpeed)


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
            walk = node.find('walk')
            aveSpeed = float(walk.attrib['routeLength']) / float(walk.attrib['duration'])
            stats['ped']['timeLoss'].append(float(walk.attrib['timeLoss']))
            stats['ped']['aveSpeed'].append(aveSpeed)

def getCumulative():
    cumulativeStats = {
        'car': defaultdict(dict),
        'bike': defaultdict(dict),
        'ped': defaultdict(dict),
        'tram': defaultdict(dict)
    }
    for vType, info in stats.items():
        for statName, statList in info.items(): # waitingTime, timeLoss, etc. 
            print(statName)
            if vType == 'car':
                print(statList)
            cumulativeStats[vType][statName]['mean'] = np.mean(statList)
            cumulativeStats[vType][statName]['std'] = np.std(statList)
    return cumulativeStats

def printStats():
    with open('all_stats_toronto.txt', 'w') as f:
        f.write('Vehicle counts:\n')
        f.write(json.dumps(vCount))
        f.write('\n')
        f.write('Throughput:\n')
        f.write('\tCar: {}\n'.format(vCount['car']*PERSONS_PER_CAR))
        f.write('\tBike: {}\n'.format(vCount['bike']))
        f.write('\tTram: {}\n'.format(vCount['tram']*PERSONS_PER_TRAM))
        f.write('\tPed: {}\n'.format(vCount['ped']))
        f.write('\tTotal: {}\n\n'.format(vCount['car']*PERSONS_PER_CAR +
                                         vCount['bike'] +
                                         vCount['tram']*PERSONS_PER_TRAM +
                                         vCount['ped']))

        f.write('Stats:\n')
        f.write(json.dumps(getCumulative(), indent=2))
        # f.write('\tCar (mean, var):\n')
        # f.write('{}, {}\n'.format(np.mean(stats[])))
        # f.write('\t\t')

initStats()
parseStats()
printStats()

