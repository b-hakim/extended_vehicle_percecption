# input a map
# loop on all edges of the map >> add the area
# loop on all junctions >> add the area
# return the sum of the area
import os
import traci
import sumolib

from math_utils import get_polygon_area


def calculate_area(scenario_path):
    net = sumolib.net.readNet(os.path.join(scenario_path, "test.net.xml"))
    edges = net.getEdges()
    junctions = net.getNodes()

    # sumoCmd = ["/usr/bin/sumo", "-c", os.path.join(scenario_path, "net.sumo.cfg"), '--no-warnings', '--quit-on-end']
    # traci.start(sumoCmd)
    area = 0

    for edge in edges:
        if edge.allows('taxi'):
            area += edge.getLength()

    area2 = 0

    for junction in junctions:
        area2 += get_polygon_area(junction.getShape())

    return area + area2


if __name__ == '__main__':
    area1 = calculate_area("/media/bassel/Career/toronto_content_selection/toronto/toronto_0/0")
    area2 = calculate_area("/media/bassel/Career/toronto_content_selection/toronto/toronto_1/0")
    area3 = calculate_area("/media/bassel/Career/toronto_content_selection/toronto/toronto_2/0")


    print("traffic_density are", 100/area1, 200/area1, 300/area1)
    print("traffic_density are", 100/area2, 200/area2, 300/area2)
    print("traffic_density are", 100/area3, 200/area3, 300/area3)