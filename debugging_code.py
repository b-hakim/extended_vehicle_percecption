import traci
from sumolib import net
import cv2
import numpy as np

img = np.ones((500, 500, 3))*255

corners = [(100, 100), (100, 200), (150, 200), (150, 100)]
corners2 = [[1552.36628864,  530.04031442],
 [1552.36628864,  525.04031442],
 [1554.16628864,  525.04031442],
 [1554.16628864,  530.04031442]]


def rotate_vector(vec, ceta_degree, pivot):
    vec -= pivot.reshape((2, 1))
    ceta_rad = np.deg2rad(ceta_degree)
    rot_mat = np.array([[np.cos(ceta_rad), -np.sin(ceta_rad)],
                        [np.sin(ceta_rad), np.cos(ceta_rad)]])
    rotated_vector = np.dot(rot_mat, vec)
    rotated_vector += pivot.reshape((2, 1))
    return rotated_vector


def draw_poly(poly, color=(128, 128, 128)):
    poly = np.round(np.array(poly)).astype(int).reshape(1, -1, 2)
    cv2.fillPoly(img, poly, color)

    for i in range(len(corners)):
        cv2.line(img, corners[i - 1], corners[i], (0, 0, 0), thickness=2)

draw_poly(corners)
pivot = np.array([125, 100])
rotated_corners = rotate_vector(np.array(corners).transpose(), 45, pivot).transpose()
pivot = np.array([1553.2662886434614, 530.0403144201933])
rotated_corners2 = rotate_vector(np.array(corners2).transpose(), 107.94727590753621, pivot).transpose()
print(rotated_corners2)
draw_poly(rotated_corners, (255, 0, 0))

cv2.imshow("7amada!", img)
cv2.waitKey()
cv2.destroyAllWindows()



exit()

vehicle_ids = traci.vehicle.getIDList()

print("Speed", traci.vehicle.getSpeed(vehicle_ids[0]), "\n"
      "Position", traci.vehicle.getPosition(vehicle_ids[0]), "\n"
      "Pos 3D", traci.vehicle.getPosition3D(vehicle_ids[0]), "\n"
      "Accel", traci.vehicle.getAccel(vehicle_ids[0]), "\n"
      "Accelaration", traci.vehicle.getAcceleration(vehicle_ids[0]), "\n"
      "Angle", traci.vehicle.getAngle(vehicle_ids[0]), "\n"
      "Slope", traci.vehicle.getSlope(vehicle_ids[0]), "\n"
      "Height", traci.vehicle.getHeight(vehicle_ids[0]), "\n"
      "Length", traci.vehicle.getLength(vehicle_ids[0]), "\n"
      "Width", traci.vehicle.getWidth(vehicle_ids[0]), "\n"
      "Line", traci.vehicle.getLine(vehicle_ids[0]), "\n"
      "Route", traci.vehicle.getRoute(vehicle_ids[0]), "\n"
      "Road", traci.vehicle.getRoadID(vehicle_ids[0]), "\n")

# for road_id in traci.vehicle.getRoute(vehicle_ids[0]):
road_id = traci.vehicle.getRoadID(vehicle_ids[0])

if True:
    if road_id[0] == ':':
        polygon = net.getNode(road_id.split("_")[0][1:]).getShape()
        print("polygon:", polygon)
    else:
        polyline = net.getEdge(road_id).getShape()
        print("poly line:","'"+road_id+"'" , polyline)