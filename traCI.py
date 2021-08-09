import os, sys
import random
import numpy as np
import traci
import sumolib

from vehicle_info import Vehicle


def get_seen_vehicles(cv2x_vehicles_indexes, non_cv2x_vehicles_ids, vehicle_id_list, max_view_range, angle_view):
    """
    1- Use angle of view to get all vehicles in range
    2- Use vector from each cv2x_vehicle to all non-cv2x-vehicles
    3- if this vector passes through building or through 60% of the mid-vehicle or the distance is > 70m
        then skip this non-cv2x
    4- else, add this non-cv2x to the seen vehicle of this specific cv2x-vehicle
    """
    cv2x_vehicles = []
    cv2x_vehicles_perception = {}

    for cv2x_index in cv2x_vehicles_indexes:
        cv2x_id = vehicle_id_list[cv2x_index]

        cv2x_vehicle = Vehicle(future_routes=[],
                               dimension=(traci.vehicle.getWidth(cv2x_id),
                                          traci.vehicle.getLength(cv2x_id),
                                          traci.vehicle.getHeight(cv2x_id)),
                               pos=traci.vehicle.getPosition(cv2x_id),
                               speed=traci.vehicle.getSpeed(cv2x_id),
                               acc=traci.vehicle.getAcceleration(cv2x_id),
                               vehicle_id=cv2x_id, vehicle_angle_degree=traci.vehicle.getAngle(cv2x_id),
                               perception_range=max_view_range, perception_angle=angle_view)

        cv2x_vehicles.append(cv2x_vehicle)

        for non_cv2x_index in non_cv2x_vehicles_ids:
            non_cv2x_id = vehicle_id_list[non_cv2x_index]

            non_cv2x_vehicle = Vehicle(future_routes=[],
                                       dimension=(traci.vehicle.getWidth(non_cv2x_id),
                                                  traci.vehicle.getLength(non_cv2x_id),
                                                  traci.vehicle.getHeight(non_cv2x_id)),
                                       pos=traci.vehicle.getPosition(non_cv2x_id),
                                       speed=traci.vehicle.getSpeed(non_cv2x_id),
                                       acc=traci.vehicle.getAcceleration(non_cv2x_id),
                                       vehicle_id=non_cv2x_id, vehicle_angle_degree=traci.vehicle.getAngle(cv2x_id),
                                       perception_range=-1, perception_angle=-1)

            if cv2x_vehicle.can_see(non_cv2x_vehicle):
                is_occluded = False

                for non_cv2x_obstacle_index in non_cv2x_vehicles_ids:
                    non_cv2x_obstacle_id = vehicle_id_list[non_cv2x_obstacle_index]

                    if non_cv2x_obstacle_id != non_cv2x_id:
                        non_cv2x_obstacle_vehicle = Vehicle(future_routes=[],
                                                            dimension=(traci.vehicle.getWidth(non_cv2x_obstacle_id),
                                                                       traci.vehicle.getLength(non_cv2x_obstacle_id),
                                                                       traci.vehicle.getHeight(non_cv2x_obstacle_id)),
                                                            pos=traci.vehicle.getPosition(non_cv2x_obstacle_id),
                                                            speed=traci.vehicle.getSpeed(non_cv2x_obstacle_id),
                                                            acc=traci.vehicle.getAcceleration(non_cv2x_obstacle_id),
                                                            vehicle_id=non_cv2x_obstacle_id,
                                                            vehicle_angle_degree=traci.vehicle.getAngle(cv2x_id),
                                                            perception_range=-1, perception_angle=-1)

                        if cv2x_vehicle.object_in_sight(non_cv2x_obstacle_vehicle, non_cv2x_vehicle):
                            is_occluded = True
                            break

                if not is_occluded:
                    if cv2x_id in cv2x_vehicles_perception:
                        cv2x_vehicles_perception[cv2x_id].append(non_cv2x_id)  # non_cv2x_vehicle)
                    else:
                        cv2x_vehicles_perception[cv2x_id] = [non_cv2x_id]  # [non_cv2x_vehicle]

    return cv2x_vehicles_perception


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


def get_interest_cv2x_in_vehicle(cv2x_id, non_cv2x_id, p):
    # Traj = the trajectory of the cv2x vehicle.
    # Assume max velocity of cv2x = 40 m/s. Therefore, Traj for 5 sec has a max of 200m travel distance
    # Get trajectory from="noncv2x_vehicle" to differnet segment of "Traj".
    # Let noncv2x_vehicle speed = max(its own speed, 20 m/s),
    #           calculate the time taken to reach different segemnts of "Traj"
    #           ignore any time above 5 sec.
    #           minT = minimum time taken to reach one of the Traj segment
    #           p = probability the cv2x seeing the non_cv2x
    #           interest = (1/minT) * (p)
    pass



def calculate_scores_per_cv2x(cv2x_detected_vehicles, hyper_params):
    cv2x_ids = list(cv2x_detected_vehicles.keys())
    scores_per_cv2x = {}

    for cv2x_id, non_cv2x_ids in cv2x_detected_vehicles.items():
        current_cv2x_vehicle = Vehicle(future_routes=[],
                               dimension=(traci.vehicle.getWidth(cv2x_id),
                                          traci.vehicle.getLength(cv2x_id),
                                          traci.vehicle.getHeight(cv2x_id)),
                               pos=traci.vehicle.getPosition(cv2x_id),
                               speed=traci.vehicle.getSpeed(cv2x_id),
                               acc=traci.vehicle.getAcceleration(cv2x_id),
                               vehicle_id=cv2x_id, vehicle_angle_degree=traci.vehicle.getAngle(cv2x_id),
                               perception_range=hyper_params["view_range"], perception_angle=hyper_params["angle_view"])

        other_cv2x_ids = list(set(cv2x_ids) - set([cv2x_id]))
        scores = []

        for non_cv2x_id in non_cv2x_ids:
            for other_cv2x_id in other_cv2x_ids:
                p = current_cv2x_vehicle.get_probability_v1_sees_v2(cv2x_id, non_cv2x_id)
                scores.append(get_interest_cv2x_in_vehicle(other_cv2x_id, non_cv2x_id))

        scores_per_cv2x[cv2x_id] = scores


def calculate_highest_score_per_cv2x(scores_per_cv2x):
    pass


def select_max_scores(score_per_cv2x):
    pass


def run(hyper_params):
    sumoBinary = "/usr/bin/sumo-gui"
    sumoCmd = [sumoBinary, "-c", hyper_params['scenario_map']]
    traci.start(sumoCmd)
    step = 0

    np.random.set_state(('MT19937', np.array(
        [865610089, 4198412327, 1133904934, 3363913031, 1893987437, 1996961430, 2137660194, 4006379805, 327501811,
         3815180377, 2376961818, 1772596801, 944851375, 548165178, 2270277342, 2706830503, 2617350599, 3006002245,
         4181908613, 4261260859, 196567961, 79925413, 1746676643, 3727890402, 446599880, 472149946, 602121443,
         1252520769, 3124639840, 960294301, 93106996, 4219518374, 3634697492, 2588039451, 256080228, 2268737323,
         1055728372, 3847679623, 3511698904, 1116559708, 594039902, 489058063, 2615744158, 966017957, 1639799043,
         3539863208, 1224214071, 2868905713, 2448144826, 54200725, 3880860801, 908442676, 2618445308, 52846167,
         1386154935, 3207309101, 2330867320, 2960502955, 3938151340, 151650619, 2624202531, 2646119385, 109960821,
         3504847553, 3317187529, 3013593809, 2502102980, 1669317305, 1047623551, 1655241401, 3744871465, 2612091319,
         3971358871, 1577372346, 3594399454, 3736935771, 1282729693, 1782438699, 3658110162, 2659314343, 2337318849,
         1431664748, 3282596920, 3354085882, 1345591885, 3891976732, 2537838203, 2761318727, 1611656718, 2028427822,
         2658858746, 2128276284, 893471560, 2456771662, 2969280788, 2893037142, 3783507557, 3256147149, 1124431221,
         3549045324, 2897012706, 650124689, 353297105, 1960676108, 942431309, 2197983100, 3512139169, 1443169332,
         456721872, 2211534288, 895855146, 2115131363, 644531643, 3971902352, 499605399, 1439288280, 3699198916,
         2953171188, 598874906, 151559385, 807412022, 1857305578, 873635973, 2171610262, 4269514597, 1806997634,
         4288566577, 3125288046, 1430649012, 1974219618, 3382551341, 4053617755, 3055265346, 2536920176, 3586970963,
         1483562603, 3096440366, 2891663973, 1227589276, 167632411, 3352236849, 2655870427, 452224940, 967596125,
         1883052332, 3639058569, 2784965432, 3114345392, 379655256, 459290065, 187882586, 850396001, 2437666250,
         3437601915, 3045438034, 2403451559, 2360221223, 1347141098, 1381901798, 1115264392, 2514305900, 3581551907,
         2538564543, 3982416207, 1726949366, 2815410974, 1647905713, 2540492989, 1584791616, 3616355492, 3096780085,
         3793538093, 3349383197, 3628279573, 3541096200, 3112902061, 4268274489, 3228944908, 3439480837, 140465049,
         378420151, 1808755134, 153034551, 224550067, 4170141977, 1309950922, 1913995221, 4173703459, 2101782232,
         112811992, 2900060916, 1021246255, 2454781217, 710631874, 155465891, 2359784630, 1820467340, 3404721193,
         3878995755, 1732652352, 1613438685, 2821854604, 4046895318, 3040803544, 957218163, 3736380066, 4160512548,
         4167939147, 3727815799, 723388656, 2829643832, 2752587264, 3631071427, 3112604343, 2252461182, 874683428,
         4218234299, 2052939156, 1400974294, 2315572194, 4167567201, 3306426421, 178570315, 2772473702, 557800462,
         2338093906, 2310489360, 422675005, 3272582513, 255415318, 1779656839, 2342722311, 1169245775, 2376160361,
         3309035991, 435267994, 2880641146, 3582566481, 2468985197, 3748054866, 583549857, 2288704898, 3500457848,
         1784161716, 3857431810, 77325300, 3344879869, 2974543695, 4116508514, 390413005, 1156532212, 108876854,
         1163545427, 1294647525, 2831875758, 1559207291, 2679998804, 1381094830, 1532202671, 3463758392, 4029041496,
         1587150946, 1903716091, 4089424780, 2585245417, 3361809278, 2091211145, 1275891495, 2037123058, 47778262,
         3435358992, 2554772786, 1443262264, 3005307038, 2203410249, 2918930007, 2427259027, 601368928, 3676804292,
         3174613803, 3493091549, 3739830369, 309175649, 530613500, 4119839494, 1725398296, 3322364404, 707982001,
         146312273, 3388443231, 2839320279, 3269194071, 1039635849, 297872110, 2340924819, 50194958, 490316743,
         1414541647, 3351664509, 1917226925, 2637534507, 189926538, 1350448570, 1467169223, 1041466234, 2143971106,
         114222738, 2964357084, 1750862178, 1472205176, 186851072, 3970928892, 712888899, 1294681204, 862541298,
         1425229039, 2316903696, 440267557, 1187903230, 1514904159, 1164665474, 1005294868, 1964318569, 2549181930,
         2558634239, 470774304, 3396558448, 1703548654, 1036340539, 4107599819, 4044988770, 771006856, 2956373887,
         1458165990, 1162832075, 2166940171, 2004251039, 1688776640, 3099480565, 3246001281, 2829082185, 1688088305,
         2880030666, 3350103018, 2462446331, 1084564986, 1377172889, 1623897025, 1846372228, 3531521969, 3540398965,
         2536650801, 242386075, 377360129, 2343518404, 1105021393, 2106422308, 4218078277, 2602643298, 2276004365,
         2097808116, 3661015038, 2948422225, 1920365853, 3417060965, 2861684894, 3684690651, 2152111269, 629697846,
         3761517020, 3347610243, 496337785, 2257632377, 237966656, 2389866246, 1231067236, 3644728535, 2749860463,
         2201576738, 1036386957, 774434806, 3140524840, 3053436236, 3267859027, 2199777780, 1031827583, 948274809,
         3825462095, 713893924, 1117745767, 3270178750, 947715224, 446016956, 517021646, 2013697021, 3090527177,
         2055749569, 2966773, 2829646252, 193816057, 2714726694, 976997971, 2346520342, 2980845312, 3409174285,
         747007465, 2178103527, 1692511781, 1523800045, 3277664213, 3672507197, 3203331429, 3266977764, 3593567785,
         1975844064, 3175554323, 4146285002, 1672623053, 4105423448, 2666608765, 2933528435, 3240631303, 1374558509,
         297642062, 3723805405, 1819521388, 1938118718, 2559017915, 4061149145, 658286820, 3588478192, 258422532,
         3664752902, 4094766382, 2820573333, 3953404447, 1080429711, 1781189266, 3132762702, 511514873, 1760007438,
         3390494366, 2153227291, 3804093046, 1660953566, 2367035865, 2054945439, 1842714902, 24655505, 2514939161,
         339641154, 1551289530, 1692095653, 2994548613, 2157355656, 2848745039, 2138602232, 2991531803, 3889455629,
         192795806, 3761600458, 198880974, 147818001, 4071591291, 1655085718, 3297965041, 1460862620, 3563554596,
         1880545518, 660070499, 3103660128, 1164792750, 3989811384, 1303229742, 4219071520, 4157692914, 4253640193,
         2347321514, 273699470, 1229361490, 1112728386, 583646845, 3907496169, 3290591030, 2041361460, 1805364472,
         3349259112, 1870062626, 3931643160, 1089568037, 3111441530, 1910207727, 2158648642, 3971484972, 254902895,
         2673660936, 3425532682, 3597820870, 673956005, 2128005141, 3481551292, 584857192, 91232622, 1287841494,
         4277614615, 3736442209, 1724759060, 3550256488, 199243560, 2346489288, 588097540, 2982629715, 3072879791,
         3157474439, 1195834312, 4237114446, 2463795029, 1158581741, 428909454, 2169399578, 3801044312, 2898059071,
         1425331450, 4187848361, 335436886, 1474906922, 13353749, 2937681690, 4122441991, 2698251872, 1921337308,
         4033964850, 1347029477, 1316929693, 1008151689, 1107012408, 3722643651, 276510767, 4224038731, 1282743894,
         3202441790, 183829857, 4293097021, 1298132380, 2686116234, 1738202926, 3776706325, 489859878, 1117248143,
         938828694, 705167809, 3571325277, 3416549555, 2764180758, 1729112265, 1837484390, 805487037, 714448854,
         1208393103, 1020799579, 1357698204, 3330873149, 1172498619, 1023198696, 1172085939, 1998616899, 2608278377,
         2812205706, 3296415180, 441444428, 4147300331, 1019350838, 344808389, 805405122, 470655187, 41783785,
         1196342601, 3282177814, 3165005864, 1280927565, 1310333890, 686769164, 2772790107, 2228017487, 1070243997,
         3905042326, 381491891, 2792592455, 169791286, 1628535813, 2077127952, 1282005098, 2836319294, 2440268304,
         2892751845, 1153089529, 4215676265, 2773313145, 90242601, 769851680, 1180722112, 3782487031, 1092596806,
         3663960471, 3432564679, 3034272847, 1488021615, 703005220, 571536384, 2685851810, 2390542757, 1779261646,
         787238516, 974950271, 1461182985, 92911325, 560087797, 659679365, 1577492191, 103023888, 1700994443,
         4033001505, 2338101717, 3125582762], dtype="uint32"), 1, 0, 0.0))

    while step < 100000:
        step += 1
        print("Step:", step)
        # if traci.inductionloop.getLastStepVehicleNumber("1") > 0:
        #     traci.trafficlight.setRedYellowGreenState("0", "GrGr")

        traci.simulationStep()
        traci.route.getIDList()
        # print(vehicle_list)
        net = sumolib.net.readNet(hyper_params['scenario_path'])

        # 1) Get All Vehicles with Wireless
        vehicle_ids = traci.vehicle.getIDList()

        print("Speed", traci.vehicle.getSpeed(vehicle_ids[0]),
              "Position", traci.vehicle.getPosition(vehicle_ids[0]),
              "Pos 3D", traci.vehicle.getPosition3D(vehicle_ids[0]),
              "Accel", traci.vehicle.getAccel(vehicle_ids[0]),
              "Accelaration", traci.vehicle.getAcceleration(vehicle_ids[0]),
              "Angle", traci.vehicle.getAngle(vehicle_ids[0]),
              "Slope", traci.vehicle.getSlope(vehicle_ids[0]),
              "Height", traci.vehicle.getHeight(vehicle_ids[0]),
              "Length", traci.vehicle.getLength(vehicle_ids[0]),
              "Width", traci.vehicle.getWidth(vehicle_ids[0]),
              "Line", traci.vehicle.getLine(vehicle_ids[0]),
              "Route", traci.vehicle.getRoute(vehicle_ids[0]),
              "Road", traci.vehicle.getRoadID(vehicle_ids[0]))

        road_id = traci.vehicle.getRoadID(vehicle_ids[0])

        if road_id[0] == ':':
            polygon = net.getNode(road_id.split("_")[0][1:]).getShape()
            print("polygon:", polygon)
        else:
            polyline = net.getEdge(road_id).getShape()
            print("poly line:", polyline)

        if len(vehicle_ids) == 0:
            break

        else:
            snapshot = False
            if len(vehicle_ids) >= 50:
                if np.random.random() > 0.5:
                    snapshot = True

        if not snapshot:
            continue

        cv2x_len = int(hyper_params["cv2x_N"] * len(vehicle_ids))
        cv2x_vehicles_ids = random.sample(range(len(vehicle_ids)), cv2x_len)
        non_cv2x_vehicles_ids = list(set(range(len(vehicle_ids))) - set(cv2x_vehicles_ids))

        # 2) Get seen non-cv2x vehicles by each cv2x_vehicle
        cv2x_perceived_vehicles = get_seen_vehicles(cv2x_vehicles_ids, non_cv2x_vehicles_ids, vehicle_ids,
                                                    hyper_params["view_range"], hyper_params["fov"])

        # 3) Solve which info to send to base station
        # 3.1) Calculate required information
        scores_per_cv2x = calculate_scores_per_cv2x(cv2x_perceived_vehicles)

        # 3.2) Get Highest Score for each cv2x vehicle
        # score_per_cv2x = [scores[np.argmax(scores)], np.armax(scores) for cv2x, scores in scores_per_cv2x.items()]
        score_per_cv2x = calculate_highest_score_per_cv2x(scores_per_cv2x)

        # 3.3) Baseline to solve the problem and select which requests to send
        select_max_scores(score_per_cv2x)

        if snapshot:
            break

    input("Simulation ended, close GUI?")

    traci.close()


if __name__ == '__main__':
    hyper_params = {}
    hyper_params['scenario_path'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/test.net.xml"
    hyper_params['scenario_map'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto/net.sumo.cfg"
    hyper_params["cv2x_N"] = 0.25
    hyper_params["fov"] = 120
    hyper_params["view_range"] = 75
    run(hyper_params)
