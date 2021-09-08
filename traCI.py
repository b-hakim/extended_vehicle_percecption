import os, sys
import random
from typing import List, Dict

import numpy as np
import traci
import sumolib

from math_utils import euclidean_distance
from solver import Solver
from sumo_visualizer import SumoVisualizer
from vehicle_info import Vehicle


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


class Simulation:
    def __init__(self, hyper_params):
        self.hyper_params = hyper_params

    def get_seen_vehicles(self, cv2x_vehicles, non_cv2x_vehicles, buildings):
        """
        1- Use angle of view to get all vehicles in range
        2- Use vector from each cv2x_vehicle to all non-cv2x-vehicles
        3- if this vector passes through building or through 60% of the mid-vehicle or the distance is > 70m
            then skip this non-cv2x
        4- else, add this non-cv2x to the seen vehicle of this specific cv2x-vehicle
        """
        cv2x_vehicles_perception = {}

        for cv2x_vehicle in cv2x_vehicles:
            for non_cv2x_vehicle in non_cv2x_vehicles:
                if cv2x_vehicle.can_see_vehicle(non_cv2x_vehicle):
                    is_occluded = False

                    for building in buildings:
                        if cv2x_vehicle.building_in_sight(building.shape, non_cv2x_vehicle):
                            is_occluded = True
                            break

                    if not is_occluded:
                        for non_cv2x_obstacle_vehicle in non_cv2x_vehicles+cv2x_vehicles:
                            if non_cv2x_obstacle_vehicle.vehicle_id != non_cv2x_vehicle.vehicle_id and \
                                non_cv2x_obstacle_vehicle.vehicle_id != cv2x_vehicle.vehicle_id:
                                if cv2x_vehicle.vehicle_in_sight(non_cv2x_obstacle_vehicle, non_cv2x_vehicle):
                                    is_occluded = True
                                    break

                    if not is_occluded:
                        if cv2x_vehicle.vehicle_id in cv2x_vehicles_perception:
                            cv2x_vehicles_perception[cv2x_vehicle.vehicle_id].append(non_cv2x_vehicle.vehicle_id)
                        else:
                            cv2x_vehicles_perception[cv2x_vehicle.vehicle_id] = [non_cv2x_vehicle.vehicle_id]

        return cv2x_vehicles_perception

    def get_interest_cv2x_in_vehicle(self, cv2x_vehicle, non_cv2x_vehicle, p):
        # Traj = the trajectory of the cv2x vehicle.
        # Assume max velocity of cv2x = 40 m/s. Therefore, Traj for 5 sec has a max of 200m travel distance

        # Get trajectory from="noncv2x_vehicle" to differnet segment of "Traj".
        # Let noncv2x_vehicle speed = max(its own speed, 40 m/s),
        #           calculate the time taken to reach different segemnts of "Traj"
        #           ignore any time above 5 sec.
        #           minT = minimum time taken to reach one of the Traj segment
        #           p = probability the cv2x seeing the non_cv2x
        #           interest = (1/minT) * (p)
        cv2x_future_edges = cv2x_vehicle.get_future_route()
        min_dist, min_dist_edge = self.get_shortest_route(non_cv2x_vehicle.get_current_road(), cv2x_future_edges)
        t = 3600 * (min_dist/40000) # get time in seconds for a speed of 40km/h

        if t > 5:
            return 0
        else:
            return p/t

    def calculate_scores_per_cv2x(self, cv2x_perceived_non_cv2x_vehicles,
                                  cv2x_vehicles, non_cv2x_vehicles,
                                  buildings):
        cv2x_ids = list(cv2x_vehicles.keys())
        scores_per_cv2x = {}

        for sender_cv2x_id, perceived_non_cv2x_ids in cv2x_perceived_non_cv2x_vehicles.items():
            sender_cv2x_vehicle = cv2x_vehicles[sender_cv2x_id]
            other_cv2x_ids = list(set(cv2x_ids) - set([sender_cv2x_id]))
            scores = []

            for receiver_cv2x_id in other_cv2x_ids:
                for perceived_non_cv2x_id in perceived_non_cv2x_ids:
                    remaining_perceived_non_cv2x_ids = list(set(perceived_non_cv2x_ids)-set([perceived_non_cv2x_id]))
                    remaining_perceived_non_cv2x_vehicles = [non_cv2x_vehicles[i] for i in remaining_perceived_non_cv2x_ids]

                    receiver_cv2x_vehicle = cv2x_vehicles[receiver_cv2x_id]
                    perceived_non_cv2x_vehicle = non_cv2x_vehicles[perceived_non_cv2x_id]

                    p = sender_cv2x_vehicle.get_probability_cv2x_sees_non_cv2x(receiver_cv2x_vehicle, perceived_non_cv2x_vehicle,
                                                                                remaining_perceived_non_cv2x_vehicles, buildings)
                    if p == 0:
                        scores.append((receiver_cv2x_id,0))
                    else:
                        scores.append((receiver_cv2x_id,
                                       self.get_interest_cv2x_in_vehicle(receiver_cv2x_vehicle, perceived_non_cv2x_vehicle, p)))

            scores_per_cv2x[sender_cv2x_id] = scores
        return scores_per_cv2x

    def get_shortest_route(self, source, list_destination_edges):
        min_distance = 100000000000
        min_dist_destination = None

        for destination in list_destination_edges:
            route = traci.simulation.findRoute(source, destination, "")

            if route.length < min_distance:
                min_distance = route.length
                min_dist_destination = destination

        # route_distance = 0

        # for edge_id in route.edges:
        #     points = net.getEdge(edge_id).getShape()
        #
        #     for i in range(1, len(points)):
        #         d = euclidean_distance(points[i], points[i-1])
        #         route_distance += d

        # return route_distance
        return min_distance, min_dist_destination

    def make_unique_requests(self, score_per_cv2x):
        receivers = {}

        for sender_cv2x_id, receiver_score in score_per_cv2x.items():
            receiver_cv2x, score = receiver_score

            if receiver_cv2x in receivers:
                if receivers[receiver_cv2x] < score:
                    receivers[receiver_cv2x] = score
            else:
                receivers[receiver_cv2x] = score

        return receivers

    def get_channel_gain_vehicle(self, tx_pos, receiver_vehicles, K):
        h_n = {}
        # wavelength = 5.5*10**-7
        # d0_squared = 10000
        # wavelength_squared = 3.025e-13

        y = 4

        for vehicle in receiver_vehicles:
            distance = euclidean_distance(tx_pos, vehicle.pos)
            # h_n_k = (distance**-4)*random.random()*np.random.exponential(1)
            tmp = []
            # this approach is canceled as it is more accurate and detailed but related to the exact wavelengths
            # changed by approach 2 (as sent in the email)
            # d0 = 100
            # L = 20 * np.log10(wavelength / (4 * np.pi * d0))

            for i in range(K):
                # r = 10 + random.random() * 90 # random number between 10 to 100
                # channel_rand = random.randint(80, 120)/100
                # h_n_k = channel_rand * L * (d0 / distance) ** y
                h_n_k = np.random.exponential(1)/distance**y
                # h_n_k = 1
                tmp.append(h_n_k)

            h_n[vehicle.vehicle_id] = (tmp)

        return h_n

    def run(self):
        sumoBinary = "/usr/bin/sumo"
        sumoCmd = [sumoBinary, "-c", self.hyper_params['scenario_map']]
        traci.start(sumoCmd)
        step = 0
        viz = SumoVisualizer(self.hyper_params)

        np.random.set_state(('MT19937', np.array([2147483648, 2443153948, 1663351052, 1980051762, 3291370578,
               3595348591, 1175957231, 1414814339, 2491694056, 4065556944,
                237650402, 3331964654, 4205689457, 4109326540, 4120309262,
               1217117340, 4294132543, 1968066253, 3320027787, 1641163237,
                741726857, 2247743962, 1667471451, 1056730411, 2185974503,
               3493401530,  603639794, 2125077469, 2833837812,  815040439,
                914039459, 3466091407, 2095338472, 1251443732, 3462536138,
                545981884,  668533285, 1476071640, 3791179136, 1523624455,
               4085295468, 2089568783, 1688403434, 1169407940, 1590651539,
               1692414915,  806231830, 3962439225, 3901386950, 4259199837,
               1283110546, 1873036691,  830778808, 1552923564, 1890061328,
               2615619120, 2051146862,  294371080, 3146656774, 3775465086,
                619900782,  925712654,  242484780, 3462211914, 2173152011,
               1996287871, 1945255130, 1625188329,  510400272, 2875805002,
               3066002247, 3335906306, 1323525775, 2130583421, 1874709365,
               1088941200, 3681301168, 2358008495, 3717233928, 1626889455,
               3776853202, 2784949373, 3945791303,  643734121, 4001755449,
                186797313, 2122210886, 1072909348, 1160400006,  325618411,
               3443756107, 3356656997, 1817375566, 1795263010, 3885088329,
               4130512677, 4120010142, 2188301037, 2495077919, 3806854096,
               3607550458, 2000963175, 2778608870, 3286678321, 2115644918,
                290271694, 1212343818, 4005336267, 3127740209, 2189001562,
                995561295, 3766836882, 2441453769,  964555262, 3807251718,
                207065696, 1911039188, 1646473984, 4117320913, 1862155049,
               2660587961, 1784541918, 2932277839, 1269314783,  905642343,
               2333263665, 3436708224, 4150938749,  619218788,  521405591,
               2907416931, 2815188219, 3868554473, 2992852544, 1823104594,
               3596291212,  355640414, 1626614647, 2198796998, 3249663430,
                572334273, 3727487503, 2015911344, 2690920563, 2844579648,
                348800841, 2567508065, 3385439742, 2938697620, 1492124883,
               2474595911, 1660839209,  252206500, 3744423084, 1431438170,
                557621323, 2269769557, 2652678463,  929343932, 1744557804,
               1419560336, 1406448983,  339241872, 2580109566,  623551190,
               2223621034,  339305025, 1101319539, 1978177796, 2544394539,
               3234513794,  993400129,  847598809,  525912187,  704581863,
               2328499421, 1629895530, 2465614965,  821924885, 1681865224,
               1605738889, 3514611295,  524245383,  416117018,  437860917,
               3795684542, 2204435181, 2790233400, 3721187408,  387438900,
               3970596594,  789257244, 3767573532,  486648962, 3044150362,
               3364952998, 3441247841, 2667094235, 1069017167,  861822345,
               1425677974, 1110139816,  766649210,  315060912, 3240184573,
               3889711611, 3584851764, 1370628517,   27571816,  815683869,
               3357439606, 3955825662, 1915704211,  920365304, 1472757232,
               1740331116, 2637903511, 1118444329,  747423620, 3436596641,
               1674195670,  413049277,  906155482, 2475759238, 2986171377,
                712555259, 3148271374, 3342955126, 2267715542, 1777262797,
               2659796465, 3516333377, 2594500424,  365145408,  740029523,
                307090490, 1073103018,  167924513,  249704698, 1210825276,
               3988455158, 2150205610, 2027675955, 3255177265, 1952957275,
               1449459527, 1727532352,  831209564,  860215913, 4228893836,
               1812494976, 1873435869, 2232243089, 2051759720, 2670315633,
               3523532051, 3539613558,   52058265, 2370118509, 1505009283,
               3072574303, 1466565449, 1159133678, 3082500460, 4082315327,
               3204865157, 1066163268,  652701328, 1320450664, 1214630982,
               1386118406,  872085485,  442530476, 2349145990, 3997293865,
               1712881199, 1230046189, 2385754640, 3437986771, 2005247058,
               3374361996, 2091476778, 1792944854,  262290133,   41953436,
               1194510353, 2666575400, 3531814804,  562774663,  506149032,
               2956007833, 3954810510, 3355722232, 3260931454, 2228110518,
               2689163422, 2277599533,  700688192, 1867678234, 2358507236,
               3714798903, 1121611257, 2446643916, 3579670346, 3103462583,
               3817012481, 1456920638, 3066797308, 2283112489, 4158736578,
               3071846304, 2490470822, 2915831776, 1603492112,  925375648,
                907235684,  707335605, 3891505616, 3325549167,  747162527,
               2330678375, 3257349028, 3149387935, 2999170548, 2650759299,
               2928087204, 4179155168, 3324437745,  226115862, 2405512930,
                635165971, 1077132356,  706963455, 3605952717, 3300405280,
               2693209232,  818891011, 2489662950,  397475800,  387750699,
               1672994827, 2501675839,  251788089, 1911063103, 2316478398,
               1334296068, 3453262044, 1804838628, 2613405913,  471835964,
               1373256706,  471180546,  600600512, 1109519010, 3449361201,
               3784149178,  379151461, 3610959385, 3103366529, 2776980818,
                900940383, 4057499995, 3909200713,  928238884, 1152579054,
                569571537, 1686596767, 1424448240, 4033815843, 1202546478,
                193775716,  275156561, 1968823397,  898604204, 2436736133,
                967384190, 2662491550, 3500417063,  104662709, 1863852143,
               1398623169,  491272555, 1535097383,  415081386, 1478762408,
               2925518300, 3062066859, 2654561550, 4262070390,  108122706,
               3154230009, 2405672372,  350548360, 3403114836, 2530237208,
               2697616573, 2869427675, 2517799473, 4068772636, 2045798887,
                203046614,  226764295,  278744565, 4111448376, 3993871025,
               2321202809,  755840053, 2160690974, 1294224264,  823061133,
               2823619141,  312349673, 2453491233,  757735945,  382837769,
               3431071126,  168865564,  418817653, 1588399365, 2371734634,
                715641179, 3673146828, 3234384560,  550349850, 2896772165,
               1520702503, 3651309290, 1549496876, 2007121406, 2669031477,
               2045867241, 3907454704, 4043243186, 1087796821,  496084345,
               4092603847, 3534401686, 1090420701, 3880140286, 3828932079,
               1963929891,   24430583, 1951982032, 3385412286, 4266500049,
               1856882830, 4047853341, 3908103143,  957356444, 2650641534,
               3133093783,  217165766, 1698675267, 1978033093, 2957958070,
               1197341066, 2218650193, 4156460605, 2679291258, 2672504636,
                302575060, 3733608617,  468510068,  856277790, 2378978816,
               1758832982, 3348633072, 1207830473, 3806762842, 3009791302,
               1486874279, 1966930667, 1815073667,  184138618,  780013942,
               1378997062, 3939164118, 2009917167, 1131443368, 2506068040,
               2151616051, 2795195928, 2374960423, 3152383698,  558192057,
               3723803863, 3111966212, 3432341570, 1841456174, 2111794524,
                 41430576, 3441875793, 2450178390, 2246096067, 2250937394,
               1339624910, 3590222514, 4148729803, 1907563611, 1333796279,
               1943197934,  151423013, 1050001709,  954779967, 3464016724,
               2917541221,  685195871, 1895887479, 3479111107,   84998451,
               2867069372, 1795734559, 1206410852, 3902102294, 3729169518,
               4110672436, 1835730199, 3883191127, 2591017623, 2680471827,
                549373906, 1353083685, 2369272560, 2517786213,  299567367,
               3163021476, 3273801914, 3046306181, 3160295570, 1005539436,
               3810518135,  838969910, 1443887283, 2143384546,  131392748,
               4288699511, 3792988856, 4195912091, 3983128929, 3066233780,
               1353100986,  483425267, 1637948793, 4048131166, 3863765312,
               3774619263,  968770582, 2784415937, 3280576117, 1150104494,
               3299561683, 1945187110, 1821048422, 3612225997, 2690457877,
               3151884714, 2783758768, 1779723575, 3523135206, 2691537530,
               4064616969, 3683757330, 3907160586, 4270032052, 2164708666,
               3883102288, 1820166341,  532945427, 2224902833, 1050304537,
               2259417379, 2991031600, 3272761486,  578356314, 2704724334,
               1463574571, 1131659775, 3330248700, 1355442146, 1572407950,
               3311248417, 3163630258, 1193734878, 2518279374, 3781714180,
               2435940530, 2128802346, 3128968951, 4113126645, 3961252408,
               1357207677, 1361927482, 4250521749,  950090018, 3932329482,
               1141786681,  270034852, 3767867062,  527406935, 2183873020,
               3916269761, 1088883044, 3218806483, 1946577265, 1242406161,
               2931638174,  614458695,  534030019,  252702467, 4148295618,
               3680135666, 2769472549, 1402123630, 1987024716,  438395737,
               2871419889, 3607166188, 3964615884, 2545222141,  560327442,
               3565109539, 3182169231, 1891724505, 2486311461], dtype=np.uint32), 623, 0, 0.0))
        random.setstate((3, (2147483648, 4188814255, 2576071098, 1981290141, 3692177990, 118787254, 252216594, 1991328997, 4180144507, 921308469, 2487928124, 3239946761, 2826883553, 3839873289, 1571685991, 1157544654, 2146266649, 883818702, 3870765550, 1773991357, 712882071, 1990690469, 2341356761, 2453637358, 2690677872, 688866005, 1390968629, 2223202298, 2832779218, 1410464222, 619572741, 3924749507, 3766955351, 113826857, 237813009, 1061137210, 3321653524, 1724031458, 3743685093, 4261495268, 1063205262, 1477074425, 627161553, 1000593558, 3582925559, 3705561499, 698411529, 466623946, 3521683163, 359493193, 450824293, 2417037535, 2262738460, 2122995347, 2022564113, 1249740973, 2865974291, 676575674, 3577261001, 1314320227, 603629163, 3390390935, 3394761236, 17880993, 234781165, 3125443897, 2786942760, 793512711, 2023144472, 3093750654, 2766091661, 2609052501, 2319025403, 3341215562, 4156650268, 410975850, 3913614951, 1487712528, 3170043108, 698353385, 541385735, 2352622440, 927269727, 1491674074, 1590662959, 587311792, 3602583651, 618469278, 4237316717, 619890768, 780972007, 1339469892, 4253705863, 708257249, 1119880941, 1766871747, 4145483563, 3647872514, 986218221, 3641779338, 4122190988, 2747472367, 3872765769, 272252071, 3800455724, 3849513685, 2083649252, 3867634652, 415912774, 3653343010, 1694994901, 2106903035, 1956235453, 1590077520, 3786751847, 255868642, 4058017966, 77353200, 3846721146, 2617215499, 343114253, 2053380581, 2315427290, 3099904553, 1862533746, 1812258994, 3977312781, 3925874965, 2987310293, 665750558, 2118670957, 297568614, 1275952289, 1318288677, 3610813894, 1145188912, 2976793756, 3249029102, 1795806231, 2216153745, 1797563037, 2604793510, 1901965637, 4000239378, 3975286932, 965989154, 1068450684, 899600040, 419193960, 789937939, 664554175, 3268042934, 2202267356, 272667955, 2860350265, 406050060, 3033212343, 186154887, 492314214, 2219577519, 171431163, 3644164265, 4222876458, 11937396, 3138600839, 3674950814, 159456381, 3447633098, 3901959516, 2327677846, 3057178124, 3266150331, 2834560126, 4103363554, 1754084345, 3936941172, 1551139706, 31929438, 779002485, 2173720595, 13024116, 1288251264, 3725448487, 564242759, 730072626, 1871386764, 843424660, 449309699, 2757428793, 3576351011, 3694903352, 2911504481, 3197522691, 3930983255, 469231666, 1504272807, 3645606922, 482050083, 2190219877, 3769330227, 2265850002, 1342794074, 1701660287, 3022298880, 3194655884, 1530735830, 3498338682, 1823845431, 1706021654, 1455364310, 2906176337, 3538422202, 2359094357, 1226882686, 1028852371, 3047259335, 2519384488, 329685729, 2472807648, 1438526905, 110444989, 1258141121, 572668311, 1476904990, 550241895, 528010010, 1476879468, 151639973, 2751691031, 3364601367, 1557287237, 1174926375, 3886766956, 2289389714, 4148945537, 2013729814, 3925243780, 4256837706, 252367795, 1394520684, 4200973895, 1327034823, 4224840270, 1064182895, 417357612, 4294759728, 1173148649, 884585156, 3246900930, 2185209708, 3469940451, 4131975000, 2765245351, 1382963535, 629279121, 2499485077, 525662471, 4212183656, 2826488117, 812299056, 3170355570, 2790931967, 1034684978, 926861527, 2465944767, 3685601225, 3664321181, 3509461860, 3523422727, 4024262721, 4030971780, 1181186277, 1400360794, 3470574682, 2651095909, 3639189232, 4178543741, 526813476, 25419814, 1686265678, 2809684512, 2556558604, 2297519025, 3125767606, 3530009832, 1258507952, 199742093, 2001418034, 3350713387, 3172975568, 1652893669, 3351760806, 3944727715, 1057462459, 532785622, 3677204679, 2968917488, 3291240519, 2631922863, 2083678882, 4125305308, 3220434826, 3844324763, 1914779780, 3908258291, 530652219, 2469772030, 2419197504, 2074244008, 2411387599, 957374395, 3414803577, 2689330001, 2859309568, 4049327773, 87641443, 119613570, 1486715558, 3194801209, 2278988528, 2506755404, 605103088, 967439119, 1353482155, 1664119522, 3867953987, 1305799651, 4129597245, 1999411550, 664442675, 4066108048, 16646946, 582844583, 3524479954, 2643745086, 1381116372, 1449877125, 4057173524, 3214713685, 3776309010, 232111141, 2697159968, 1374450542, 3913067939, 2661149313, 1246489080, 1747138399, 3873286168, 2507657270, 3548865340, 295477484, 4085569296, 3718467809, 2587840470, 3808995095, 284400653, 1285103858, 401836788, 88323285, 2700632402, 1133655414, 1012581938, 2892871649, 907630432, 2077481373, 747383578, 1220420661, 970370513, 389766508, 3611629092, 1429486836, 491587138, 2711726576, 118471488, 4084261471, 880422452, 1518353653, 479425970, 4057935673, 511239714, 1196753042, 1289871667, 969664336, 1412943436, 3449445352, 4246908060, 4818493, 1678060713, 1326830131, 1216633077, 353041684, 3854307852, 536117536, 2890741875, 3180774516, 555526149, 450726843, 3032639212, 1622733539, 625416480, 3344314857, 3745894168, 3529917550, 3235240784, 2012239341, 190387833, 13901361, 1958218380, 1700199294, 771068739, 2767026758, 3788876541, 1588087406, 3223311913, 2196717638, 2207164032, 1541385122, 2058450620, 3029941601, 759797264, 3871936078, 2600491443, 1360719640, 125800245, 1496960498, 2796021377, 1214510775, 1932357130, 2629178861, 1869620318, 1868038462, 3176405916, 2060718990, 2721937507, 3454563542, 321888460, 3196827802, 4204125128, 3328964051, 3032989931, 3245103176, 1332452348, 2127065891, 1996240544, 3853639038, 3595141577, 192969701, 1853713295, 1225766962, 2211358432, 3908497684, 1416589158, 2969651956, 640228320, 4285576688, 3779451640, 1966864400, 1071185660, 1952977906, 806740047, 3923123084, 2459440570, 1169443163, 1295341140, 781495025, 3865331914, 2194721744, 1070803196, 2101266853, 917268457, 769046710, 2999821758, 2510538216, 781250432, 2537725459, 611147711, 283472945, 962439282, 3026944203, 3559936189, 2552150899, 1640213143, 967688761, 3407361320, 1226522196, 2754076341, 2268504635, 2432186412, 3810385871, 749223410, 3310273364, 1352411574, 3481515215, 4107952353, 1179234909, 3180088989, 81696415, 994976802, 3824345698, 2593674263, 1870683755, 2249834898, 2255132131, 1908779533, 1184727317, 622342247, 2050704676, 3235939911, 2572604494, 463202459, 1010865026, 2385402289, 450439336, 2850614914, 3789900577, 3877892656, 920257652, 478106720, 396765749, 292963716, 2183224628, 868302321, 1880244108, 1154644024, 1973364660, 1157327777, 2599834019, 555388974, 2278271416, 4131090720, 541721643, 3102735497, 1023200412, 3840847815, 1882853767, 882655881, 3047742237, 3641792086, 3852214430, 1955173704, 1095963730, 1012956565, 11958573, 1531668997, 318021866, 2776796029, 4246952229, 3529632127, 1903484245, 2932699126, 2573307198, 487963338, 864758141, 4274654382, 2040766952, 2275551788, 1297152179, 2509074312, 1926997965, 1561268126, 3131177326, 3524272043, 378878966, 1326542574, 902797927, 698049998, 3373679536, 1579857663, 2602386727, 1980545243, 3277452601, 1692094428, 2608088788, 3227021435, 2694485211, 3451696017, 1109372860, 2517232455, 1196563764, 496912315, 905316680, 804154549, 3383198500, 1797227462, 1735884864, 2808729790, 3669646565, 1665472636, 48491904, 1788473129, 1891720278, 739251441, 17192513, 1387789873, 1677569848, 1914058513, 1776236428, 590211649, 1120395288, 3780222067, 1476926250, 3107240353, 458415803, 2882976773, 101503461, 1863174431, 2843617113, 3164116617, 3488694416, 1357545665, 153532621, 201302980, 2775761343, 806228104, 1844690630, 791412730, 724935152, 2901367368, 172966247, 1121373979, 1919611833, 3549597064, 3310013585, 553211189, 624), None))

        print(np.random.get_state())
        print(random.getstate())

        vehicles = {}
        # net = sumolib.net.readNet(self.hyper_params['scenario_path'])
        buildings = sumolib.shapes.polygon.read(self.hyper_params['scenario_polys'])
        tmp = []

        for building in buildings:
            if building.type != "unknown":
                tmp.append(building)

        buildings = tmp

        while step < 100000:
            step += 1
            print("Step:", step)
            # if traci.inductionloop.getLastStepVehicleNumber("1") > 0:
            #     traci.trafficlight.setRedYellowGreenState("0", "GrGr")

            traci.simulationStep()
            traci.route.getIDList()

            # 1) Get All Vehicles with Wireless
            vehicle_ids = traci.vehicle.getIDList()
            # print(vehicle_list)

            for vid in vehicle_ids:
                v_found = False

                for vehicle_id, vehicle in vehicles.items():
                    if vehicle_id == vid:
                        v_road_id = traci.vehicle.getRoadID(vid)
                        vehicle.update_latest_edge_road(v_road_id)
                        v_found = True

                if not v_found:
                    vehicles[vid] = Vehicle(vehicle_id=vid, hyper_params=self.hyper_params)

            need_to_remove = set(vehicles.keys()) - set(vehicle_ids)

            for vid in need_to_remove:
                vehicles.pop(vid)

            if len(vehicle_ids) == 0:
                break
            else:
                snapshot = False
                if len(vehicle_ids) >= self.hyper_params['tot_num_vehicles']:
                    # if np.random.random() > 0.5:
                    snapshot = True

            print("=============================================================")

            if not snapshot:
                continue

            print(vehicles.keys())
            viz.draw_vehicles(vehicles.values())

            cv2x_len = int(self.hyper_params["cv2x_N"] * len(vehicle_ids))
            cv2x_vehicles_indexes = random.sample(range(len(vehicle_ids)), cv2x_len)
            non_cv2x_vehicles_indexes = list(set(range(len(vehicle_ids))) - set(cv2x_vehicles_indexes))
            # Transform indexes into IDs and vehicles
            cv2x_vehicles = {vehicle_ids[index]:vehicles[vehicle_ids[index]] for index in cv2x_vehicles_indexes}
            non_cv2x_vehicles = {vehicle_ids[index]:vehicles[vehicle_ids[index]] for index in non_cv2x_vehicles_indexes}

            show_id = None

            for cv2x_id, vehicle in cv2x_vehicles.items():
                if cv2x_id != show_id and cv2x_id is not None:
                    continue

                viz.draw_vehicle_perception(vehicle, (185, 218, 255))

            # 2) Get seen non-cv2x vehicles by each cv2x_vehicle
            cv2x_perceived_non_cv2x_vehicles = self.get_seen_vehicles(list(cv2x_vehicles.values()), list(non_cv2x_vehicles.values()), buildings)
            tot_perceived_objects = 0

            for cv2x_id, non_cv2x_ids in cv2x_perceived_non_cv2x_vehicles.items():
                if cv2x_id != show_id and cv2x_id is not None:
                    continue

                cv2x_non_vehs = [vehicles[id] for id in non_cv2x_ids]

                for vehicle in cv2x_non_vehs:
                    viz.draw_vehicle_body(vehicle, color=(0, 0, 128))

                tot_perceived_objects += len(non_cv2x_ids)

            viz.save_img(os.path.join(os.path.dirname(self.hyper_params['scenario_path']), "map.png"))

            # 3) Solve which info to send to base station
            # 3.1) Calculate required information
            scores_per_cv2x = self.calculate_scores_per_cv2x(cv2x_perceived_non_cv2x_vehicles, cv2x_vehicles, non_cv2x_vehicles,
                                                        buildings)

            # 3.2) Get Highest Score for each cv2x vehicle
            score_per_cv2x = {cv2x: max(scores, key=lambda x: x[1]) for cv2x, scores in scores_per_cv2x.items()}

            # 3.3) Prevent Vehicles from sending with score = 0
            score_per_cv2x = {cv2x:score_receiver for cv2x, score_receiver in score_per_cv2x.items() if score_receiver[1] != 0}

            # 3.4) Make unique
            score_per_cv2x = self.make_unique_requests(score_per_cv2x)

            h_n_k = self.get_channel_gain_vehicle(self.hyper_params["base_station_position"],
                                             [vehicle for vehicle in vehicles.values() if
                                                   vehicle.vehicle_id in score_per_cv2x],
                                             self.hyper_params['num_RBs'])

            # 3.3) Baseline to solve the problem and select which requests to send
            solver = Solver(score_per_cv2x, h_n_k, self.hyper_params['num_RBs'], self.hyper_params['message_size'])
            sent, unsent = solver.find_optimal_assignment()

            with open(os.path.join(os.path.dirname(self.hyper_params['scenario_path']),
                                   "results_"+str(self.hyper_params['num_RBs'])+"_"+str(self.hyper_params['tot_num_vehicles'])+".txt"), 'w') as fw:
                fw.write("Total Requests: " + str(sent+unsent)
                         + "\nSent: " + str(sent)
                         + "\nUnsent: " + str(unsent)
                         + "\nPerceived Vehicles: " + str(tot_perceived_objects))

            if snapshot:
                break

        viz.save_img()
        # input("Simulation ended, close GUI?")
        print("Simulation terminated!")#, scores_per_cv2x.items())
        traci.close()


if __name__ == '__main__':
    hyper_params = {}
    hyper_params['scenario_path'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_test/test.net.xml"
    hyper_params['scenario_map'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_test/net.sumo.cfg"
    hyper_params['scenario_polys'] = "/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto_test/map.poly.xml"
    hyper_params["cv2x_N"] = 0.25
    hyper_params["fov"] = 120
    hyper_params["view_range"] = 75
    hyper_params["base_station_position"] = 1600, 600
    hyper_params["num_RBs"] = 10
    hyper_params['message_size'] = 2000*4
    hyper_params['tot_num_vehicles'] = 100
    sim = Simulation(hyper_params)
    sim.run()
