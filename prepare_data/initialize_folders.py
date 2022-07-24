import os
import shutil
import threading


class myThread (threading.Thread):
    def __init__(self, src_files, dirnames):
        threading.Thread.__init__(self)

        self.src_files = src_files
        self.dirnames = dirnames

    def run(self):
        for dirname_ in self.dirnames:
            for sub_folder in range(100):
                dirname = dirname_ + "/" + str(sub_folder) + "/"

                for fsrc in self.src_files:
                    shutil.copy(fsrc, dirname + "/" + os.path.basename(fsrc))

                os.system(
                    "cd " + dirname + " && pwd && netconvert --osm-files map.osm -o test.net.xml -t osmNetconvert.typ.xml --xml-validation never &&"
                                      "polyconvert --net-file test.net.xml --osm-files map.osm --type-file typemap.xml -o map.poly.xml --xml-validation never &&"
                                      "python randomTrips.py --random -n test.net.xml -r map.rou.xml -o trips.trips.xml --fringe-factor 2 --min-distance 100 "
                                      "--validate -p 0.01 -b 0 -e 20 "
                                      "--trip-attributes=\"type=\\\"typedist1\\\"\" --additional-file typedistrib1.xml")


# path = '/media/bassel/Entertainment/sumo_traffic/sumo_map/toronto'
# path = '/media/bassel/Career/toronto_content_selection/toronto'
path = '/home/bassel/toronto_AVpercentage_RBs'
# path = '/media/bassel/Career/toronto_content_selection/toronto_more_busses'
maps = './data'

if os.path.exists(path):
    shutil.rmtree(path, True)

os.makedirs(path)

file_names = os.listdir(maps)
map_file_names = [name for name in file_names if name.find(".osm") != -1]
base_pos_file_names = [name for name in file_names if name.find(".txt") != -1]
map_file_names.sort()

base_name='toronto_'

for i in range(len(map_file_names)):
    os.makedirs(path+"/"+base_name+str(i))

    for sub_folder in range(100):
        dirname = path+"/"+base_name+str(i) + "/" + str(sub_folder)+ "/"
        os.makedirs(dirname)

for i, (map_name, base_pos_name) in enumerate(zip(map_file_names, base_pos_file_names)):
    for sub_folder in range(100):
        dirname = path + "/" + base_name + str(i) + "/" + str(sub_folder) + "/"

        shutil.copy(maps + "/" + map_name, dirname + "/map.osm")
        shutil.copy(maps + "/" + base_pos_name, dirname + "/basestation_pos.txt")

src_files = ['/media/bassel/Entertainment/sumo_traffic/sumo_map/default/net.sumo.cfg',
            '/media/bassel/Entertainment/sumo_traffic/sumo_map/default/net2geojson.py',
            '/media/bassel/Entertainment/sumo_traffic/sumo_map/default/osmNetconvert.typ.xml',
            '/media/bassel/Entertainment/sumo_traffic/sumo_map/default/randomTrips.py',
            '/media/bassel/Entertainment/sumo_traffic/sumo_map/default/typemap.xml',
            '/media/bassel/Entertainment/sumo_traffic/sumo_map/default/typedistrib1.xml']

n_threads = 12
n = n_threads if n_threads<len(map_file_names) else len(map_file_names)
block_Size = len(map_file_names)//n
list_threads = []

for i in range(n):
    start = i * block_Size
    end = start + block_Size

    if i == n-1:
        end = len(map_file_names)

    print("thread " + str(i) + " launched")

    dirnames = [path + "/" + base_name + str(i) for i in range(start, end)]

    initialize_maps_thread = myThread(src_files, dirnames)
    initialize_maps_thread.start()
    list_threads.append(initialize_maps_thread)

for i in range(n):
    list_threads[i].join()

print("File Preparation Done! Happy Simulations : )")

