import multiprocessing

class Worker(multiprocessing.Process):
    def __init__(self, a):
        multiprocessing.Process.__init__(self)
        self.name2 = a

    def run(self):
        print ('In %s' % self.name2)
        return

if __name__ == '__main__':
    jobs = []
    for i in range(5):
        p = Worker(i)
        jobs.append(p)
        p.start()
    for j in jobs:
        j.join()
