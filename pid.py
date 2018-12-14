import Queue

class PID_controller():
    def __init__(self, KPID, QUEUE_SIZE = 2000):
        self.PID = list([0., 0., 0.])
        self.KPID = list([1.*KPID[0], 1.*KPID[1]/QUEUE_SIZE, 1.*KPID[2]])
        self.integral_ghb = Queue.Queue(QUEUE_SIZE)
        self.last_tmp = 0
        self.ctrl = 0
        self.queue_size = QUEUE_SIZE

    def step(self, cur_data):
        self.PID[0] = cur_data
        self.PID[2] = cur_data - self.last_tmp
        self.last_tmp = cur_data
        if not self.integral_ghb.full():
            self.integral_ghb.put(cur_data)
            self.PID[1] += cur_data
        else:
            self.PID[1] = self.PID[1] - self.integral_ghb.get() + cur_data
            self.integral_ghb.put(cur_data)
        self.ctrl =  sum(self.PID[i] * self.KPID[i] for i in range(3))/sum(self.KPID)

    def get_KPID(self):
        return [i for i in KPID]

    def get_PID(self):
        return [i for i in PID]

    def set_KPID(self,new_kpid):
        self.KPID[0] = new_kpid[0]
        self.KPID[1] = new_kpid[1] / float(QUEUE_SIZE)
        self.KPID[2] = new_kpid[2]

    def get_ctrl(self):
        return self.ctrl

    def dump(self):
        print("PID : " + str(self.PID))
        print("CONTROL : " + str(self.ctrl))
