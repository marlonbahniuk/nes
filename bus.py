from cpu import CPU


class Bus(object):

    def __init__(self):
        self.cpu = CPU()
        self.cpu.set_bus(self)

        # self.cpu.reset()

    def read(self):
        pass

    def write(self):
        pass
