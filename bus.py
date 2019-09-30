from cpu import CPU


class Bus(object):

    def __init__(self):
        self.cpu = CPU()
        self.cpu.set_bus(self)

        self.ram = [0] * 64 * 1024

    def cpu_read(self, address, read_only):
        if 0x0000 <= address <= 0xffff:
            return self.ram[address]

        return 0x00

    def cpu_write(self, address, data):
        if self.cartridge.cpuWrite(address, data)
        if 0x0000 <= address <= 0xffff:
            self.ram[address] = data

    def set_cartridge(self, cartridge):
        self.cartridge = cartridge