from cpu import CPU
from ppu import PPU

class Bus(object):

    def __init__(self):
        self.cpu = CPU()
        self.cpu.set_bus(self)

        self.ppu = PPU()

        self.cpu_ram = [0] * 64 * 1024

    def cpu_read(self, address, read_only):
        ignore_next, data = self.cartridge.cpu_read(address)

        if ignore_next:
            pass
        elif 0x0000 <= address <= 0x1FFF:
            # System RAM Address Range, mirrored every 2048
            data = self.cpu_ram[address & 0x07FF]
        elif 0x2000 <= address <= 0x3FFF:
            # PPU Address range, mirrored every 8
            data = self.ppu.cpu_read(address & 0x0007, read_only)
        return data

    def cpu_write(self, address, data):
        if self.cartridge.cpu_write(address, data):
            pass

        if 0x0000 <= address <= 0xffff:
            self.cpu_ram[address] = data

    def set_cartridge(self, cartridge):
        self.cartridge = cartridge
        self.ppu.set_cartridge(cartridge)