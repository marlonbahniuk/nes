from .mapper import Mapper


class Mapper0(Mapper):

    def cpuMapRead(self, address):
        if 0x8000 <= address <= 0xffff:
            return address & (0x7fff if self.prg_banks > 1 else 0x3fff), True
        return address, False

    def cpuMapWrite(self, address):
        if 0x8000 <= address <= 0xffff:
            return address & (0x7fff if self.prg_banks > 1 else 0x3fff), True
        return address, False

    def ppuMapRead(self, address):
        if 0x0000 <= address <= 0x1ffff:
            return address, True
        return address, False

    def ppuMapWrite(self, address):
        if 0x0000 <= address <= 0x1fff:
            if self.chr_banks == 0:
                return address, True
            return address, False