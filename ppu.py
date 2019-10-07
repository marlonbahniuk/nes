import numpy as np

import palette
from cartridge import Cartridge


class PPU(object):

    def __init__(self):
        self.screen = np.full((256, 240), (20, 255, 100, 100), (int, 4))
        self.pattern_tables = np.full((2, 128, 128), (0, 0, 0, 100), (int, 4))
        self.name_tables = np.full((2, 1024), 0)
        self.pattern = np.full((2, 4096), 0)
        self.palette_table = palette.palette
        self.palette = np.full(32, 0)

    def set_cartridge(self, cartridge):
        self.cartridge = cartridge

    def ppu_read(self, address, read_only=False):
        data = 0x00
        address &= 0x3FFF

        if self.cartridge.ppu_read(address, data):
            pass
        elif 0x0000 <= address <= 0x1FFF:
            data = self.pattern[(address & 0x1000) >> 12][address & 0x0FFF]
        elif 0x2000 <= address <= 0x3EFF:
            address &= 0x0FFF

            if self.cartridge.mirroring == Cartridge.MIRRORING_VERTICAL:
                # Vertical
                if 0x0000 <= address <= 0x03FF:
                    data = self.name_tables[0][address & 0x03FF]
                if 0x0400 <= address <= 0x07FF:
                    data = self.name_tables[1][address & 0x03FF]
                if 0x0800 <= address <= 0x0BFF:
                    data = self.name_tables[0][address & 0x03FF]
                if 0x0C00 <= address <= 0x0FFF:
                    data = self.name_tables[1][address & 0x03FF]
            elif self.cartridge.mirroring == Cartridge.MIRRORING_HORIZONTAL:
                # Horizontal
                if 0x0000 <= address <= 0x03FF:
                    data = self.name_tables[0][address & 0x03FF]
                if 0x0400 <= address <= 0x07FF:
                    data = self.name_tables[0][address & 0x03FF]
                if 0x0800 <= address <= 0x0BFF:
                    data = self.name_tables[1][address & 0x03FF]
                if 0x0C00 <= address <= 0x0FFF:
                    data = self.name_tables[1][address & 0x03FF]
        elif 0x3F00 <= address <= 0x3FFF:

            address &= 0x001F

            if address == 0x0010:
                address = 0x0000
            if address == 0x0014:
                address = 0x0004
            if address == 0x0018:
                address = 0x0008
            if address == 0x001C:
                address = 0x000C

            data = self.palette[address] & (0x30 if False else 0x3F)

        return data

    def cpu_read(self, address, read_only):
        data = 0x00

        return data

    def get_screen(self):
        return self.screen

    def get_color_from_palette_ram(self, palette, pixel):
        return self.palette[self.ppu_read(0x3F00 + (palette << 2) + pixel) & 0x3F]

    def get_pattern_table(self, pattern_table, pallet):
        for x in range(16):
            for y in range(16):
                offset = x * 16 + y * 256

                for row in range(8):
                    low_byte = self.ppu_read(pattern_table + 0x1000 + offset + row + 0x0000)
                    high_byte = self.ppu_read(pattern_table + 0x1000 + offset + row + 0x0008)

                    for col in range(8):
                        pixel = (low_byte & 0x01) + (high_byte & 0x01)

                        low_byte >>= 1
                        high_byte >>= 1

                        self.pattern_tables[pattern_table][x * 8 + (7 - col)][
                            y * 8 + row] = self.get_color_from_palette_ram(pallet, pixel)

        return self.pattern_tables[pattern_table]

    def clock(self):
        pass
