from mappers import mapper_selector


class Cartridge(object):

    MIRRORING_VERTICAL = 1
    MIRRORING_HORIZONTAL = 0

    def __init__(self, file_path):
        with open(file_path, 'rb') as file_data:
            self.header = file_data.read(16)

            self.prg_banks = self.header[4]
            self.chr_banks = self.header[5]

            self.flags6 = self.header[6]

            self.mirroring = self.flags6 & (1 << 0)
            self.has_battery = self.flags6 & (1 << 1)
            self.has_trainer = self.flags6 & (1 << 2)
            self.four_screen = self.flags6 & (1 << 4)
            mapper_id_lower_byte = self.flags6 << 4

            self.flags7 = self.header[7]
            mapper_id_upper_byte = self.flags7 >> 4

            self.mapper_id = mapper_id_upper_byte | mapper_id_lower_byte

            if self.has_trainer:
                self.trainer = file_data.read(512)

            self.prg_data = file_data.read(self.prg_banks * 16384)

            self.chr_data = file_data.read(self.chr_banks * 8192)

            mapper_class = mapper_selector.select(self.mapper_id)

            self.mapper = mapper_class(self.prg_banks, self.chr_banks)

    def cpu_read(self, address):
        new_address, should_map = self.mapper.cpu_map_read(address)

        data = 0x00

        if should_map:
            data = self.prg_data[new_address]
            return True, data
        else:
            return False, data

    def cpu_write(self, address, data):
        pass

    def ppu_read(self, address, data):
        pass

    def ppu_write(self, address, data):
        pass
