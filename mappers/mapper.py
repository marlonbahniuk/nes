class Mapper(object):

    def __init__(self, prg_banks, chr_banks):
        self.prg_banks = prg_banks
        self.chr_banks = chr_banks

    def cpu_map_read(self, address):
        pass

    def cpu_map_write(self, address):
        pass

    def ppu_map_read(self, address):
        pass

    def ppu_map_write(self, address):
        pass
