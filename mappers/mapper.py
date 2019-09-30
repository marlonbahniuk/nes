class Mapper(object):

    def __init__(self, prg_banks, chr_banks):
        self.prg_banks = prg_banks
        self.chr_banks = chr_banks

    def cpuMapRead(self, address):
        pass

    def cpuMapWrite(self, address):
        pass

    def ppuMapRead(self, address):
        pass

    def ppuMapWrite(self, address):
        pass