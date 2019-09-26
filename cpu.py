class CPU(object):

    def __init__(self):
        self.address_absolute = 0x0000
        self.address_relative = 0x0000
        self.fetched = 0x00

        self.cycles = 0

        # Registers
        self.accumulator = 0
        self.x = 0
        self.y = 0
        self.status = 0
        self.program_counter = 0
        self.stack_pointer = 0

    def set_bus(self, bus):
        self.bus = bus

    def reset(self):
        address_absolute = 0xFFFC
        low_byte = self.read(address_absolute + 0)
        high_byte = self.read(address_absolute + 1)

        self.program_counter = (high_byte << 8) | low_byte

        self.accumulator = 0
        self.x = 0
        self.y = 0
        self.stack_pointer = 0xFD
        self.status = 0x00 | U

        self.address_relative = 0x0000
        self.address_absolute = 0x0000
        self.fetched = 0x00

        self.cycles = 8

    def read(self, address):
        return self.bus(address, False)

    def write(self):
        pass
