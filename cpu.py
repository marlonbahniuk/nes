class CPU(object):
    # Flags
    C = 1 << 0  # Carry Bit
    Z = 1 << 1  # Zero
    I = 1 << 2  # Disable Interrupts
    D = 1 << 3  # Decimal Mode(unused in this implementation)
    B = 1 << 4  # Break
    U = 1 << 5  # Unused
    V = 1 << 6  # Overflow
    N = 1 << 7  # Negative

    def __init__(self):
        self.bus = None

        # Registers
        self.accumulator = 0
        self.x = 0
        self.y = 0
        self.status = 0
        self.program_counter = 0
        self.stack_pointer = 0

        # Helpers
        self.address_absolute = 0x0000
        self.address_relative = 0x0000
        self.fetched = 0x00

        self.remaining_cycles = 0

    def set_bus(self, bus):
        self.bus = bus

    def reset(self):
        # Set the initial program counter
        self.address_absolute = 0xFFFC
        low_byte = self.read(self.address_absolute + 0)
        high_byte = self.read(self.address_absolute + 1)
        self.program_counter = (high_byte << 8) | low_byte

        # Registers
        self.accumulator = 0
        self.x = 0
        self.y = 0
        self.stack_pointer = 0xFD
        self.status = 0x00 | self.U

        # Helpers
        self.address_relative = 0x0000
        self.address_absolute = 0x0000
        self.fetched = 0x00

        self.remaining_cycles = 8

    def get_flag(self, flag):
        return 1 if (self.status & flag) > 0 else 0

    def set_flag(self, flag, value):
        if value:
            self.status |= flag
        else:
            self.status &= flag

    def read(self, address):
        return self.bus(address, False)

    def write(self, address, data):
        self.bus.write(address, data)

    def irq(self):
        if self.get_flag(self.I) == 0:
            # Push the program counter to the stack. It's 16-bits, dont
            # forget so that takes two pushes
            self.write(0x0100 + self.stack_pointer, (self.program_counter >> 8) & 0x00FF)
            self.stack_pointer -= 1
            self.write(0x0100 + self.stack_pointer, self.program_counter & 0x00FF)
            self.stack_pointer -= 1

            # Then push the status register to the stack
            self.set_flag(self.B, 0)
            self.set_flag(self.U, 1)
            self.set_flag(self.I, 1)
            self.write(0x0100 + self.stack_pointer, self.status)
            self.stack_pointer -= 1

            # Read new program counter location from fixed address
            self.address_absolute = 0xFFFE
            low_byte = self.read(self.address_absolute + 0)
            high_byte = self.read(self.address_absolute + 1)
            self.program_counter = (high_byte << 8) | low_byte

            # IRQs take time
            self.remaining_cycles = 7

    def nmi(self):
        # Push the program counter to the stack. It's 16-bits, dont
        # forget so that takes two pushes
        self.write(0x0100 + self.stack_pointer, (self.program_counter >> 8) & 0x00FF)
        self.stack_pointer -= 1
        self.write(0x0100 + self.stack_pointer, self.program_counter & 0x00FF)
        self.stack_pointer -= 1

        # Then push the status register to the stack
        self.set_flag(self.B, 0)
        self.set_flag(self.U, 1)
        self.set_flag(self.I, 1)
        self.write(0x0100 + self.stack_pointer, self.status)
        self.stack_pointer -= 1

        # Read new program counter location from fixed address
        self.address_absolute = 0xFFFA
        low_byte = self.read(self.address_absolute + 0)
        high_byte = self.read(self.address_absolute + 1)
        self.program_counter = (high_byte << 8) | low_byte

        # NMIs take time
        self.remaining_cycles = 8
