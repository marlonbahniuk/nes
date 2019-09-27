import numpy as np

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

    # Instructions lookup table
    lookup = (
        ('BRK', 'brk', 'imm', 7), ('ORA', 'ora', 'izx', 6), ('???', 'xxx', 'imp', 2), ('???', 'xxx', 'imp', 8),
        ('???', 'nop', 'imp', 3), ('ORA', 'ora', 'zp0', 3), ('ASL', 'asl', 'zp0', 5), ('???', 'xxx', 'imp', 5),
        ('PHP', 'php', 'imp', 3), ('ORA', 'ora', 'imm', 2), ('ASL', 'asl', 'imp', 2), ('???', 'xxx', 'imp', 2),
        ('???', 'nop', 'imp', 4), ('ORA', 'ora', 'abs', 4), ('ASL', 'asl', 'abs', 6), ('???', 'xxx', 'imp', 6),
        ('BPL', 'bpl', 'rel', 2), ('ORA', 'ora', 'izy', 5), ('???', 'xxx', 'imp', 2), ('???', 'xxx', 'imp', 8),
        ('???', 'nop', 'imp', 4), ('ORA', 'ora', 'zpx', 4), ('ASL', 'asl', 'zpx', 6), ('???', 'xxx', 'imp', 6),
        ('CLC', 'clc', 'imp', 2), ('ORA', 'ora', 'aby', 4), ('???', 'nop', 'imp', 2), ('???', 'xxx', 'imp', 7),
        ('???', 'nop', 'imp', 4), ('ORA', 'ora', 'abx', 4), ('ASL', 'asl', 'abx', 7), ('???', 'xxx', 'imp', 7),
        ('JSR', 'jsr', 'abs', 6), ('AND', 'and', 'izx', 6), ('???', 'xxx', 'imp', 2), ('???', 'xxx', 'imp', 8),
        ('BIT', 'bit', 'zp0', 3), ('AND', 'and', 'zp0', 3), ('ROL', 'rol', 'zp0', 5), ('???', 'xxx', 'imp', 5),
        ('PLP', 'plp', 'imp', 4), ('AND', 'and', 'imm', 2), ('ROL', 'rol', 'imp', 2), ('???', 'xxx', 'imp', 2),
        ('BIT', 'bit', 'abs', 4), ('AND', 'and', 'abs', 4), ('ROL', 'rol', 'abs', 6), ('???', 'xxx', 'imp', 6),
        ('BMI', 'bmi', 'rel', 2), ('AND', 'and', 'izy', 5), ('???', 'xxx', 'imp', 2), ('???', 'xxx', 'imp', 8),
        ('???', 'nop', 'imp', 4), ('AND', 'and', 'zpx', 4), ('ROL', 'rol', 'zpx', 6), ('???', 'xxx', 'imp', 6),
        ('SEC', 'sec', 'imp', 2), ('AND', 'and', 'aby', 4), ('???', 'nop', 'imp', 2), ('???', 'xxx', 'imp', 7),
        ('???', 'nop', 'imp', 4), ('AND', 'and', 'abx', 4), ('ROL', 'rol', 'abx', 7), ('???', 'xxx', 'imp', 7),
        ('RTI', 'rti', 'imp', 6), ('EOR', 'eor', 'izx', 6), ('???', 'xxx', 'imp', 2), ('???', 'xxx', 'imp', 8),
        ('???', 'nop', 'imp', 3), ('EOR', 'eor', 'zp0', 3), ('LSR', 'lsr', 'zp0', 5), ('???', 'xxx', 'imp', 5),
        ('PHA', 'pha', 'imp', 3), ('EOR', 'eor', 'imm', 2), ('LSR', 'lsr', 'imp', 2), ('???', 'xxx', 'imp', 2),
        ('JMP', 'jmp', 'abs', 3), ('EOR', 'eor', 'abs', 4), ('LSR', 'lsr', 'abs', 6), ('???', 'xxx', 'imp', 6),
        ('BVC', 'bvc', 'rel', 2), ('EOR', 'eor', 'izy', 5), ('???', 'xxx', 'imp', 2), ('???', 'xxx', 'imp', 8),
        ('???', 'nop', 'imp', 4), ('EOR', 'eor', 'zpx', 4), ('LSR', 'lsr', 'zpx', 6), ('???', 'xxx', 'imp', 6),
        ('CLI', 'cli', 'imp', 2), ('EOR', 'eor', 'aby', 4), ('???', 'nop', 'imp', 2), ('???', 'xxx', 'imp', 7),
        ('???', 'nop', 'imp', 4), ('EOR', 'eor', 'abx', 4), ('LSR', 'lsr', 'abx', 7), ('???', 'xxx', 'imp', 7),
        ('RTS', 'rts', 'imp', 6), ('ADC', 'adc', 'izx', 6), ('???', 'xxx', 'imp', 2), ('???', 'xxx', 'imp', 8),
        ('???', 'nop', 'imp', 3), ('ADC', 'adc', 'zp0', 3), ('ROR', 'ror', 'zp0', 5), ('???', 'xxx', 'imp', 5),
        ('PLA', 'pla', 'imp', 4), ('ADC', 'adc', 'imm', 2), ('ROR', 'ror', 'imp', 2), ('???', 'xxx', 'imp', 2),
        ('JMP', 'jmp', 'ind', 5), ('ADC', 'adc', 'abs', 4), ('ROR', 'ror', 'abs', 6), ('???', 'xxx', 'imp', 6),
        ('BVS', 'bvs', 'rel', 2), ('ADC', 'adc', 'izy', 5), ('???', 'xxx', 'imp', 2), ('???', 'xxx', 'imp', 8),
        ('???', 'nop', 'imp', 4), ('ADC', 'adc', 'zpx', 4), ('ROR', 'ror', 'zpx', 6), ('???', 'xxx', 'imp', 6),
        ('SEI', 'sei', 'imp', 2), ('ADC', 'adc', 'aby', 4), ('???', 'nop', 'imp', 2), ('???', 'xxx', 'imp', 7),
        ('???', 'nop', 'imp', 4), ('ADC', 'adc', 'abx', 4), ('ROR', 'ror', 'abx', 7), ('???', 'xxx', 'imp', 7),
        ('???', 'nop', 'imp', 2), ('STA', 'sta', 'izx', 6), ('???', 'nop', 'imp', 2), ('???', 'xxx', 'imp', 6),
        ('STY', 'sty', 'zp0', 3), ('STA', 'sta', 'zp0', 3), ('STX', 'stx', 'zp0', 3), ('???', 'xxx', 'imp', 3),
        ('DEY', 'dey', 'imp', 2), ('???', 'nop', 'imp', 2), ('TXA', 'txa', 'imp', 2), ('???', 'xxx', 'imp', 2),
        ('STY', 'sty', 'abs', 4), ('STA', 'sta', 'abs', 4), ('STX', 'stx', 'abs', 4), ('???', 'xxx', 'imp', 4),
        ('BCC', 'bcc', 'rel', 2), ('STA', 'sta', 'izy', 6), ('???', 'xxx', 'imp', 2), ('???', 'xxx', 'imp', 6),
        ('STY', 'sty', 'zpx', 4), ('STA', 'sta', 'zpx', 4), ('STX', 'stx', 'zpy', 4), ('???', 'xxx', 'imp', 4),
        ('TYA', 'tya', 'imp', 2), ('STA', 'sta', 'aby', 5), ('TXS', 'txs', 'imp', 2), ('???', 'xxx', 'imp', 5),
        ('???', 'nop', 'imp', 5), ('STA', 'sta', 'abx', 5), ('???', 'xxx', 'imp', 5), ('???', 'xxx', 'imp', 5),
        ('LDY', 'ldy', 'imm', 2), ('LDA', 'lda', 'izx', 6), ('LDX', 'ldx', 'imm', 2), ('???', 'xxx', 'imp', 6),
        ('LDY', 'ldy', 'zp0', 3), ('LDA', 'lda', 'zp0', 3), ('LDX', 'ldx', 'zp0', 3), ('???', 'xxx', 'imp', 3),
        ('TAY', 'tay', 'imp', 2), ('LDA', 'lda', 'imm', 2), ('TAX', 'tax', 'imp', 2), ('???', 'xxx', 'imp', 2),
        ('LDY', 'ldy', 'abs', 4), ('LDA', 'lda', 'abs', 4), ('LDX', 'ldx', 'abs', 4), ('???', 'xxx', 'imp', 4),
        ('BCS', 'bcs', 'rel', 2), ('LDA', 'lda', 'izy', 5), ('???', 'xxx', 'imp', 2), ('???', 'xxx', 'imp', 5),
        ('LDY', 'ldy', 'zpx', 4), ('LDA', 'lda', 'zpx', 4), ('LDX', 'ldx', 'zpy', 4), ('???', 'xxx', 'imp', 4),
        ('CLV', 'clv', 'imp', 2), ('LDA', 'lda', 'aby', 4), ('TSX', 'tsx', 'imp', 2), ('???', 'xxx', 'imp', 4),
        ('LDY', 'ldy', 'abx', 4), ('LDA', 'lda', 'abx', 4), ('LDX', 'ldx', 'aby', 4), ('???', 'xxx', 'imp', 4),
        ('CPY', 'cpy', 'imm', 2), ('CMP', 'cmp', 'izx', 6), ('???', 'nop', 'imp', 2), ('???', 'xxx', 'imp', 8),
        ('CPY', 'cpy', 'zp0', 3), ('CMP', 'cmp', 'zp0', 3), ('DEC', 'dec', 'zp0', 5), ('???', 'xxx', 'imp', 5),
        ('INY', 'iny', 'imp', 2), ('CMP', 'cmp', 'imm', 2), ('DEX', 'dex', 'imp', 2), ('???', 'xxx', 'imp', 2),
        ('CPY', 'cpy', 'abs', 4), ('CMP', 'cmp', 'abs', 4), ('DEC', 'dec', 'abs', 6), ('???', 'xxx', 'imp', 6),
        ('BNE', 'bne', 'rel', 2), ('CMP', 'cmp', 'izy', 5), ('???', 'xxx', 'imp', 2), ('???', 'xxx', 'imp', 8),
        ('???', 'nop', 'imp', 4), ('CMP', 'cmp', 'zpx', 4), ('DEC', 'dec', 'zpx', 6), ('???', 'xxx', 'imp', 6),
        ('CLD', 'cld', 'imp', 2), ('CMP', 'cmp', 'aby', 4), ('NOP', 'nop', 'imp', 2), ('???', 'xxx', 'imp', 7),
        ('???', 'nop', 'imp', 4), ('CMP', 'cmp', 'abx', 4), ('DEC', 'dec', 'abx', 7), ('???', 'xxx', 'imp', 7),
        ('CPX', 'cpx', 'imm', 2), ('SBC', 'sbc', 'izx', 6), ('???', 'nop', 'imp', 2), ('???', 'xxx', 'imp', 8),
        ('CPX', 'cpx', 'zp0', 3), ('SBC', 'sbc', 'zp0', 3), ('INC', 'inc', 'zp0', 5), ('???', 'xxx', 'imp', 5),
        ('INX', 'inx', 'imp', 2), ('SBC', 'sbc', 'imm', 2), ('NOP', 'nop', 'imp', 2), ('???', 'sbc', 'imp', 2),
        ('CPX', 'cpx', 'abs', 4), ('SBC', 'sbc', 'abs', 4), ('INC', 'inc', 'abs', 6), ('???', 'xxx', 'imp', 6),
        ('BEQ', 'beq', 'rel', 2), ('SBC', 'sbc', 'izy', 5), ('???', 'xxx', 'imp', 2), ('???', 'xxx', 'imp', 8),
        ('???', 'nop', 'imp', 4), ('SBC', 'sbc', 'zpx', 4), ('INC', 'inc', 'zpx', 6), ('???', 'xxx', 'imp', 6),
        ('SED', 'sed', 'imp', 2), ('SBC', 'sbc', 'aby', 4), ('NOP', 'nop', 'imp', 2), ('???', 'xxx', 'imp', 7),
        ('???', 'nop', 'imp', 4), ('SBC', 'sbc', 'abx', 4), ('INC', 'inc', 'abx', 7), ('???', 'xxx', 'imp', 7),
    )

    def __init__(self):
        self.bus = None

        # Registers
        self.accumulator = 0x00
        self.x = 0x00
        self.y = 0x00
        self.status = 0x00
        self.program_counter = 0x0000
        self.stack_pointer = 0x00

        # Helpers
        self.address_absolute = 0x0000
        self.address_relative = 0x0000
        self.fetched = 0x00

        self.remaining_cycles = 0

        self.clock_count = 0

    def set_bus(self, bus):
        self.bus = bus

    def reset(self):
        # Set the initial program counter
        self.address_absolute = 0xfffc
        _, low_byte = divmod(self.read(self.address_absolute + 0), 0x100)
        high_byte, _ = divmod(self.read(self.address_absolute + 1), 0x100)
        self.program_counter = (high_byte << 8) | low_byte

        # Registers
        self.accumulator = 0
        self.x = 0
        self.y = 0
        self.stack_pointer = 0xfd
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
        return self.bus.read(address, False)

    def write(self, address, data):
        self.bus.write(address, data)

    def clock(self):
        if self.remaining_cycles == 0:
            self.opcode = self.read(self.program_counter)

            self.set_flag(self.U, True)

            self.program_counter += 1

            instruction_data = self.lookup[self.opcode]

            cycles = instruction_data[3]
            addressing_mode = getattr(self, instruction_data[2], None)
            instruction = getattr(self, instruction_data[1], None)

            if addressing_mode and instruction:
                additional_cycle_1 = addressing_mode()
                additional_cycle_2 = instruction()

                cycles += additional_cycle_1 and additional_cycle_2

            self.remaining_cycles = cycles

            self.set_flag(self.U, True)

        self.clock_count += 1
        self.remaining_cycles -= 1

    def fetch(self):

        if not self.lookup[self.opcode][2] == 'imp':
            self.fetched = self.read(self.address_absolute)
        return self.fetched

    def irq(self):
        if self.get_flag(self.I) == 0:
            # Push the program counter to the stack. It's 16-bits, dont
            # forget so that takes two pushes
            self.write(0x0100 + self.stack_pointer, (self.program_counter >> 8) & 0x00ff)
            self.stack_pointer -= 1
            self.write(0x0100 + self.stack_pointer, self.program_counter & 0x00ff)
            self.stack_pointer -= 1

            # Then push the status register to the stack
            self.set_flag(self.B, 0)
            self.set_flag(self.U, 1)
            self.set_flag(self.I, 1)
            self.write(0x0100 + self.stack_pointer, self.status)
            self.stack_pointer -= 1

            # Read new program counter location from fixed address
            self.address_absolute = 0xfffe
            low_byte = self.read(self.address_absolute + 0)
            high_byte = self.read(self.address_absolute + 1)
            self.program_counter = (high_byte << 8) | low_byte

            # IRQs take time
            self.remaining_cycles = 7

    def nmi(self):
        # Push the program counter to the stack. It's 16-bits, dont
        # forget so that takes two pushes
        self.write(0x0100 + self.stack_pointer, (self.program_counter >> 8) & 0x00ff)
        self.stack_pointer -= 1
        self.write(0x0100 + self.stack_pointer, self.program_counter & 0x00ff)
        self.stack_pointer -= 1

        # Then push the status register to the stack
        self.set_flag(self.B, 0)
        self.set_flag(self.U, 1)
        self.set_flag(self.I, 1)
        self.write(0x0100 + self.stack_pointer, self.status)
        self.stack_pointer -= 1

        # Read new program counter location from fixed address
        self.address_absolute = 0xfffa
        low_byte = self.read(self.address_absolute + 0)
        high_byte = self.read(self.address_absolute + 1)
        self.program_counter = (high_byte << 8) | low_byte

        # NMIs take time
        self.remaining_cycles = 8

    # Addressing modes
    def imm(self):
        self.address_absolute = self.program_counter
        self.program_counter += 1
        return 0

    def abs(self):
        low_byte = self.read(self.program_counter)
        self.program_counter += 1
        high_byte = self.read(self.program_counter)
        self.program_counter += 1

        self.address_absolute = (high_byte << 8) | low_byte

        return 0

    def imp(self):
        self.fetched = self.accumulator
        return 0

    # Instructions
    def adc(self):
        self.fetch()

        temp = self.accumulator + self.fetched + self.get_flag(self.C)

        self.set_flag(self.C, temp > 256)

        self.set_flag(self.Z, (temp & 0x00ff) == 0)

        self.set_flag(self.V, (~(self.accumulator ^ self.fetched) & (self.accumulator ^ temp)) & 0x0080)

        self.set_flag(self.N, temp & 0x80)

        self.accumulator = temp & 0x00ff

        return 1

    def ldx(self):

        self.fetch()

        self.x = self.fetched
        self.set_flag(self.Z, self.x == 0x00)
        self.set_flag(self.N, self.x & 0x80)
        return 1

    def ldy(self):

        self.fetch()

        self.y = self.fetched
        self.set_flag(self.Z, self.y == 0x00)
        self.set_flag(self.N, self.y & 0x80)
        return 1

    def lda(self):
        self.fetch()

        self.accumulator = self.fetched
        self.set_flag(self.Z, self.accumulator == 0x00)
        self.set_flag(self.N, self.accumulator & 0x80)
        return 1

    def stx(self):
        self.write(self.address_absolute, self.x)

    def clc(self):
        self.set_flag(self.C, False)
        return 0