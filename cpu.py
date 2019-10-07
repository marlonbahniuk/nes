import numpy as np

# nestest_log = open('nestest.log', 'r')


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
        ('BRK', 'brk', 'imm', 7), ('ORA', 'ora', 'izx', 6), ('???0', 'xxx', 'imp', 2), ('???1', 'xxx', 'imp', 8),
        ('NOP', 'nop', 'imp', 3), ('ORA', 'ora', 'zp0', 3), ('ASL', 'asl', 'zp0', 5), ('???2', 'xxx', 'imp', 5),
        ('PHP', 'php', 'imp', 3), ('ORA', 'ora', 'imm', 2), ('ASL', 'asl', 'imp', 2), ('???3', 'xxx', 'imp', 2),
        ('NOP', 'nop', 'imp', 4), ('ORA', 'ora', 'abs', 4), ('ASL', 'asl', 'abs', 6), ('???4', 'xxx', 'imp', 6),
        ('BPL', 'bpl', 'rel', 2), ('ORA', 'ora', 'izy', 5), ('???5', 'xxx', 'imp', 2), ('???6', 'xxx', 'imp', 8),
        ('NOP', 'nop', 'imp', 4), ('ORA', 'ora', 'zpx', 4), ('ASL', 'asl', 'zpx', 6), ('???7', 'xxx', 'imp', 6),
        ('CLC', 'clc', 'imp', 2), ('ORA', 'ora', 'aby', 4), ('NOP', 'nop', 'imp', 2), ('???8', 'xxx', 'imp', 7),
        ('NOP', 'nop', 'imp', 4), ('ORA', 'ora', 'abx', 4), ('ASL', 'asl', 'abx', 7), ('???9', 'xxx', 'imp', 7),
        ('JSR', 'jsr', 'abs', 6), ('AND', 'and_', 'izx', 6), ('???10', 'xxx', 'imp', 2), ('???11', 'xxx', 'imp', 8),
        ('BIT', 'bit', 'zp0', 3), ('AND', 'and_', 'zp0', 3), ('ROL', 'rol', 'zp0', 5), ('???12', 'xxx', 'imp', 5),
        ('PLP', 'plp', 'imp', 4), ('AND', 'and_', 'imm', 2), ('ROL', 'rol', 'imp', 2), ('???13', 'xxx', 'imp', 2),
        ('BIT', 'bit', 'abs', 4), ('AND', 'and_', 'abs', 4), ('ROL', 'rol', 'abs', 6), ('???14', 'xxx', 'imp', 6),
        ('BMI', 'bmi', 'rel', 2), ('AND', 'and_', 'izy', 5), ('???15', 'xxx', 'imp', 2), ('???16', 'xxx', 'imp', 8),
        ('NOP', 'nop', 'imp', 4), ('AND', 'and_', 'zpx', 4), ('ROL', 'rol', 'zpx', 6), ('???17', 'xxx', 'imp', 6),
        ('SEC', 'sec', 'imp', 2), ('AND', 'and_', 'aby', 4), ('NOP', 'nop', 'imp', 2), ('???18', 'xxx', 'imp', 7),
        ('NOP', 'nop', 'imp', 4), ('AND', 'and_', 'abx', 4), ('ROL', 'rol', 'abx', 7), ('???19', 'xxx', 'imp', 7),
        ('RTI', 'rti', 'imp', 6), ('EOR', 'eor', 'izx', 6), ('???20', 'xxx', 'imp', 2), ('???21', 'xxx', 'imp', 8),
        ('NOP', 'nop', 'imp', 3), ('EOR', 'eor', 'zp0', 3), ('LSR', 'lsr', 'zp0', 5), ('???22', 'xxx', 'imp', 5),
        ('PHA', 'pha', 'imp', 3), ('EOR', 'eor', 'imm', 2), ('LSR', 'lsr', 'imp', 2), ('???23', 'xxx', 'imp', 2),
        ('JMP', 'jmp', 'abs', 3), ('EOR', 'eor', 'abs', 4), ('LSR', 'lsr', 'abs', 6), ('???24', 'xxx', 'imp', 6),
        ('BVC', 'bvc', 'rel', 2), ('EOR', 'eor', 'izy', 5), ('???25', 'xxx', 'imp', 2), ('???26', 'xxx', 'imp', 8),
        ('NOP', 'nop', 'imp', 4), ('EOR', 'eor', 'zpx', 4), ('LSR', 'lsr', 'zpx', 6), ('???27', 'xxx', 'imp', 6),
        ('CLI', 'cli', 'imp', 2), ('EOR', 'eor', 'aby', 4), ('NOP', 'nop', 'imp', 2), ('???28', 'xxx', 'imp', 7),
        ('NOP', 'nop', 'imp', 4), ('EOR', 'eor', 'abx', 4), ('LSR', 'lsr', 'abx', 7), ('???29', 'xxx', 'imp', 7),
        ('RTS', 'rts', 'imp', 6), ('ADC', 'adc', 'izx', 6), ('???30', 'xxx', 'imp', 2), ('???31', 'xxx', 'imp', 8),
        ('NOP', 'nop', 'imp', 3), ('ADC', 'adc', 'zp0', 3), ('ROR', 'ror', 'zp0', 5), ('???32', 'xxx', 'imp', 5),
        ('PLA', 'pla', 'imp', 4), ('ADC', 'adc', 'imm', 2), ('ROR', 'ror', 'imp', 2), ('???33', 'xxx', 'imp', 2),
        ('JMP', 'jmp', 'ind', 5), ('ADC', 'adc', 'abs', 4), ('ROR', 'ror', 'abs', 6), ('???34', 'xxx', 'imp', 6),
        ('BVS', 'bvs', 'rel', 2), ('ADC', 'adc', 'izy', 5), ('???35', 'xxx', 'imp', 2), ('???36', 'xxx', 'imp', 8),
        ('NOP', 'nop', 'imp', 4), ('ADC', 'adc', 'zpx', 4), ('ROR', 'ror', 'zpx', 6), ('???37', 'xxx', 'imp', 6),
        ('SEI', 'sei', 'imp', 2), ('ADC', 'adc', 'aby', 4), ('NOP', 'nop', 'imp', 2), ('???38', 'xxx', 'imp', 7),
        ('NOP', 'nop', 'imp', 4), ('ADC', 'adc', 'abx', 4), ('ROR', 'ror', 'abx', 7), ('???39', 'xxx', 'imp', 7),
        ('NOP', 'nop', 'imp', 2), ('STA', 'sta', 'izx', 6), ('NOP', 'nop', 'imp', 2), ('SAX', 'sax', 'izx', 6),
        ('STY', 'sty', 'zp0', 3), ('STA', 'sta', 'zp0', 3), ('STX', 'stx', 'zp0', 3), ('SAX', 'sax', 'zp0', 3),
        ('DEY', 'dey', 'imp', 2), ('NOP', 'nop', 'imp', 2), ('TXA', 'txa', 'imp', 2), ('???42', 'xxx', 'imp', 2),
        ('STY', 'sty', 'abs', 4), ('STA', 'sta', 'abs', 4), ('STX', 'stx', 'abs', 4), ('SAX', 'sax', 'abs', 4),
        ('BCC', 'bcc', 'rel', 2), ('STA', 'sta', 'izy', 6), ('???44', 'xxx', 'imp', 2), ('???45', 'xxx', 'imp', 6),
        ('STY', 'sty', 'zpx', 4), ('STA', 'sta', 'zpx', 4), ('STX', 'stx', 'zpy', 4), ('SAX', 'sax', 'zpy', 4),
        ('TYA', 'tya', 'imp', 2), ('STA', 'sta', 'aby', 5), ('TXS', 'txs', 'imp', 2), ('???47', 'xxx', 'imp', 5),
        ('NOP', 'nop', 'imp', 5), ('STA', 'sta', 'abx', 5), ('???48', 'xxx', 'imp', 5), ('???49', 'xxx', 'imp', 5),
        ('LDY', 'ldy', 'imm', 2), ('LDA', 'lda', 'izx', 6), ('LDX', 'ldx', 'imm', 2), ('LAX', 'lax', 'izx', 6),
        ('LDY', 'ldy', 'zp0', 3), ('LDA', 'lda', 'zp0', 3), ('LDX', 'ldx', 'zp0', 3), ('LAX', 'lax', 'zp0', 3),
        ('TAY', 'tay', 'imp', 2), ('LDA', 'lda', 'imm', 2), ('TAX', 'tax', 'imp', 2), ('???50', 'xxx', 'imp', 2),
        ('LDY', 'ldy', 'abs', 4), ('LDA', 'lda', 'abs', 4), ('LDX', 'ldx', 'abs', 4), ('LAX', 'lax', 'abs', 4),
        ('BCS', 'bcs', 'rel', 2), ('LDA', 'lda', 'izy', 5), ('???52', 'xxx', 'imp', 2), ('LAX', 'lax', 'izy', 5),
        ('LDY', 'ldy', 'zpx', 4), ('LDA', 'lda', 'zpx', 4), ('LDX', 'ldx', 'zpy', 4), ('LAX', 'lax', 'zpy', 4),
        ('CLV', 'clv', 'imp', 2), ('LDA', 'lda', 'aby', 4), ('TSX', 'tsx', 'imp', 2), ('???55', 'xxx', 'imp', 4),
        ('LDY', 'ldy', 'abx', 4), ('LDA', 'lda', 'abx', 4), ('LDX', 'ldx', 'aby', 4), ('LAX', 'lax', 'aby', 4),
        ('CPY', 'cpy', 'imm', 2), ('CMP', 'cmp', 'izx', 6), ('NOP', 'nop', 'imp', 2), ('DCP', 'dcp', 'izx', 8),
        ('CPY', 'cpy', 'zp0', 3), ('CMP', 'cmp', 'zp0', 3), ('DEC', 'dec', 'zp0', 5), ('DCP', 'dcp', 'zp0', 5),
        ('INY', 'iny', 'imp', 2), ('CMP', 'cmp', 'imm', 2), ('DEX', 'dex', 'imp', 2), ('???59', 'xxx', 'imp', 2),
        ('CPY', 'cpy', 'abs', 4), ('CMP', 'cmp', 'abs', 4), ('DEC', 'dec', 'abs', 6), ('DCP', 'dcp', 'abs', 6),
        ('BNE', 'bne', 'rel', 2), ('CMP', 'cmp', 'izy', 5), ('???61', 'xxx', 'imp', 2), ('DCP', 'dcp', 'izy', 8),
        ('NOP', 'nop', 'imp', 4), ('CMP', 'cmp', 'zpx', 4), ('DEC', 'dec', 'zpx', 6), ('DCP', 'dcp', 'zpy', 6),
        ('CLD', 'cld', 'imp', 2), ('CMP', 'cmp', 'aby', 4), ('NOP', 'nop', 'imp', 2), ('???64', 'xxx', 'imp', 7),
        ('NOP', 'nop', 'imp', 4), ('CMP', 'cmp', 'abx', 4), ('DEC', 'dec', 'abx', 7), ('???65', 'xxx', 'imp', 7),
        ('CPX', 'cpx', 'imm', 2), ('SBC', 'sbc', 'izx', 6), ('NOP', 'nop', 'imp', 2), ('???66', 'xxx', 'imp', 8),
        ('CPX', 'cpx', 'zp0', 3), ('SBC', 'sbc', 'zp0', 3), ('INC', 'inc', 'zp0', 5), ('???67', 'xxx', 'imp', 5),
        ('INX', 'inx', 'imp', 2), ('SBC', 'sbc', 'imm', 2), ('NOP', 'nop', 'imp', 2), ('SBC', 'sbc', 'imm', 2),
        ('CPX', 'cpx', 'abs', 4), ('SBC', 'sbc', 'abs', 4), ('INC', 'inc', 'abs', 6), ('???69', 'xxx', 'imp', 6),
        ('BEQ', 'beq', 'rel', 2), ('SBC', 'sbc', 'izy', 5), ('???70', 'xxx', 'imp', 2), ('???71', 'xxx', 'imp', 8),
        ('NOP', 'nop', 'imp', 4), ('SBC', 'sbc', 'zpx', 4), ('INC', 'inc', 'zpx', 6), ('???72', 'xxx', 'imp', 6),
        ('SED', 'sed', 'imp', 2), ('SBC', 'sbc', 'aby', 4), ('NOP', 'nop', 'imp', 2), ('???73', 'xxx', 'imp', 7),
        ('NOP', 'nop', 'imp', 4), ('SBC', 'sbc', 'abx', 4), ('INC', 'inc', 'abx', 7), ('???74', 'xxx', 'imp', 7),
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
        low_byte = np.uint8(self.read(self.address_absolute + 0))
        high_byte = np.uint8(self.read(self.address_absolute + 1))
        self.program_counter = np.uint16((high_byte << 8) | low_byte)

        # Registers
        self.accumulator = 0
        self.x = 0
        self.y = 0
        self.stack_pointer = 0xfd
        self.status = 0x24

        # Helpers
        self.address_relative = 0x0000
        self.address_absolute = 0x0000
        self.fetched = 0x00

        self.remaining_cycles = 8

        self.clock_count = 0

    def complete(self):
        return self.remaining_cycles == 0

    def get_flag(self, flag):
        return 1 if (self.status & flag) > 0 else 0

    def set_flag(self, flag, value):
        if value:
            self.status |= flag
        else:
            self.status &= ~flag

    def read(self, address):
         return self.bus.cpu_read(address, False)

    def write(self, address, data):
        self.bus.cpu_write(address, data)

    def clock(self):
        if self.remaining_cycles == 0:
            self.opcode = self.read(self.program_counter)

            current_pc = self.program_counter
            current_status = self.status
            current_sp = self.stack_pointer
            current_a = self.accumulator
            current_y = self.y
            current_x = self.x

            self.program_counter += 1

            instruction_data = self.lookup[self.opcode]

            cycles = instruction_data[3]
            addressing_mode = getattr(self, instruction_data[2], None)
            instruction = getattr(self, instruction_data[1], None)

            if addressing_mode and instruction:
                additional_cycle_1 = addressing_mode()
                additional_cycle_2 = instruction()

                cycles += 1 if additional_cycle_1 and additional_cycle_2 else 0

            else:
                print('{}, {} not found'.format(instruction_data[1], instruction_data[2]))

            self.remaining_cycles = cycles

            self.set_flag(self.U, True)

            # log_line = nestest_log.readline()
            print('{:04X} {:02X} {:s} {:s} {:s}'.format(current_pc, self.opcode, instruction_data[0], instruction_data[1], instruction_data[2]))

            # log_pc = int(log_line[0:4], 16)
            # log_instruction = log_line[16:19]
            # log_a = int(log_line[50:52], 16)
            # log_x = int(log_line[55:57], 16)
            # log_y = int(log_line[60:62], 16)
            # log_p = int(log_line[65:67], 16)
            # log_sp = int(log_line[71:73], 16)
#C000  4C F5 C5  JMP $C5F5                       A:00 X:00 Y:00 P:24 SP:FD CYC:  0 SL:241

            # if current_a != log_a or current_x != log_x or current_y != log_y or current_pc != log_pc or current_sp != log_sp or current_status != log_p:
            #     print('something went wrong')


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
    def imp(self):
        self.fetched = self.accumulator
        return 0

    def imm(self):
        self.address_absolute = self.program_counter
        self.program_counter += 1
        return 0

    def zp0(self):
        self.address_absolute = self.read(self.program_counter)
        self.program_counter += 1
        self.address_absolute &= 0x00ff
        return 0

    def zpx(self):
        self.address_absolute = self.read(self.program_counter) + self.x
        self.program_counter += 1
        self.address_absolute &= 0x00ff
        return 0

    def zpy(self):
        self.address_absolute = self.read(self.program_counter) + self.y
        self.program_counter += 1
        self.address_absolute &= 0x00ff
        return 0

    def rel(self):
        self.address_relative = self.read(self.program_counter)
        self.program_counter += 1

        if self.address_relative & 0x80:
            self.address_relative |= 0xff00

        return 0

    def abs(self):
        low_byte = self.read(self.program_counter)
        self.program_counter += 1
        high_byte = self.read(self.program_counter)
        self.program_counter += 1

        self.address_absolute = (high_byte << 8) | low_byte

        return 0

    def abx(self):
        low_byte = self.read(self.program_counter)
        self.program_counter += 1
        high_byte = self.read(self.program_counter)
        self.program_counter += 1

        self.address_absolute = (high_byte << 8) | low_byte
        self.address_absolute += self.x

        if self.address_absolute & 0xff00 != high_byte << 8:
            return 1
        else:
            return 0

    def aby(self):
        low_byte = self.read(self.program_counter)
        self.program_counter += 1
        high_byte = self.read(self.program_counter)
        self.program_counter += 1

        self.address_absolute = (high_byte << 8) | low_byte
        self.address_absolute = np.int16(self.address_absolute + self.y)

        if self.address_absolute & 0xff00 != high_byte << 8:
            return 1
        else:
            return 0

    def ind(self):
        low_byte = self.read(self.program_counter)
        self.program_counter += 1
        high_byte = self.read(self.program_counter)
        self.program_counter += 1

        address = (high_byte << 8) | low_byte

        if low_byte == 0x00ff:
            self.address_absolute = (self.read(address & 0xff00) << 8) | self.read(address + 0)
        else:
            self.address_absolute = (self.read(address + 1) << 8) | self.read(address + 0)

        return 0

    def izx(self):
        temp = self.read(self.program_counter)
        self.program_counter += 1

        low_byte = self.read((temp + self.x) & 0x00ff)
        high_byte = self.read((temp + self.x + 1) & 0x00ff)

        self.address_absolute = (high_byte << 8) | low_byte

        return 0

    def izy(self):
        temp = self.read(self.program_counter)
        self.program_counter += 1

        low_byte = self.read(temp & 0x00ff)
        high_byte = self.read((temp + 1) & 0x00ff)

        self.address_absolute = (high_byte << 8) | low_byte
        self.address_absolute = self.address_absolute + self.y

        if (self.address_absolute & 0xFF00) != (high_byte << 8):
            return 1
        else:
            return 0

    # Instructions
    def and_(self):
        self.fetch()
        self.accumulator &= self.fetched
        self.set_flag(self.Z, self.accumulator == 0x00)
        self.set_flag(self.N, self.accumulator & 0x80)
        return 1

    def adc(self):
        self.fetch()

        temp = self.accumulator + self.fetched + self.get_flag(self.C)

        self.set_flag(self.C, temp > 255)

        self.set_flag(self.Z, (temp & 0x00ff) == 0)

        self.set_flag(self.V, (~(self.accumulator ^ self.fetched) & (self.accumulator ^ temp)) & 0x0080)

        self.set_flag(self.N, temp & 0x80)

        self.accumulator = temp & 0x00ff

        return 1

    def sbc(self):
        self.fetch()

        value = self.fetched ^ 0x00ff

        temp = self.accumulator + value + self.get_flag(self.C)

        self.set_flag(self.C, temp & 0xff00)

        self.set_flag(self.Z, (temp & 0x00ff) == 0)

        self.set_flag(self.V, (temp ^ self.accumulator) & (temp ^ value) & 0x0080)

        self.set_flag(self.N, temp & 0x0080)

        self.accumulator = temp & 0x00ff

        return 1

    def cmp(self):
        self.fetch()

        temp = self.accumulator - self.fetched
        self.set_flag(self.C, self.accumulator >= self.fetched)
        self.set_flag(self.Z, temp & 0x00ff == 0x0000)
        self.set_flag(self.N, temp & 0x0080)
        return 1

    def cpx(self):
        self.fetch()

        temp = self.x - self.fetched
        self.set_flag(self.C, self.x >= self.fetched)
        self.set_flag(self.Z, temp & 0x00ff == 0x0000)
        self.set_flag(self.N, temp & 0x0080)
        return 0

    def cpy(self):
        self.fetch()

        temp = self.y - self.fetched
        self.set_flag(self.C, self.y >= self.fetched)
        self.set_flag(self.Z, temp & 0x00ff == 0x0000)
        self.set_flag(self.N, temp & 0x0080)
        return 0

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

    def lax(self):
        self.fetch()

        self.accumulator = self.x = self.fetched

        self.set_flag(self.Z, self.accumulator == 0x00)
        self.set_flag(self.N, self.accumulator & 0x80)

        return 0

    def sax(self):
        self.write(self.address_absolute, self.accumulator & self.x)
        return 0

    def dcp(self):
        self.fetch()
        value = np.uint8(self.fetched - 1)
        self.write(self.address_absolute, value & 0x00FF)
        self.set_flag(self.Z, (value & 0x00FF) == 0x0000)
        self.set_flag(self.N, value & 0x0080)

        temp = self.accumulator - value
        self.set_flag(self.C, self.accumulator >= value)
        self.set_flag(self.Z, temp & 0x00ff == 0x0000)
        self.set_flag(self.N, temp & 0x0080)

    def sta(self):
        self.write(self.address_absolute, self.accumulator)
        return 0

    def stx(self):
        self.write(self.address_absolute, self.x)
        return 0

    def sty(self):
        self.write(self.address_absolute, self.y)
        return 0

    def dey(self):
        self.y = np.uint8(self.y - 1)

        self.set_flag(self.Z, self.y == 0x00)
        self.set_flag(self.N, self.y & 0x80)

        return 0

    def dex(self):
        self.x = np.uint8(self.x - 1)
        self.set_flag(self.Z, self.x == 0x00)
        self.set_flag(self.N, self.x & 0x80)

        return 0

    def inc(self):
        self.fetch()
        temp = self.fetched + 1
        self.write(self.address_absolute, temp & 0x00ff)
        self.set_flag(self.Z, temp & 0x00ff == 0x0000)
        self.set_flag(self.N, temp & 0x0080)

        return 0

    def iny(self):
        self.y = np.uint8(self.y + 1)
        self.set_flag(self.Z, self.y & 0x00ff == 0x0000)
        self.set_flag(self.N, self.y & 0x0080)

        return 0

    def inx(self):
        self.x = np.uint8(self.x + 1)
        self.set_flag(self.Z, self.x & 0x00ff == 0x0000)
        self.set_flag(self.N, self.x & 0x0080)

        return 0

    def tay(self):
        self.y = self.accumulator
        self.set_flag(self.Z, self.y & 0x00ff == 0x00)
        self.set_flag(self.N, self.y & 0x80)
        return 0

    def tax(self):
        self.x = self.accumulator
        self.set_flag(self.Z, self.x & 0x00ff == 0x00)
        self.set_flag(self.N, self.x & 0x80)
        return 0

    def tsx(self):
        self.x = self.stack_pointer
        self.set_flag(self.Z, self.x & 0x00ff == 0x00)
        self.set_flag(self.N, self.x & 0x80)
        return 0

    def tya(self):
        self.accumulator = self.y
        self.set_flag(self.Z, self.accumulator & 0x00ff == 0x00)
        self.set_flag(self.N, self.accumulator & 0x80)
        return 0

    def txa(self):
        self.accumulator = self.x
        self.set_flag(self.Z, self.accumulator & 0x00ff == 0x00)
        self.set_flag(self.N, self.accumulator & 0x80)
        return 0

    def txs(self):
        self.stack_pointer = self.x
        return 0

    def rti(self):
        self.stack_pointer += 1
        self.status = self.read(0x0100 + self.stack_pointer)

        self.status &= ~self.B
        self.status &= ~self.U

        self.stack_pointer += 1
        self.program_counter = self.read(0x0100 + self.stack_pointer)
        self.stack_pointer += 1
        self.program_counter |= self.read(0x0100 + self.stack_pointer) << 8
        return 0

    def lsr(self):
        self.fetch()
        self.set_flag(self.C, self.fetched & 0x0001)
        temp = self.fetched >> 1

        self.set_flag(self.Z, temp & 0x00ff == 0x0000)
        self.set_flag(self.N, temp & 0x0080)

        if self.lookup[self.opcode][2] == 'imp':
            self.accumulator = temp & 0x00ff
        else:
            self.write(self.address_absolute, temp & 0x00ff)

        return 0

    def asl(self):
        self.fetch()
        temp = self.fetched << 1
        self.set_flag(self.C, temp & 0xff00 > 0)
        self.set_flag(self.Z, temp & 0x00ff == 0x00)
        self.set_flag(self.N, temp & 0x80)

        if self.lookup[self.opcode][2] == 'imp':
            self.accumulator = temp & 0x00ff
        else:
            self.write(self.address_absolute, temp & 0x00ff)

        return 0

    def ror(self):
        self.fetch()
        temp = (self.get_flag(self.C) << 7) | (self.fetched >> 1)
        self.set_flag(self.C, self.fetched & 0x01)
        self.set_flag(self.Z, temp & 0x00ff == 0x00)
        self.set_flag(self.N, temp & 0x80)

        if self.lookup[self.opcode][2] == 'imp':
            self.accumulator = temp & 0x00ff
        else:
            self.write(self.address_absolute, temp & 0x00ff)

        return 0

    def rol(self):
        self.fetch()
        temp = (self.fetched << 1) | self.get_flag(self.C)
        self.set_flag(self.C, temp & 0xff00)
        self.set_flag(self.Z, temp & 0x00ff == 0x0000)
        self.set_flag(self.N, temp & 0x0080)

        if self.lookup[self.opcode][2] == 'imp':
            self.accumulator = temp & 0x00ff
        else:
            self.write(self.address_absolute, temp & 0x00ff)

        return 0

    def dec(self):
        self.fetch()
        temp = self.fetched - 1
        self.write(self.address_absolute, temp & 0x00FF)
        self.set_flag(self.Z, (temp & 0x00FF) == 0x0000)
        self.set_flag(self.N, temp & 0x0080)
        return 0

    def bne(self):
        if self.get_flag(self.Z) == 0:
            self.remaining_cycles += 1
            self.address_absolute = self.program_counter + np.int8(self.address_relative)

            if self.address_absolute & 0xff00 and self.program_counter & 0xff00:
                self.remaining_cycles += 1

            self.program_counter = self.address_absolute

        return 0

    def jmp(self):
        self.program_counter = self.address_absolute
        return 0

    def jsr(self):
        self.program_counter -= 1

        self.write(0x0100 + self.stack_pointer, (self.program_counter >> 8) & 0x00ff)
        self.stack_pointer -= 1
        self.write(0x0100 + self.stack_pointer, self.program_counter & 0x00ff)
        self.stack_pointer -= 1

        self.program_counter = self.address_absolute

        return 0

    def bcs(self):
        if self.get_flag(self.C) == 1:
            self.remaining_cycles += 1
            self.address_absolute = self.program_counter + self.address_relative

            if self.address_absolute & 0xff00 != self.program_counter & 0xff00:
                self.remaining_cycles += 1

            self.program_counter = self.address_absolute

        return 0

    def bcc(self):
        if self.get_flag(self.C) == 0:
            self.remaining_cycles += 1
            self.address_absolute = self.program_counter + self.address_relative

            if self.address_absolute & 0xff00 != self.program_counter & 0xff00:
                self.remaining_cycles += 1

            self.program_counter = self.address_absolute

        return 0

    def beq(self):
        if self.get_flag(self.Z) == 1:
            self.remaining_cycles += 1
            self.address_absolute = self.program_counter + self.address_relative

            if self.address_absolute & 0xff00 != self.program_counter & 0xff00:
                self.remaining_cycles += 1

            self.program_counter = self.address_absolute

        return 0

    def bmi(self):
        if self.get_flag(self.N) == 1:
            self.remaining_cycles += 1
            self.address_absolute = self.program_counter + self.address_relative

            if self.address_absolute & 0xff00 != self.program_counter & 0xff00:
                self.remaining_cycles += 1

            self.program_counter = self.address_absolute

        return 0

    def bvc(self):
        if self.get_flag(self.V) == 0:
            self.remaining_cycles += 1
            self.address_absolute = self.program_counter + self.address_relative

            if self.address_absolute & 0xff00 != self.program_counter & 0xff00:
                self.remaining_cycles += 1

            self.program_counter = self.address_absolute

        return 0

    def bvs(self):
        if self.get_flag(self.V) == 1:
            self.remaining_cycles += 1
            self.address_absolute = self.program_counter + self.address_relative

            if self.address_absolute & 0xff00 != self.program_counter & 0xff00:
                self.remaining_cycles += 1

            self.program_counter = self.address_absolute

        return 0

    def bpl(self):
        if self.get_flag(self.N) == 0:
            self.remaining_cycles += 1
            self.address_absolute = self.program_counter + self.address_relative

            if self.address_absolute & 0xff00 != self.program_counter & 0xff00:
                self.remaining_cycles += 1

            self.program_counter = self.address_absolute

        return 0

    def rts(self):
        self.stack_pointer += 1
        self.program_counter = self.read(0x0100 + self.stack_pointer)
        self.stack_pointer += 1
        self.program_counter |= self.read(0x0100 + self.stack_pointer) << 8

        self.program_counter += 1
        return 0

    def bit(self):
        self.fetch()

        temp = self.accumulator & self.fetched
        self.set_flag(self.Z, temp & 0x00ff == 0x00)
        self.set_flag(self.N, self.fetched & (1 << 7))
        self.set_flag(self.V, self.fetched & (1 << 6))
        return 0

    def eor(self):
        self.fetch()

        self.accumulator ^= self.fetched
        self.set_flag(self.Z, self.accumulator == 0x00)
        self.set_flag(self.N, self.accumulator & 0x80)
        return 1

    def ora(self):
        self.fetch()

        self.accumulator |= self.fetched
        self.set_flag(self.Z, self.accumulator == 0x00)
        self.set_flag(self.N, self.accumulator & 0x80)

    def sec(self):
        self.set_flag(self.C, True)
        return 0

    def clc(self):
        self.set_flag(self.C, False)
        return 0

    def sei(self):
        self.set_flag(self.I, True)
        return 0

    def sed(self):
        self.set_flag(self.D, True)
        return 0

    def cld(self):
        self.set_flag(self.D, False)
        return 0

    def clv(self):
        self.set_flag(self.V, False)
        return 0

    def brk(self):
        self.program_counter += 1
        self.set_flag(self.I, True)
        self.write(0x0100 + self.stack_pointer, (self.program_counter >> 8) & 0x00ff)
        self.stack_pointer -= 1
        self.write(0x0100 + self.stack_pointer, self.program_counter & 0x00ff)
        self.stack_pointer -= 1

        self.set_flag(self.B, True)
        self.write(0x0100 + self.stack_pointer, self.status)
        self.stack_pointer -= 1
        self.set_flag(self.B, False)

        self.program_counter = self.read(0xfffe) | (self.read(0xffff) << 8)
        return 0

    def pha(self):
        self.write(0x0100 + self.stack_pointer, self.accumulator)

        self.stack_pointer -= 1
        return 0

    def php(self):
        self.set_flag(self.B, True)
        self.set_flag(self.U, True)
        self.write(0x0100 + self.stack_pointer, self.status)

        self.set_flag(self.B, False)
        self.set_flag(self.U, False)

        self.stack_pointer -= 1
        return 0

    def pla(self):
        self.stack_pointer += 1
        self.accumulator = self.read(0x0100 + self.stack_pointer)
        self.set_flag(self.Z, self.accumulator == 0x00)
        self.set_flag(self.N, self.accumulator & 0x80)
        return 0

    def plp(self):
        self.stack_pointer += 1
        self.status = self.read(0x0100 + self.stack_pointer)
        self.set_flag(self.B, False)
        return 0

    def nop(self):
        if self.opcode == 0x80:
            self.program_counter += 1
        if self.opcode & 0x4 == 4:
            self.program_counter += 1
        if self.opcode & 0xC == 12:
            self.program_counter += 1
        if self.opcode == 0xFC:
            return 1
        return 0

    def xxx(self):
        return 0

    def disassemble(self, start, end):
        current_address = start

        lines = dict()

        while current_address <= end:

            instruction_address = current_address

            instruction = '${:04x}: '.format(current_address)

            opcode = self.bus.cpu_read(current_address, True)

            current_address += 1

            instruction_data = self.lookup[opcode]

            instruction += '{} '.format(instruction_data[0])

            addressing_mode = instruction_data[2]

            if addressing_mode == 'imp':
                instruction += '{IMP}'
            elif addressing_mode == 'imm':
                value = self.bus.cpu_read(current_address, True)
                current_address += 1
                instruction += '#${:02x} {{IMM}}'.format(value)
            elif addressing_mode == 'zp0':
                low_byte = self.bus.cpu_read(current_address, True)
                current_address += 1
                instruction += '${:02x} {{ZP0}}'.format(low_byte)
            elif addressing_mode == 'zpx':
                low_byte = self.bus.cpu_read(current_address, True)
                current_address += 1
                instruction += '${:02x} , X {{ZPX}}'.format(low_byte)
            elif addressing_mode == 'zpy':
                low_byte = self.bus.cpu_read(current_address, True)
                current_address += 1
                instruction += '${:02x} , Y {{ZPY}}'.format(low_byte)
            elif addressing_mode == 'izx':
                low_byte = self.bus.cpu_read(current_address, True)
                current_address += 1
                instruction += '(${:02x}, X) {{IZX}}'.format(low_byte)
            elif addressing_mode == 'izy':
                low_byte = self.bus.cpu_read(current_address, True)
                current_address += 1
                instruction += '(${:02x}, Y) {{IZY}}'.format(low_byte)
            elif addressing_mode == 'abs':
                low_byte = self.bus.cpu_read(current_address, True)
                current_address += 1
                high_byte = self.bus.cpu_read(current_address, True)
                current_address += 1
                instruction += '${:04x} {{ABS}}'.format((high_byte << 8) | low_byte)
            elif addressing_mode == 'abx':
                low_byte = self.bus.cpu_read(current_address, True)
                current_address += 1
                high_byte = self.bus.cpu_read(current_address, True)
                current_address += 1
                instruction += '${:04x}, X {{ABX}}'.format((high_byte << 8) | low_byte)
            elif addressing_mode == 'aby':
                low_byte = self.bus.cpu_read(current_address, True)
                current_address += 1
                high_byte = self.bus.cpu_read(current_address, True)
                current_address += 1
                instruction += '${:04x}, Y {{ABY}}'.format((high_byte << 8) | low_byte)
            elif addressing_mode == 'ind':
                low_byte = self.bus.cpu_read(current_address, True)
                current_address += 1
                high_byte = self.bus.cpu_read(current_address, True)
                current_address += 1
                instruction += '(${:04x}) {{IND}}'.format((high_byte << 8) | low_byte)
            elif addressing_mode == 'rel':
                value = self.bus.cpu_read(current_address, True)
                current_address += 1
                instruction += '${:02x} [${:04x}] {{REL}}'.format(value, current_address + value)

            lines[instruction_address] = instruction

        return lines
