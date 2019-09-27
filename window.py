import pygame as pg
from bus import Bus

pg.init()

size = (700, 500)
screen = pg.display.set_mode(size)
pg.display.set_caption("NES")

running = True

clock = pg.time.Clock()

nes = Bus()

font = pg.font.SysFont('Courier', 12)

red = pg.color.Color("red")
white = pg.color.Color("white")
black = pg.color.Color("black")
blue = pg.color.Color("blue")
green = pg.color.Color("green")


test_code = bytes.fromhex('A2 0A 8E 00 00 A2 03 8E 01 00 AC 00 00 A9 00 18 6D 01 00 88 D0 FA 8D 02 00 EA EA EA')
ram_offset = 0x8000

for byte in test_code:
    nes.ram[ram_offset] = byte
    ram_offset += 1

nes.ram[0xFFFC] = 0x0000
nes.ram[0xFFFD] = 0x8000

# mapAsm = nes.cpu.disassemble(0x0000, 0xFFFF);

nes.cpu.reset()

# code_map = nes.cpu.disassemble()

while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False



    screen.fill(white)

    # DrawRam(2, 2, 0x0000, 16, 16);
    # DrawRam(2, 182, 0x8000, 16, 16);

    x, y = 2, 0
    current_address = 0x0000
    for row in range(16):
        row_text = '${:04x}:'.format(current_address)
        for column in range(16):
            row_text += ' {:02x}'.format(nes.read(current_address, True))
            current_address += 1

        screen.blit(font.render(row_text, True, black), (x, y))

        y += 12

    x, y = 2, 204
    current_address = 0x8000
    for row in range(16):
        row_text = '${:04x}:'.format(current_address)
        for column in range(16):
            row_text += ' {:02x}'.format(nes.read(current_address, True))
            current_address += 1

        screen.blit(font.render(row_text, True, black), (x, y))

        y += 12

    # DrawCode(448, 72, 26);

    x, y = 500, 0

    screen.blit(font.render("STATUS:", True, black), (x, y))
    screen.blit(font.render("N", True, green if nes.cpu.status & nes.cpu.N else red), (x + 64, y))
    screen.blit(font.render("V", True, green if nes.cpu.status & nes.cpu.V else red), (x + 80, y))
    screen.blit(font.render("-", True, green if nes.cpu.status & nes.cpu.U else red), (x + 96, y))
    screen.blit(font.render("B", True, green if nes.cpu.status & nes.cpu.B else red), (x + 112, y))
    screen.blit(font.render("D", True, green if nes.cpu.status & nes.cpu.D else red), (x + 128, y))
    screen.blit(font.render("I", True, green if nes.cpu.status & nes.cpu.I else red), (x + 144, y))
    screen.blit(font.render("Z", True, green if nes.cpu.status & nes.cpu.Z else red), (x + 160, y))
    screen.blit(font.render("C", True, green if nes.cpu.status & nes.cpu.C else red), (x + 176, y))
    screen.blit(font.render("PC: ${:04x}".format(nes.cpu.program_counter), True, black), (x, y + 12))
    screen.blit(font.render("A: ${:02x}  [{}]".format(nes.cpu.accumulator, nes.cpu.accumulator), True, black), (x, y + 24))
    screen.blit(font.render("X: ${:02x}  [{}]".format(nes.cpu.x, nes.cpu.x), True, black), (x, y + 36))
    screen.blit(font.render("Y: ${:02x}  [{}]".format(nes.cpu.y, nes.cpu.y), True, black), (x, y + 48))
    screen.blit(font.render("Stack P: ${:04x}".format(nes.cpu.stack_pointer), True, black), (x, y + 60))

    screen.blit(font.render('Clocks: {}'.format(nes.cpu.clock_count), True, black), (0, 480))

    pg.display.flip()

    nes.cpu.clock()

    clock.tick(60)

pg.quit()
