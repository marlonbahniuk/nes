import pygame as pg

from bus import Bus
from cartridge import Cartridge

pg.init()

size = (700, 500)
screen = pg.display.set_mode(size)
pg.display.set_caption('NES')

running = True

clock = pg.time.Clock()

nes = Bus()

font = pg.font.SysFont('Courier', 12)

red = pg.color.Color('red')
white = pg.color.Color('white')
black = pg.color.Color('black')
blue = pg.color.Color('blue')
green = pg.color.Color('green')
gray = pg.color.Color('gray')

cartridge = Cartridge('nestest.nes')
nes.set_cartridge(cartridge)
nes.cpu.reset()

nes.cpu.program_counter = 0x0c000

code_map = nes.cpu.disassemble(0x0000, 0xffff)

stepping = False

while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False

        if event.type == pg.KEYDOWN:

            if event.key == pg.K_p:
                stepping = not stepping

            if stepping and event.key == pg.K_SPACE:
                complete = False

                while not complete:
                    nes.cpu.clock()

                    complete = nes.cpu.complete()
            if event.key == pg.K_r:
                nes.cpu.reset()

    if not stepping:
        complete = False

        while not complete:
            nes.cpu.clock()

            complete = nes.cpu.complete()

    screen.fill(white)

    x, y = 2, 0
    current_address = 0x0000
    for row in range(16):
        row_text = '${:04x}:'.format(current_address)
        for column in range(16):
            row_text += ' {:02x}'.format(nes.cpu_read(current_address, True))
            current_address += 1

        screen.blit(font.render(row_text, True, black), (x, y))

        y += 12

    x, y = 2, 204
    current_address = 0x8000
    for row in range(16):
        row_text = '${:04x}:'.format(current_address)
        for column in range(16):
            row_text += ' {:02x}'.format(nes.cpu_read(current_address, True))
            current_address += 1

        screen.blit(font.render(row_text, True, black), (x, y))

        y += 12

    # DrawCode(448, 72, 26);

    keys_list = sorted(code_map.keys())

    if nes.cpu.program_counter in keys_list:
        current_index = keys_list.index(nes.cpu.program_counter)

        lower_index = current_index - 5

        upper_index = current_index + 6

        x, y = 500, 200

        for index in range(lower_index, upper_index):
            color = black if index == current_index else gray

            if index >= 0:
                screen.blit(font.render(code_map[keys_list[index]], True, color), (x, y))

            y += 12

    x, y = 500, 0

    screen.blit(font.render('STATUS:', True, black), (x, y))
    screen.blit(font.render('N', True, green if nes.cpu.status & nes.cpu.N else red), (x + 64, y))
    screen.blit(font.render('V', True, green if nes.cpu.status & nes.cpu.V else red), (x + 80, y))
    screen.blit(font.render('-', True, green if nes.cpu.status & nes.cpu.U else red), (x + 96, y))
    screen.blit(font.render('B', True, green if nes.cpu.status & nes.cpu.B else red), (x + 112, y))
    screen.blit(font.render('D', True, green if nes.cpu.status & nes.cpu.D else red), (x + 128, y))
    screen.blit(font.render('I', True, green if nes.cpu.status & nes.cpu.I else red), (x + 144, y))
    screen.blit(font.render('Z', True, green if nes.cpu.status & nes.cpu.Z else red), (x + 160, y))
    screen.blit(font.render('C', True, green if nes.cpu.status & nes.cpu.C else red), (x + 176, y))
    screen.blit(font.render('PC: ${:04x}'.format(nes.cpu.program_counter), True, black), (x, y + 12))
    screen.blit(font.render('A: ${:02x}  [{}]'.format(nes.cpu.accumulator, nes.cpu.accumulator), True, black),
                (x, y + 24))
    screen.blit(font.render('X: ${:02x}  [{}]'.format(nes.cpu.x, nes.cpu.x), True, black), (x, y + 36))
    screen.blit(font.render('Y: ${:02x}  [{}]'.format(nes.cpu.y, nes.cpu.y), True, black), (x, y + 48))
    screen.blit(font.render('Stack P: ${:04x}'.format(nes.cpu.stack_pointer), True, black), (x, y + 60))

    screen.blit(font.render('Clocks: {}'.format(nes.cpu.clock_count), True, black), (0, 480))

    pg.display.set_caption('NES - {:02.0f} fps'.format(clock.get_fps()))
    pg.display.flip()

    nes.ppu.get_screen()

    clock.tick()

pg.quit()
