import pygame as pg
from bus import Bus

pg.init()

size = (700, 500)
screen = pg.display.set_mode(size)
pg.display.set_caption("NES")

running = True

clock = pg.time.Clock()

bus = Bus()

font = pg.font.SysFont('Courier', 12)

red = pg.color.Color("red")
white = pg.color.Color("white")
black = pg.color.Color("black")
blue = pg.color.Color("blue")
green = pg.color.Color("green")

while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False

    screen.fill(white)

    x, y = 500, 0

    screen.blit(font.render("STATUS:", True, black), (x, y))
    screen.blit(font.render("N", True, green if bus.cpu.status & bus.cpu.N else red), (x + 64, y))
    screen.blit(font.render("V", True, green if bus.cpu.status & bus.cpu.V else red), (x + 80, y))
    screen.blit(font.render("-", True, green if bus.cpu.status & bus.cpu.U else red), (x + 96, y))
    screen.blit(font.render("B", True, green if bus.cpu.status & bus.cpu.B else red), (x + 112, y))
    screen.blit(font.render("D", True, green if bus.cpu.status & bus.cpu.D else red), (x + 128, y))
    screen.blit(font.render("I", True, green if bus.cpu.status & bus.cpu.I else red), (x + 144, y))
    screen.blit(font.render("Z", True, green if bus.cpu.status & bus.cpu.Z else red), (x + 160, y))
    screen.blit(font.render("C", True, green if bus.cpu.status & bus.cpu.C else red), (x + 176, y))
    screen.blit(font.render("PC: ${:04x}".format(bus.cpu.program_counter), True, black), (x, y + 12))
    screen.blit(font.render("A: ${:02x}  [{}]".format(bus.cpu.accumulator, bus.cpu.accumulator), True, black), (x, y + 24))
    screen.blit(font.render("X: ${:02x}  [{}]".format(bus.cpu.x, bus.cpu.x), True, black), (x, y + 36))
    screen.blit(font.render("Y: ${:02x}  [{}]".format(bus.cpu.y, bus.cpu.y), True, black), (x, y + 48))
    screen.blit(font.render("Stack P: ${:04x}".format(bus.cpu.stack_pointer), True, black), (x, y + 60))

    pg.display.flip()

    clock.tick(60)

pg.quit()
