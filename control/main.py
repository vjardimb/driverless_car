import pygame

from car import PlayerCar
from path import gen_path, get_speeds, get_start_point
from utils import draw, scale_image


# control variables
STEER_CONTROL = 1
SPEED_CONTROL = 1
KEEP_GOING = 0
control_info = (STEER_CONTROL, SPEED_CONTROL, KEEP_GOING)

# simulation variables
run = True
clock = pygame.time.Clock()
FPS = 60

# images to display
GRASS = scale_image(pygame.image.load("imgs/grass.jpg"), 2.5)  # not being used
TRACK_BORDER = scale_image(pygame.image.load("imgs/track-border.png"), 0.9)  # not being used
TRACK = scale_image(pygame.image.load("imgs/track.png"), 0.6)  # not being used
images = [(GRASS, (0, 0)), (TRACK, (0, 0))]  # [(GRASS, (0, 0)), (TRACK, (0, 0))] or [(GRASS, (0, 0))]

# configure pygame display
WIDTH, HEIGHT = TRACK.get_width(), TRACK.get_height()
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Racing Game!")

# generate path and get proper speeds for each point
spl_dots, spl_array, tck = gen_path(WIN, WIDTH, False)  # position references
trgt_speeds = get_speeds(tck)  # speed references

# Get initial position
init_x, init_y = get_start_point(spl_array, standard=0)

# Initialize car
player_car = PlayerCar(1, (init_x, init_y))

while run:
    clock.tick(FPS)
    draw(WIN, images, player_car)
    WIN.fill(0)  # set background color (black)

    for dot in spl_dots:
        dot.draw()

    keys = pygame.key.get_pressed()

    # apply actuation commands (steering angle rate and acceleration)
    player_car.controller(WIN, spl_array, keys, trgt_speeds, control_info)

    # Update car state (x, y and theta)
    player_car.move()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
            break

pygame.quit()
