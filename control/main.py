import pygame
import time
import math

from car import PlayerCar
from path import gen_path, get_closest_point, get_speeds, get_top_point
from utils import scale_image, blit_rotate_center


def draw(win, images, player_car):
    for img, pos in images:
        win.blit(img, pos)

    player_car.draw(win)
    pygame.display.update()


if __name__ == "__main__":
    # control variables
    steer_control = 1
    speed_control = 0
    keep_going = 1

    # simulation variables
    run = True
    clock = pygame.time.Clock()
    FPS = 60

    # images to display
    GRASS = scale_image(pygame.image.load("imgs/grass.jpg"), 2.5)
    TRACK_BORDER = scale_image(pygame.image.load("imgs/track-border.png"), 0.9)
    TRACK = scale_image(pygame.image.load("imgs/track.png"), 0.6)
    images = []  # [(GRASS, (0, 0)), (TRACK, (0, 0))] or [(GRASS, (0, 0))]

    # configure pygame display
    WIDTH, HEIGHT = TRACK.get_width(), TRACK.get_height()
    WIN = pygame.display.set_mode((WIDTH, HEIGHT))
    WIN.fill(0)
    pygame.display.set_caption("Racing Game!")

    # generate path and get proper speeds for each point
    spl_dots, spl_array, tck, u = gen_path(WIN, WIDTH, True)
    trgt_speeds = get_speeds(tck)

    # Get initisl position
    init_x, init_y = get_top_point(spl_array, standard=0)

    # Initialize car
    player_car = PlayerCar(1, (init_x, init_y))

    while run:
        clock.tick(FPS)
        draw(WIN, images, player_car)

        keys = pygame.key.get_pressed()

        closest_dot, coords, closest_index = get_closest_point(
            (player_car.x, player_car.y, player_car.angle),
            spl_array,
            WIN
        )

        # spl_dots = spl_dots + [closest_dot]

        for dot in spl_dots:
            dot.update()

        player_car.pid(WIN, spl_array, keys, trgt_speeds, steer_control, speed_control, keep_going)
        player_car.move()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                break

    pygame.quit()
