import pygame
import time
import math

from car import PlayerCar
from path import gen_path, get_closest_point, right_or_left, turn, get_speeds, accelerate
from utils import scale_image, blit_rotate_center


def draw(win, images, player_car):
    for img, pos in images:
        win.blit(img, pos)

    player_car.draw(win)
    pygame.display.update()


if __name__ == "__main__":
    steer_control = 1
    speed_control = 0
    keep_going = 1

    run = True
    clock = pygame.time.Clock()

    GRASS = scale_image(pygame.image.load("imgs/grass.jpg"), 2.5)
    TRACK_BORDER = scale_image(pygame.image.load("imgs/track-border.png"), 0.9)
    TRACK = scale_image(pygame.image.load("imgs/track.png"), 0.6)

    # images = [(GRASS, (0, 0)), (TRACK, (0, 0))]
    # images = [(TRACK, (0, 0))]
    images = []
    player_car = PlayerCar(1, 4)

    WIDTH, HEIGHT = TRACK.get_width(), TRACK.get_height()

    WIN = pygame.display.set_mode((WIDTH, HEIGHT))

    pygame.display.set_caption("Racing Game!")

    FPS = 60

    spl_dots, spl_array, tck, u = gen_path(WIN, WIDTH, False)

    trgt_speeds = get_speeds(tck)

    WIN.fill(0)

    while run:
        clock.tick(FPS)

        draw(WIN, images, player_car)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                break

        closest_dot, coords, closest_index = get_closest_point(
            (player_car.x, player_car.y, player_car.angle),
            spl_array,
            WIN
        )

        # for dot in spl_dots+[closest_dot]:
        for dot in spl_dots:
            # for dot in spl_dots:
            dot.update()
            # the_dot.update()

        side, pos_error = right_or_left((player_car.x, player_car.y, player_car.angle), coords)

        keys = pygame.key.get_pressed()

        turn(player_car, side, pos_error, keys, control=steer_control)
        accelerate(player_car, trgt_speeds, closest_index, keys, control=speed_control, keep_going=keep_going)
        player_car.move()

        # print("ccords", coords)

    pygame.quit()
