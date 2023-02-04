import pygame
import math
import numpy as np
from scipy.spatial import ConvexHull
from scipy import interpolate
import matplotlib.pyplot as plt


class Dot:
    def __init__(self, screen, color, pos, size=8):
        self.screen = screen
        self.color = color
        self.pos = pos
        self.size = size

    def draw(self):
        pygame.draw.circle(self.screen, self.color, self.pos, self.size)


def gen_path(screen, dim, random=True):
    if random:
        # generate 20 random points
        points = np.random.randint(dim*0.2, dim*0.8, size=(20, 2))
    else:
        # in case we don't want a random track
        points = np.array([[194, 250],
           [227, 174],
           [429, 402],
           [347, 407],
           [162, 350],
           [250, 158],
           [375, 424],
           [378, 370],
           [381, 361],
           [358, 185],
           [314, 429],
           [305, 281],
           [223, 307],
           [299, 255],
           [273, 166],
           [187, 412],
           [181, 261],
           [383, 197],
           [431, 366],
           [219, 408]])

    hull = ConvexHull(points)

    x_vertices = np.r_[points[hull.vertices, 0], points[hull.vertices[0], 0]]
    y_vertices = np.r_[points[hull.vertices, 1], points[hull.vertices[0], 1]]

    spl_dots = []

    # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
    # is needed in order to force the spline fit to pass through all the input points.
    tck, u = interpolate.splprep([x_vertices, y_vertices], s=0, per=True)
    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, 1000), tck)

    for [x, y] in zip(xi, yi):
        spl_dots.append(Dot(screen, "yellow", (x, y), 3))

    return spl_dots, np.concatenate((xi.reshape((xi.shape[0], 1)), yi.reshape((yi.shape[0], 1))), axis=1), tck


def get_start_point(spl_array, standard=False):
    highest_index = np.argmin(spl_array[:, 0])

    if standard:
        x, y = 180, 200
    else:
        x, y = spl_array[highest_index, 0], spl_array[highest_index, 1]

    return x, y


def get_closest_point(pose, spl_array):
    position = np.array([pose[0], pose[1]]).reshape((1, 2))
    position = np.repeat(position, len(spl_array), axis=0)

    dists_sqrd = np.sum((position - spl_array)**2, axis=1)
    closest_index = np.argmin(dists_sqrd)

    return (spl_array[closest_index, 0], spl_array[closest_index, 1]), closest_index


def get_speeds(tck):
    m = 0.03

    xi_der, yi_der = interpolate.splev(np.linspace(0, 1, 1000), tck, der=2)

    derivatives = np.concatenate((xi_der.reshape((xi_der.shape[0], 1)), yi_der.reshape((yi_der.shape[0], 1))), axis=1)

    lat_accel = np.sum((derivatives)**2, axis=1)

    norm_accs = lat_accel/max(lat_accel)

    return m/norm_accs + 0.5


if __name__ == "__main__":
    # win = pygame.display.set_mode((810, 810))
    #
    # spl_dots, spl_array = gen_path(win, 810)
    #
    # get_closest_point((30, 50, 100), spl_array, win)
    pass