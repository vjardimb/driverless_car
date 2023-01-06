from utils import blit_rotate_center
import pygame
import time
import math
from utils import scale_image, blit_rotate_center

RED_CAR = scale_image(pygame.image.load("imgs/red-car.png"), 0.55)
GREEN_CAR = scale_image(pygame.image.load("imgs/green-car.png"), 0.55)

class AbstractCar:
	def __init__(self, max_vel, rotation_vel):
		self.img = self.IMG
		self.max_vel = max_vel
		self.min_vel = 1
		self.vel = 0
		self.rotation_vel = rotation_vel
		self.angle = 0
		self.x, self.y = self.START_POS
		self.acceleration = 0.1
		self.old_error = 0
		self.errors_list = [0]*100

	def rotate(self, error, left=False, right=False, control=True):
		# kp = 0.07
		# kd = 0.7
		kp = 0.09
		kd = 1.5
		ki = 4E-4

		turn = kp*min(error, 15) + kd*(error - self.old_error) + ki*sum(self.errors_list) if control else 2
		# print(turn)
		if left:
			self.angle += min(turn, 3)
		elif right:
			self.angle -= min(turn, 3)

		self.old_error = error
		self.errors_list.append(error)
		self.errors_list.pop(0)

	def draw(self, win):
		blit_rotate_center(win, self.img, (self.x, self.y), self.angle)

	def move_forward(self, error, control=True):
		kp = 2
		print(error)

		if control:
			acceleration = kp*error
		else:
			acceleration = self.acceleration

		self.vel = max(min(self.vel + acceleration, self.max_vel), self.min_vel)

		self.move()

	def move(self):
		radians = math.radians(self.angle)
		vertical = math.cos(radians) * self.vel
		horizontal = math.sin(radians) * self.vel

		self.y -= vertical
		self.x -= horizontal

	def reduce_speed(self):
		self.vel = max(self.vel - self.acceleration / 2, 0)
		self.move()


class PlayerCar(AbstractCar):
    IMG = RED_CAR
    START_POS = (180, 200)