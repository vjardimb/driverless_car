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
		self.front_vel = 0
		self.rotation_vel = 0
		self.angle = 0
		self.steer_angle = 0
		self.L = 42
		self.x, self.y = self.START_POS
		self.acceleration = 0.1
		self.old_error = 0
		self.errors_list = [0]*100

	def steer(self, error, left=False, right=False, control=True):
		# kp = 0.07
		# kd = 0.7
		kp = 0.09
		kd = 1.5
		ki = 4E-4

		steer_angle_rate = kp*min(error, 15) + kd*(error - self.old_error) + ki*sum(self.errors_list) if control else 3

		if left:
			if self.steer_angle < 30:
				self.steer_angle += min(steer_angle_rate, 30)
		elif right:
			if self.steer_angle > -30:
				self.steer_angle -= min(steer_angle_rate, 30)

		self.old_error = error
		self.errors_list.append(error)
		self.errors_list.pop(0)

	def draw(self, win):
		blit_rotate_center(win, self.img, (self.x, self.y), self.angle)

	def move_forward(self, error, control=True):
		kp = 2

		if control:
			acceleration = kp*error
		else:
			acceleration = self.acceleration

		self.front_vel = max(min(self.front_vel + acceleration, self.max_vel), self.min_vel)

	def move(self):
		steer_ang = math.radians(self.steer_angle)
		slip_ang = math.atan(math.tan(steer_ang)/2)
		# rad_slip_ang = math.radians(slip_ang)
		S = self.L / math.tan(steer_ang) if steer_ang != 0 else 10E6
		R1 = S / math.cos(slip_ang)
		R2 = S / math.cos(steer_ang)

		self.rotation_vel = self.front_vel / R2
		self.vel = self.rotation_vel * R1

		print("R2: ", R2, "      rotation_vel: ", self.rotation_vel)

		orientation = math.radians(self.angle)
		vert_speed = math.cos(orientation + steer_ang) * self.vel
		horiz_speed = math.sin(orientation + steer_ang) * self.vel

		self.y -= vert_speed
		self.x -= horiz_speed
		self.angle += self.rotation_vel

	def reduce_speed(self):
		self.front_vel = max(self.front_vel - self.acceleration / 2, 0)
		self.move()


class PlayerCar(AbstractCar):
	IMG = RED_CAR
	START_POS = (180, 200)
