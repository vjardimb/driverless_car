from utils import blit_rotate_center
import pygame
import time
import numpy as np
import math
from utils import scale_image, blit_rotate_center
from path import Dot

RED_CAR = scale_image(pygame.image.load("imgs/red-car.png"), 0.55)
GREEN_CAR = scale_image(pygame.image.load("imgs/green-car.png"), 0.55)


class AbstractCar:
	def __init__(self, max_vel, start_pos):
		self.img = self.IMG
		self.max_vel = max_vel
		self.min_vel = 1
		self.vel = 0
		self.front_vel = 0
		self.rotation_vel = 0
		self.angle = 0
		self.steer_angle = 0
		self.max_steer_angle = 30
		self.L = 10
		self.x, self.y = start_pos
		self.acceleration = 0.1
		self.old_error = 0
		self.errors_list = [0] * 100

	def steer(self, error, left=False, right=False, control=True):
		# kp = 0.07
		# kd = 0.7
		kp = 1
		# kd = 1.5
		kd = 10
		# ki = 4E-4
		ki = 0

		steer_angle_rate = kp * min(error, 10000000) + kd * (error - self.old_error) + ki * sum(
			self.errors_list) if control else 3

		if left:
			if self.steer_angle < self.max_steer_angle:
				self.steer_angle += min(steer_angle_rate, 30)
		elif right:
			if self.steer_angle > - self.max_steer_angle:
				self.steer_angle -= min(steer_angle_rate, 30)

		self.old_error = error
		self.errors_list.append(error)
		self.errors_list.pop(0)

	def draw(self, win):
		blit_rotate_center(win, self.img, (self.x, self.y), self.angle)

	def move_forward(self, error, control=True):
		kp = 2

		if control:
			acceleration = kp * error
		else:
			acceleration = self.acceleration

		self.front_vel = max(min(self.front_vel + acceleration, self.max_vel), self.min_vel)

	def turn(self, side, error, keys, control):
		if keys[pygame.K_a]:
			self.steer(40, left=True, control=False)
		elif keys[pygame.K_d]:
			self.steer(40, right=True, control=False)
		elif control:
			if side == "left":
				self.steer(error, left=True)
			else:
				self.steer(error, right=True)

	def accelerate(car, trgt_speeds, closest_index, keys, control=True, keep_going=False):
		if keys[pygame.K_w] or keep_going:
			car.move_forward(0, control=False)
		elif control:
			# error = trgt_speeds[closest_index] - car.vel
			start = (closest_index - 10) % len(trgt_speeds)
			end = start + 50
			error = np.mean(trgt_speeds[closest_index]) - car.vel
			# error = np.mean(trgt_speeds[start:end]) - car.vel
			car.move_forward(error)
		else:
			car.reduce_speed()

	def bike_kinematics(self):
		steer_ang = math.radians(self.steer_angle)
		slip_ang = math.atan(math.tan(steer_ang) / 2)
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

		return orientation, vert_speed, horiz_speed

	def move(self):
		orientation, vert_speed, horiz_speed = self.bike_kinematics()

		self.y -= vert_speed
		self.x -= horiz_speed
		self.angle += math.degrees(self.rotation_vel)

	def reduce_speed(self):
		self.front_vel = max(self.front_vel - self.acceleration / 2, 0)

	def right_or_left(self, closest_point):
		angle = math.radians(self.angle)

		c_x = closest_point[0]
		c_y = closest_point[1]

		rotation_matrix = [[math.cos(angle), -math.sin(angle), 0],
						   [math.sin(angle), math.cos(angle), 0],
						   [0, 0, 1]]

		transl_vector = [[self.x],
						 [self.y],
						 [0]]

		position = [[c_x],
					[c_y],
					[0]]

		new_closest = np.dot(rotation_matrix, np.array(position) - np.array(transl_vector))

		return "right" if new_closest[0] > 0 else "left", abs(new_closest[0])

	def get_closest_point(self, spl_array, screen):
		position = np.array([self.x, self.y]).reshape((1, 2))
		position = np.repeat(position, len(spl_array), axis=0)

		dists_sqrd = np.sum((position - spl_array) ** 2, axis=1)
		closest_index = np.argmin(dists_sqrd)

		dot = Dot(screen, "red", (spl_array[closest_index, 0], spl_array[closest_index, 1]), 8)
		closest_coords = (spl_array[closest_index, 0], spl_array[closest_index, 1])
		return dot, closest_coords, closest_index

	def pid(self, WIN, spl_array, keys, trgt_speeds, steer_control, speed_control, keep_going):
		# get reference
		closest_dot, closest_point, closest_index = self.get_closest_point(spl_array, WIN)

		# get error
		side, pos_error = self.right_or_left(closest_point)

		# apply command
		self.turn(side, pos_error, keys, control=steer_control)
		self.accelerate(trgt_speeds, closest_index, keys, control=speed_control, keep_going=keep_going)


class PlayerCar(AbstractCar):
	IMG = RED_CAR
	# START_POS = (180, 200)
