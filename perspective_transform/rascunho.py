import cv2
import numpy as np
from matplotlib import pyplot as plt

# Capturing video
cap = cv2.VideoCapture("video.mp4")

top_height = 20
top_width = 100
bottom_width = 1280


while True:
	# Get frame
	_, frame = cap.read()

	# Crop top of the image
	frame = frame[100:]

	# Plotting four circles on the video of the object you want to see the transformation of.
	cv2.circle(frame, (640 + top_width, 20), 5, (0, 0, 255), -1)
	cv2.circle(frame, (640 - top_width, 20), 5, (0, 0, 255), -1)
	cv2.circle(frame, (1260, 700), 5, (0, 0, 255), -1)
	cv2.circle(frame, (20, 700), 5, (0, 0, 255), -1)

	# Select all the above four points in an array
	imgPts = np.float32([[640 - top_width, 20], [640 + top_width, 20], [0, 720], [1280, 720]])

	# Select four points in an array for the destination video (the one you want to see as your output)
	objPoints = np.float32([[220, 0], [420, 0], [220, 310], [420, 310]])

	# Apply perspective transformation function of openCV2.
	# This function will return the matrix which you can feed into warpPerspective function to get the warped image.
	matrix = cv2.getPerspectiveTransform(imgPts, objPoints)
	result = cv2.warpPerspective(frame, matrix, (640, 310))

	# put images side by side
	comp = np.concatenate((frame[::2, ::2], result), axis=1)

	# Now Plotting both the videos(original, warped video)using matplotlib
	cv2.imshow('Perspective Transformation', comp)

	key = cv2.waitKey(1)
	# plt.imshow(frame)
	plt.show()
	if key == 27:
		break

cap.release()
cv2.destroyAllWindows()
