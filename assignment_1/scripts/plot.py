from path import *
import matplotlib.pyplot as plt 

filename = "/home/dikshant/catkin_ws/src/sc627_assignments/assignment_1/input.txt"
start, goal, step_size, obstaclesList = read(filename)

for obs in range(len(obstaclesList)):
	plt.gca().add_patch(plt.Polygon(obstaclesList[obs], color = "cyan"))

plt.scatter(start[0], start[1])
plt.scatter(goal[0], goal[1])

filename = "/home/dikshant/catkin_ws/src/sc627_assignments/assignment_1/scripts/output_1.txt"
file = open(filename, "r")
lines = file.readlines()
X, Y = [], []
for i in lines:
	x, y= i.split(',')
	X.append(float(x))
	Y.append(float(y))
plt.plot(X, Y, color = "red", marker = ".", markersize=1)
plt.show()