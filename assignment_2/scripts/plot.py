from path import *
import os
import matplotlib.pyplot as plt 

def plot():
	filename = os.getcwd()+"/src/sc627_assignments/assignment_2/input.txt"
	start, goal, step_size, obstaclesList = read(filename)

	# plotting start, goal and obstacles
	for obs in range(len(obstaclesList)):
		plt.gca().add_patch(plt.Polygon(obstaclesList[obs], color = "cyan", fill = False))

	plt.scatter(start[0], start[1])
	plt.scatter(goal[0], goal[1])

	# plotting the traced path
	filename = os.getcwd()+"/src/sc627_assignments/assignment_2/output.txt"
	file = open(filename, "r")
	lines = file.readlines()
	X, Y = [], []
	for i in lines:
		x, y= i.split(',')
		X.append(float(x))
		Y.append(float(y))
	plt.plot(X, Y, color = "red", marker = ".", markersize=1)

	plt.show()

if __name__ == "__main__":  
	plot()
	