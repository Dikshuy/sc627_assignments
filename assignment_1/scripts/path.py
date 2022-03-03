def read(filename):
    file = open(filename, "r")
    lines = file.readlines()
    start_x, start_y = lines[0].split(',')
    start = [float(start_x), float(start_y)]
    goal_x, goal_y = lines[1].split(',')
    goal = [float(goal_x), float(goal_y)]
    step_size = float(lines[2])
    line = lines[3:]
    obstaclesList = []
    tmp = []
    for i, obs in enumerate(line):
        if obs == "\n":
            tmp = []
            obstaclesList.append(tmp)
            continue
        obs = obs.split('\n')
        for j in obs:
            if not j == "":
                x,y = j.split(',')
                tmp.append((float(x), float(y)))

    return start, goal, step_size, obstaclesList 

def write(path, algo):
    filename = "output_"+algo+".txt"
    file = open(filename, "w")
    for i in path:
        file.write(str(i[0])+", "+str(i[1])+"\n")
    file.close()

def write(path, algo):
    filename = "output_"+algo+".txt"
    file = open(filename, "w")
    for i in path:
        file.write(str(i[0])+", "+str(i[1])+"\n")
    file.close()

