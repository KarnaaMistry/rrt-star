#Karnaa Mistry CS560 file_parse.py


def parse_problem(world_file, problem_file):
    robot = []
    obstacles = []
    problems = []
    with open(world_file) as file:
        lines = file.readlines()
    for i in range(len(lines)):
        line = lines[i]
        line = line.split('(')
        if i == 0: #robot
            for s in line:
                s = s.strip("(), \n")
                if s == '':
                    continue
                coord = s.split(',')
                robot.append((float(coord[0]), float(coord[1])))
        else: #not robot
            obstacle = []
            for s in line:
                s = s.strip("(), \n")
                if s == '':
                    continue
                coord = s.split(',')
                obstacle.append((float(coord[0]), float(coord[1])))   
            obstacles.append(obstacle)
        
    with open(problem_file) as pfile:
        lines = pfile.readlines()
    for line in lines:
        coord = line.split(' ')
        problems.append([(float(coord[0]), float(coord[1])), (float(coord[2]), float(coord[3]))])
            
    return (robot, obstacles, problems)


