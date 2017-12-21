import numpy as np
import random

dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
               'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
               'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
               'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}

dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0],
            'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0]}

dir_rotate = {'u': [0, 90, 180, -90], 'r': [-90, 0, 90, 180], 'd': [180, -90, 0, 90], 'l': [90, 180, -90, 0]}         

dir_head = ['u', 'r', 'd', 'l']   

dir_point =  {'u': '^', 'r': '>', 'd': 'v', 'l': '<',
            'up': '^', 'right': '>', 'down': 'v', 'left': '<'}    

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim
        self.goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]
        self.heuristics =  [[min(min(abs(i+1-self.maze_dim/2 - 1) + abs(j+1-self.maze_dim/2 - 1), abs(i+1-self.maze_dim/2) + abs(j+1-self.maze_dim/2)),  min(abs(i+1-self.maze_dim/2 - 1) + abs(j+1-self.maze_dim/2), abs(i+1-self.maze_dim/2) + abs(j+1-self.maze_dim/2 - 1))) for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        self.visited =  [[0 for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        self.neighbour_wall = [[[-1, -1, -1, -1] for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        self.neighbour_wall[0][0] = [0, 1, 1, 1] 
        self.policy = [[' ' for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        self.trial_run = True
        self.goal_found = False
        self.best_path = []
        self.path_index = -1
        self.path_len = 0
        self.explored = 0
        self.num_steps = 0
    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''

        
        # During first run
        if(self.trial_run == True):
            rotation = 0
            movement = 0
            
            self.num_steps += 1

            # if new cell is visited increment number of explored cells and mark the cell as visited
            if(self.visited[self.location[0]][self.location[1]] == 0):
                self.explored += 1   
            self.visited[self.location[0]][self.location[1]] = 1

            # check if there are walls around the cell and remember them 
            self.check_walls(sensors)
            
            # if goal is found set 'self.goal_found' to true
            if(self.location[0] in self.goal_bounds and self.location[1] in self.goal_bounds):
                self.goal_found = True

            # calculate percentage of maze explored
            per_covered = self.explored*1.0/(self.maze_dim*self.maze_dim)*100.0        
            
            # end the first run when goal is found
            if(self.goal_found == True):
                path = self.bfs_path()                              # find path to goal using breadth first search 
                self.path_len = len(path)                           # path length
                self.best_path = self.find_policy(path)             # minimize number of moves to traverse the path 
                print "AREA OF MAZE EXPLORED: ", per_covered, " %"        
                self.trial_run = False
                self.heading = 'u'
                self.location = [0, 0]
                return ('Reset', 'Reset')    
            
            found = False           # initialise flag to detect if next move has been found
            pos_ind = [0, 1, 2]
            random.shuffle(pos_ind)  # shuffle the order of sensing walls(random motion)
            for i in pos_ind:
                if(sensors[i] > 0):
                    next_dir = dir_sensors[self.heading][i];
                    next_pos_x = self.location[0] + dir_move[next_dir][0]
                    next_pos_y = self.location[1] + dir_move[next_dir][1]
                    self.heading = next_dir
                    self.location[0] = next_pos_x
                    self.location[1] = next_pos_y
                    movement = 1
                    if(i == 0):
                        rotation = -90
                    elif(i == 1):
                        rotation = 0
                    elif(i == 2):
                        rotation = 90
                    found = True     
                    break
            
            # if dead-end is reached, randomly rotate by 90 or -90 with 0 movement  
            if(found == False):
                i = random.choice([0, 2])
                self.heading = dir_sensors[self.heading][i]
                if(i == 0):
                    rotation = -90
                elif(i == 2):
                    rotation = 90
                movement = 0        

            return rotation, movement   

        # During second run
        elif(self.trial_run == False):
            

            self.path_index = self.path_index + 1           # increment index of list to get the next rotation and movement
            
            # find the next direction the robot is heading
            if(self.best_path[self.path_index][0] == -90):
                self.heading = dir_sensors[self.heading][0]

            elif(self.best_path[self.path_index][0] == 0):
                self.heading = dir_sensors[self.heading][1]

            else:
                self.heading = dir_sensors[self.heading][2]
            
            # 'arrow' symbol for direction in which robot is heading
            point = dir_point[self.heading]

            # value of movement
            i = self.best_path[self.path_index][1]
            
            # add heading symbols('arrow' symbols) i number of times in the maze
            while i>0:
                self.policy[self.location[0]][self.location[1]] = point
                self.location[0] += dir_move[self.heading][0]
                self.location[1] += dir_move[self.heading][1]
                i -= 1

            if(self.path_index == 0):    
                print "MOVES: "
            print(self.heading, self.best_path[self.path_index][1])          # print the moves made to reach the goal     

            if(self.location[0] in self.goal_bounds and self.location[1] in self.goal_bounds):
                print "Number of moves: ", len(self.best_path)   
                print "PATH:"
                self.policy = zip(*self.policy)
                self.policy = self.policy[::-1]
                for row in self.policy:                                     
                        print(row)                                         # visual representation of the policy using 'arrow' symbols
                print "Path length: ", self.path_len       

            return self.best_path[self.path_index][0], self.best_path[self.path_index][1]

    # function to check if there are walls surrounding the cell and to remember them 
    def check_walls(self, sensors):
        for i in range(3):
            if(sensors[i] == 0):
                wall = dir_sensors[self.heading][i]
                if(wall == 'u' or wall == 'up'):
                    self.neighbour_wall[self.location[0]][self.location[1]][0] = 1
                elif(wall == 'r' or wall == 'right'):
                    self.neighbour_wall[self.location[0]][self.location[1]][1] = 1
                elif(wall == 'd' or wall == 'down'):
                    self.neighbour_wall[self.location[0]][self.location[1]][2] = 1
                elif(wall == 'l' or wall == 'left'):
                    self.neighbour_wall[self.location[0]][self.location[1]][3] = 1    
        for i in range(4):
            if(self.neighbour_wall[self.location[0]][self.location[1]][i] == -1):
                self.neighbour_wall[self.location[0]][self.location[1]][i] = 0                  
    
    # function to obtain a path using breadth first search
    def bfs_path(self):

        vis = [[0 for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        parent = [[[] for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        rotate = [[-1 for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        head = [['' for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        q = []
        q.append([0, 0])
        head[0][0] = 'u'
        while(len(q) > 0):
            loc = q[0]
            q.pop(0)
            vis[loc[0]][loc[1]] = 1
            
            if(loc[0] in self.goal_bounds and loc[1] in self.goal_bounds):       # if goal is reached
                goal = loc
                break
            
            for i in range(4):
                if self.neighbour_wall[loc[0]][loc[1]][i] == 0:
                    xn = loc[0] + dir_move[dir_head[i]][0]
                    yn = loc[1] + dir_move[dir_head[i]][1]
                    if vis[xn][yn] == 0 and self.visited[xn][yn] == 1:
                        parent[xn][yn] = loc
                        rotate[xn][yn] = dir_rotate[head[loc[0]][loc[1]]][i]
                        head[xn][yn] = dir_head[i]
                        q.append([xn, yn])     
                           
        path = []
        cell = goal
        while(cell[0] != 0 or cell[1] != 0):
            path.append([rotate[cell[0]][cell[1]], 1])
            cell = parent[cell[0]][cell[1]]
        path.reverse()
        
        return path    
    
    # function to find policy by minimizing number of moves needed to traverse the path
    def find_policy(self, path):
        edited_path = []
        j = 0
        for i in range(len(path)):
            if(j > 0):
                j = j - 1
                continue
            if((i+2)<len(path) and path[i+1] == [0, 1] and path[i+2] == [0, 1]):
                edited_path.append([path[i][0], 3])
                j = 2
            elif((i+1)<len(path) and path[i+1] == [0, 1]):
                edited_path.append([path[i][0], 2])
                j = 1
            else:
                edited_path.append(path[i])            

        return edited_path   
