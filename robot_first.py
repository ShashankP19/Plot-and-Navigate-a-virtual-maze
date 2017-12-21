import numpy as np
import random

# global dictionaries for robot movement and sensing
dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
               'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
               'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
               'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}
dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0],
            'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0]}
dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
               'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}

dir_point =  {'u': '^', 'r': '>', 'd': 'v', 'l': '<',
            'up': '^', 'right': '>', 'down': 'v', 'left': '<'}              

neighbour = [[0,1], [1,0], [0, -1], [-1, 0]]


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
        self.visited =  [[0 for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        self.policy = [[' ' for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        self.best_path = []
        self.new_path = []
        self.distance_value =  [[self.maze_dim*self.maze_dim+1 for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        self.neighbour_wall = [[[0, 0, 0, 0] for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        self.neighbour_wall[0][0] = [0, 1, 1, 1] 
        self.trial_run = True
        self.path_index = -1
        self.path_found = False
        self.init = 'start'
        self.min_moves = self.maze_dim*self.maze_dim+1
        self.path_len = self.maze_dim*self.maze_dim+1
        self.explored = 0
        self.num_steps = 0
        self.found_goal = False
        self.num_path = 0

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
            
            # if a new cell is visited increment number of explored cells and mark the cell as visited
            if(self.visited[self.location[0]][self.location[1]] == 0):
                self.explored += 1
            self.visited[self.location[0]][self.location[1]] = 1

            # find the walls that are surrounding the cell and remember them
            self.check_walls(sensors)

            # update 'distance value' of each cell
            self.update_distance()

            # if the goal is found from the start location (i.e, robot had started from the start location and has reached the goal)
            if(self.init == 'start' and self.location[0] in self.goal_bounds and self.location[1] in self.goal_bounds):
                self.found_goal = True
                new_path = self.new_path
                self.find_best_policy(new_path)          # check if the new policy is better than the previous policy and update it if required
                  
                self.num_path += 1     
                self.new_path = []       
                self.init = 'goal'                       # make 'goal' as the starting point         
       
            # if start location is found from the goal (i.e, robot had started from the goal and has reached the start location)      
            elif(self.init == 'goal' and self.location == [0,0]):
                self.found_goal = True
                new_path = self.rev_path()               # reverse the path
                self.find_best_policy(new_path)          # check if the new policy is better than the previous policy and update it if required
                
                self.num_path += 1
                self.new_path = []           
                self.init = 'start'                      # make 'start' location as the starting point

            per_covered = self.explored*1.0/(self.maze_dim*self.maze_dim)*100.0                   # calculate percentage of maze explored

            # when its time to end the first run     
            if((self.num_path >= 5 or per_covered >= 70) and self.found_goal == True):
                print "AREA OF MAZE EXPLORED: ", per_covered, " %"        
                self.trial_run = False
                self.heading = 'u'
                self.location = [0, 0]
                return ('Reset', 'Reset')    
            
            # when robot reaches start position, turn 90 degrees clockwise with 0 movement
            if((self.location[0] == 0) and (self.location[1] == 0) and not(self.heading == 'u' or self.heading == 'up')):
                self.heading = dir_sensors[self.heading][2]
                return 90, 0

            found = False              # initialise the flag which detects if next move has been found
            
            # find next unvisited cell with lower or equal 'distance value' to move to
            pos_ind = [0, 1, 2]
            random.shuffle(pos_ind)    # shuffle the order of sensing
            for i in pos_ind:
                if(sensors[i] > 0):
                    next_dir = dir_sensors[self.heading][i];
                    next_pos_x = self.location[0] + dir_move[next_dir][0]
                    next_pos_y = self.location[1] + dir_move[next_dir][1]
                    if((self.distance_value[next_pos_x][next_pos_y] <= self.distance_value[self.location[0]][self.location[1]]) and ~self.visited[self.location[0]][self.location[1]]):
                        found = True    
                        break

            # if all neighbouring cells have been visited, find any cell with lower or equal 'distance value' 
            if(found == False):
                pos_ind = [0, 1, 2]
                random.shuffle(pos_ind)
                for i in pos_ind:
                    if(sensors[i] > 0):
                        next_dir = dir_sensors[self.heading][i];
                        next_pos_x = self.location[0] + dir_move[next_dir][0]
                        next_pos_y = self.location[1] + dir_move[next_dir][1]
                        if((self.distance_value[next_pos_x][next_pos_y] <= self.distance_value[self.location[0]][self.location[1]])):
                            found = True    
                            break
               
            # if next cell is found, update location, movement and rotation           
            if(found == True):            
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
                        
            # if no neighbouring cell with 'lower distance value' is found, randomly rotate by 90 or -90 without movement             
            if(found == False):
                i = random.choice([0, 2])
                self.heading = dir_sensors[self.heading][i]
                if(i == 0):
                    rotation = -90
                elif(i == 2):
                    rotation = 90
                movement = 0    
                       
            # if the robot is present in goal area, do not append rotation, movement to 'new_path' list           
            if(not(self.init == 'goal' and self.location[0] in self.goal_bounds and self.location[1] in self.goal_bounds)):
                self.new_path.append([rotation, movement])
            elif(self.init == 'start'):    
                self.new_path.append([rotation, movement])
 
            return rotation, movement
        
        # During second run
        elif(self.trial_run == False):

            self.path_index = self.path_index + 1              # increment index of list to get the next rotation and movement
            
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

            print(self.heading, self.best_path[self.path_index][1])                     # print the moves made to reach the goal        

            # if goal is reached
            if(self.location[0] in self.goal_bounds and self.location[1] in self.goal_bounds):
                print "Number of moves: ", len(self.best_path)   
                print "PATH:"
                self.policy = zip(*self.policy)
                self.policy = self.policy[::-1]
                for row in self.policy:                                                 # visual representation of the policy using 'arrow' symbols
                        print(row)
                print "Path length: ", self.path_len       

            return self.best_path[self.path_index][0], self.best_path[self.path_index][1]
        

    # function to update 'distance values' of each grid cell 
    def update_distance(self):

        self.distance_value =  [[self.maze_dim*self.maze_dim+1 for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        
        if(self.init == 'start'):      # if goal is the destination(robot is going from start to goal)
            end = self.goal_bounds    
        elif(self.init == 'goal'):     # if starting position is the destination(robot is going from goal to start)
            end = [0]   
        
        change = True

        # while there is a change in 'distance value' of any cell
        while(change):
            change = False

            for x in range(self.maze_dim):
                for y in range(self.maze_dim):

                    if x in end and y in end:                          # destination cells have 'distance value' of 0
                        if(self.distance_value[x][y] > 0):
                            self.distance_value[x][y] = 0
                            change = True
                   
                    else:
                        for k in range(len(neighbour)):

                            nx = x + neighbour[k][0]
                            ny = y + neighbour[k][1]
                            
                            if(nx >= 0 and nx < self.maze_dim and ny >= 0 and ny < self.maze_dim and not(self.neighbour_wall[x][y][k])):
                            
                                new_value = self.distance_value[nx][ny] + 1
                                if(new_value < self.distance_value[x][y]):                           
                                    change = True                                             # change distance value of that cell
                                    self.distance_value[x][y] = new_value
    
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

    # function to find the optimum policy for the path
    def find_best_policy(self, new_path):
        

        path_head = [[' ' for i in range(self.maze_dim)] for j in range(self.maze_dim)]
        loc = [0, 0]
        heading = 'u'                                                                             
        for i in range(len(new_path)):                                                           # find heading directions of each cell for the new path
            if(new_path[i][0] == -90):
                heading = dir_sensors[heading][0]

            elif(new_path[i][0] == 0):
                heading = dir_sensors[heading][1]

            else:
                heading = dir_sensors[heading][2]
            
            if new_path[i][1]>0:
                path_head[loc[0]][loc[1]] = heading
                loc[0] += dir_move[heading][0]
                loc[1] += dir_move[heading][1]
        
        short_path = []
        loc = [0, 0]
        heading = 'u'
        while(loc[0] not in self.goal_bounds or loc[1] not in self.goal_bounds):                   # remove redundant steps and loops in the path
            if(dir_sensors[heading][0] == path_head[loc[0]][loc[1]]):
                short_path.append([-90, 1])
                heading = dir_sensors[heading][0]
            elif(dir_sensors[heading][1] == path_head[loc[0]][loc[1]]):
                short_path.append([0, 1])
                heading = dir_sensors[heading][1]
            else:
                short_path.append([90, 1])
                heading = dir_sensors[heading][2]
            d = dir_move[path_head[loc[0]][loc[1]]]    
            loc[0] += d[0]
            loc[1] += d[1]

        min_policy = []
        j = 0
        for i in range(len(short_path)):                                                             # find policy with minimum number of moves for the path
            if(j > 0):
                j = j - 1
                continue
            if((i+2)<len(short_path) and short_path[i+1] == [0, 1] and short_path[i+2] == [0, 1]):
                min_policy.append([short_path[i][0], 3])
                j = 2
            elif((i+1)<len(short_path) and short_path[i+1] == [0, 1]):
                min_policy.append([short_path[i][0], 2])
                j = 1
            else:
                min_policy.append(short_path[i]) 

        if(len(min_policy) < self.min_moves):                                                       # check if obtained policy is better than previously obtained best policy
            self.best_path = min_policy
            self.min_moves = len(min_policy)
            self.path_len = len(short_path)                   

        return min_policy        

    # function to reverse policy for new path
    def rev_path(self):
        rev_neg_path = []
        for i in range(len(self.new_path)):
            rev_neg_path.append([-1*self.new_path[i][0], self.new_path[i][1]])
        rev_neg_path.append(rev_neg_path[-1])   
        rev_neg_path.reverse()
        for i in range(len(rev_neg_path)):
            if rev_neg_path[i][1] == 0:
                temp = rev_neg_path[i-1]
                rev_neg_path[i-1] = rev_neg_path[i]
                rev_neg_path[i] = temp    

        return rev_neg_path   