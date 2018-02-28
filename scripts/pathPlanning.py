import cv2
import numpy as np
import heapq
import matplotlib.pyplot as plt
# no of rows and columns in grid
rows = 14 #mat_size_1
columns = 21 #mat_size_2


def grid_map(img):
    # create a 2d array
    grid = np.zeros((rows, columns))
    for i in range(rows):
        for j in range(columns):
            # white blocks
            if (np.array_equal(img[20+(i*40), 20+(j*40)], [255, 255, 255])):
                grid[i][j] = 0
            # start -> orange block
            elif (np.array_equal(img[20+(i*40), 20+(j*40)], [39, 127, 255])):
                grid[i][j] = 2
            # end -> pink block
            elif (np.array_equal(img[20+(i*40), 20+(j*40)], [201, 174, 255])):
                grid[i][j] = 3
            # obstacles ->black blocks
            else:
                grid[i][j] = 1
    return grid


class Cell(object):
    def __init__(self, x, y, reachable):
        # setting some parameters for each cell
        self.reachable = reachable
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0
        self.heuristic = 0
        # net_cost=cost+heuristic
        self.net_cost = 0


class Astar(object):
    def __init__(self):
        # list of unchecked neighbour cells
        self.open = []
        # keeps cells with lowest total_cost at top
        heapq.heapify(self.open)
        # list of already checked cells
        self.closed = set()
        # list of neighbour cells
        self.cells = []

    def init_grid(self, grid):
        for i in range(rows):
            for j in range(columns):
                #route_path.reverse()
                # detecting the obstacles
                if grid[i][j] == 1:
                    reachable = False
                else:
                    reachable = True
                self.cells.append(Cell(i, j, reachable))
                # detecting the start and end
                if(grid[i][j] == 2):
                    self.start = self.cell(i, j)
                if(grid[i][j] == 3):
                    self.end = self.cell(i, j)
	# print "initialised"

    def cell(self, x, y):
        # returns the location to identify each cell

        return self.cells[x*columns+y]

    def cell_heuristic(self, cell):
        # returns the heuristic for astar algo
        return abs(cell.x-self.end.x)+abs(cell.y-self.end.y)

    def neighbour(self, cell):
        cells = []
        # returns a list of neigbours of a cell
        if cell.x < rows - 1:
            cells.append(self.cell(cell.x+1, cell.y))
        if cell.x > 0:
            cells.append(self.cell(cell.x-1, cell.y))
        if cell.y < columns-1:
            cells.append(self.cell(cell.x, cell.y+1))
        if cell.y > 0:
            cells.append(self.cell(cell.x, cell.y-1))
        if cell.x < rows-1 and cell.y < columns-1:
	        cells.append(self.cell(cell.x+1, cell.y+1))
        if cell.x < rows-1 and cell.y > 0:
	        cells.append(self.cell(cell.x+1, cell.y-1))
        if cell.x > 0 and cell.y < columns-1:
            cells.append(self.cell(cell.x-1, cell.y+1))
        if cell.x > 0 and cell.y > 0:
	        cells.append(self.cell(cell.x-1, cell.y-1))


        return cells

    def update_cell(self, adj, cell):
        # update the details about the selected neigbour cell
        adj.cost = cell.cost + 1
        adj.heuristic = self.cell_heuristic(adj)
        adj.parent = cell
        adj.net_cost = adj.cost + adj.heuristic

    def display_path(self):
        # list for storing the path
        route_path = []
        # flag to determine length of path(
        count = 0
        cell = self.end
        while cell.parent is not None:
            # storing the parents in list from end to start
            route_path.append([(cell.x), (cell.y)])
            cell = cell.parent
            count += 1
        return route_path, count

    def search(self):
        # pushing the first element in open queue
        heapq.heappush(self.open, (self.start.net_cost, self.start))
        while(len(self.open)):
            net_cost, cell = heapq.heappop(self.open)
            # adding the checked cell to closed list
            self.closed.add(cell)
            if cell is self.end:
                # store path and path legth
                route_path, route_length = self.display_path()
                route_path.append([self.start.x, self.start.y])
                break
            # getting the adjoint cells
            neighbours = self.neighbour(cell)
            for path in neighbours:
                # if cell is not an obstacle and has not been already checked
                if path.reachable and path not in self.closed:
                    if (path.net_cost, path) in self.open:
                        # selecting the cell with least cost
                        if path.cost > cell.cost + 1:
                            self.update_cell(path, cell)
                    else:
                        self.update_cell(path, cell)
                        heapq.heappush(self.open, (path.net_cost, path))
        route_path.reverse()
        return route_path, route_length


def play(grid):
    # map the grid in an array
    #grid = grid_map(img)
    #grid = [[0,0,0,0,0],[2,0,0,1,0],[0,1,1,0,0],[0,1,1,0,0],[0,0,0,0,3]]
    # executing A*
    solution = Astar()
    solution.init_grid(grid)
    route_path, route_length = solution.search()

    return route_length, route_path

def curve_fit(x,y):

	# get x and y vectors
    #x = points[:,0]
    #y = points[:,1]

	# calculate polynomial
	#z = np.polyfit(x, y, 5)
	#print "z:", z
	#f = np.poly1d(z)

	# calculate new x's and y's
	#x_new = np.linspace(x[0], x[-1], 50)
	#y_new = f(x_new)
    time_x=np.zeros(len(x))
    time_x[0] = (0.5/8)
    for i in range(1, len(x)):
        time_x[i] = time_x[i-1] + (0.5/8) # found this suitable after trial and error

    order = 4 #len(x) - 3
    time_x_new = np.linspace(time_x[0], time_x[-1], 3*len(x))



    y_coeff = np.polyfit(time_x, y, order)
    func_y = np.poly1d(y_coeff)
    func_y_dot = func_y.deriv()
    new_y = func_y(time_x_new)
    new_y_dot = func_y_dot(time_x_new)

    x_coeff = np.polyfit(time_x, x, order)
    func_x = np.poly1d(x_coeff)
    func_x_dot = func_x.deriv()
    new_x = func_x(time_x_new)
    new_x_dot = func_x_dot(time_x_new)

    # plt.figure(1)
    # plt.plot(time_x_new, new_y, 'o')
    # plt.xlabel('time')
    # plt.ylabel('bot y')
    # plt.figure(2)
    # plt.plot(time_x_new, new_x, 'x')
    # plt.xlabel('time')
    # plt.ylabel('bot x')
    # plt.figure(3)
    # plt.plot(time_x_new, new_y_dot, 'o')
    # plt.xlabel('time')
    # plt.ylabel('y_dot')
    # plt.figure(4)
    # plt.plot(time_x_new, new_x_dot, 'x')
    # plt.xlabel('time')
    # plt.ylabel('x_dot')
    # plt.show()

    return time_x_new, new_x, new_y, new_x_dot, new_y_dot

    # print "velocity:", new_x_dot
    # x_dot = func_x.diff
    #plt.plot(time_x_new, new_x, 'o')#, time_x_new, new_x, time_x_new, new_x_dot, 'x')
    # plt.xlim([time_x[0]-1, time_x[-1] + 1 ])
    # plt.ylim([0, 15 ])
    #plt.show()
	# plt.plot(x,y,'o', x_new, y_new)
	# plt.xlim([x[0]-1, x[-1] + 1 ])
	# plt.show()
