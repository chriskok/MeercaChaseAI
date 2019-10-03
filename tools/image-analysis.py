from PIL import ImageGrab, Image
import numpy as np
import cv2
import time 
import matplotlib.pyplot as plt

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

# Returns a direction to press the button toward
# 1 = up, 2 = right, 3 = down, 4 = left
def getDirection(meer, goal):
    if (meer[1] - goal[1] < 0):
        return 2
    elif (meer[1] - goal[1] > 0):
        return 4
    
    if (meer[0] - goal[0] > 0):
        return 1
    elif (meer[0] - goal[0] < 0):
        return 3

def main():

    image = cv2.imread('C:\\Users\\Christopher Kok\\Desktop\\MeercaChaseAI\\imgs\\new.png')
    im_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    myMap = []

    for row in im_rgb:
        currentRow = []
        for item in row:
            if (item == [153,153,153]).all():
                # print(0, end="")
                currentRow.append(0)
            else:
                # print(item, end="")
                currentRow.append(1)
        myMap.append(currentRow)
        # print('\n')

    # FIND THE GOAL
    YELLOW_MIN = np.array([200, 200, 0], np.uint8)
    YELLOW_MAX = np.array([255, 255, 50], np.uint8)

    dst = cv2.inRange(np.array(im_rgb), YELLOW_MIN, YELLOW_MAX)
    locations_of_color = np.transpose(dst.nonzero())
    
    GOAL_X, GOAL_Y = locations_of_color[0]

    # FIND THE MEERCA
    MEER_MIN = np.array([200, 150, 0], np.uint8)
    MEER_MAX = np.array([255, 190, 50], np.uint8)

    dst2 = cv2.inRange(np.array(im_rgb), MEER_MIN, MEER_MAX)
    locations_of_meer = np.transpose(dst2.nonzero())
    
    MEER_X, MEER_Y = locations_of_meer[0]

    # DIG HOLES SO WE CAN PATHFIND BETWEEN THEM
    spread = 6
    for i in range (spread):
        if GOAL_X + i < 100:
            myMap[GOAL_X + i][GOAL_Y] = 0
        if GOAL_X - i >= 0:
            myMap[GOAL_X - i][GOAL_Y] = 0
        if GOAL_Y + i < 100:
            myMap[GOAL_X][GOAL_Y + i] = 0
        if GOAL_Y - i >= 0:
            myMap[GOAL_X][GOAL_Y - i] = 0

        if MEER_X + i < 100:
            myMap[MEER_X + i][MEER_Y] = 0
        if MEER_X - i >= 0:
            myMap[MEER_X - i][MEER_Y] = 0
        if MEER_Y + i < 100:
            myMap[MEER_X][MEER_Y + i] = 0
        if MEER_Y - i >= 0:
            myMap[MEER_X][MEER_Y - i] = 0

    # PATHFIND
    path = astar(myMap, (MEER_X,MEER_Y), (GOAL_X, GOAL_Y))
    for item in path:
        myMap[item[0]][item[1]] = 2

    # SHOW THE RESULTS
    # plt.imshow(myMap)
    # plt.show()

    if (len(path) > spread):
        dir = getDirection(path[0], path[spread])
        print(path[0])
        print(path[spread])
        if (dir == 1): print("up")
        if (dir == 2): print("right")
        if (dir == 3): print("down")
        if (dir == 4): print("left")

main()