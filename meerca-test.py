from PIL import ImageGrab, Image
import numpy as np
import cv2
import time 
import pyautogui
import matplotlib.pyplot as plt
from ahk import AHK

ahk = AHK(executable_path='C:\\Program Files\\AutoHotkey\\AutoHotkey.exe')

# Full Screen
X_START = 460
Y_START = 140
X_STOP = 1060
Y_STOP = 840
COMPRESSION_SIZE = 100
SPREAD = 6

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
    
    def __hash__(self):            
        return hash(self.position)

def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = set()

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
        closed_list.add(current_node)

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
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

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
            if child in closed_list:
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
LAST_COMMAND = 0
def getDirection(meer, goal):
    global LAST_COMMAND
    if (meer[1] - goal[1] < 0 and LAST_COMMAND != 2):
        LAST_COMMAND = 2
        return 2
    elif (meer[1] - goal[1] > 0 and LAST_COMMAND != 4):
        LAST_COMMAND = 4
        return 4
    
    if (meer[0] - goal[0] > 0 and LAST_COMMAND != 1):
        LAST_COMMAND = 1
        return 1
    elif (meer[0] - goal[0] < 0 and LAST_COMMAND != 3):
        LAST_COMMAND = 3
        return 3

    if (LAST_COMMAND == 1 or LAST_COMMAND == 3):
        LAST_COMMAND = 2
        return 2
    else:
        LAST_COMMAND = 1
        return 1

def pathFindInImage(im_rgb):
    global SPREAD
    myMap = []

    for row in im_rgb:
        currentRow = []
        for item in row:
            if (item == [153,153,153]).all():
                currentRow.append(0)
            else:
                currentRow.append(1)
        myMap.append(currentRow)

    # FIND THE GOAL
    YELLOW_MIN = np.array([200, 200, 0], np.uint8)
    YELLOW_MAX = np.array([255, 255, 25], np.uint8)

    dst = cv2.inRange(np.array(im_rgb), YELLOW_MIN, YELLOW_MAX)
    locations_of_color = np.transpose(dst.nonzero())
    if (len(locations_of_color) == 0): return "NO GOAL"
    
    GOAL_X, GOAL_Y = locations_of_color[0]

    # FIND THE MEERCA
    MEER_MIN = np.array([170, 130, 50], np.uint8)
    MEER_MAX = np.array([210, 180, 100], np.uint8)

    dst2 = cv2.inRange(np.array(im_rgb), MEER_MIN, MEER_MAX)
    locations_of_meer = np.transpose(dst2.nonzero())
    if (len(locations_of_meer) == 0): return "NO MEER"
    
    MEER_X, MEER_Y = locations_of_meer[0]

    # DIG HOLES SO WE CAN PATHFIND BETWEEN THEM
    for i in range (SPREAD):
        if GOAL_X + i < COMPRESSION_SIZE:
            myMap[GOAL_X + i][GOAL_Y] = 0
        if GOAL_X - i >= 0:
            myMap[GOAL_X - i][GOAL_Y] = 0
        if GOAL_Y + i < COMPRESSION_SIZE:
            myMap[GOAL_X][GOAL_Y + i] = 0
        if GOAL_Y - i >= 0:
            myMap[GOAL_X][GOAL_Y - i] = 0

        if MEER_X + i < COMPRESSION_SIZE:
            myMap[MEER_X + i][MEER_Y] = 0
        if MEER_X - i >= 0:
            myMap[MEER_X - i][MEER_Y] = 0
        if MEER_Y + i < COMPRESSION_SIZE:
            myMap[MEER_X][MEER_Y + i] = 0
        if MEER_Y - i >= 0:
            myMap[MEER_X][MEER_Y - i] = 0

    # PATHFIND
    path = astar(myMap, (MEER_X,MEER_Y), (GOAL_X, GOAL_Y))

    # SHOW THE RESULTS
    for item in path:
        myMap[item[0]][item[1]] = 2
    plt.imsave('imgs/state.png', myMap)

    dir = 0
    if (len(path) > SPREAD * 2):
        dir = getDirection(path[0], path[SPREAD * 2])
    elif (len(path) > SPREAD): 
        dir = getDirection(path[0], path[SPREAD])
    else:
        dir = getDirection(path[0], path[1])

    if (dir == 1): 
        # pyautogui.press("up")
        ahk.key_press('up')
        print("up")
    elif (dir == 2): 
        # pyautogui.press("right")
        ahk.key_press('right')
        print("right")
    elif (dir == 3): 
        # pyautogui.press("down")
        ahk.key_press('down')
        print("down")
    elif (dir == 4): 
        # pyautogui.press('left')
        ahk.key_press('left')
        print("left")
    
    return "OK"
        
def main():

    for i in range(3):
        print(i)
        time.sleep(0.5)

    counter = 0
    while True:
        # Save part of the screen where game is
        im=ImageGrab.grab(bbox=(X_START,Y_START,X_STOP,Y_STOP)).convert('RGB')
        new_im = im.resize((COMPRESSION_SIZE,COMPRESSION_SIZE),Image.ANTIALIAS)

        # new_im.save("imgs/new{}.png".format(counter),optimize=True,quality=95)

        val = pathFindInImage(np.array(new_im))
        if (val != "OK"): 
            print(val)

        # time.sleep(1)

        # counter += 1

if __name__ == '__main__':
    main()