from random import randint
from a_star import AStar, MinTurns, ShortestRoute


def make_maze_line(size):
    items = ['.', '.', '.', 'x']
    maze_line = [items[randint(0, 3)] for _ in range(size)]
    return ''.join(maze_line)


def make_maze(size):
    return [make_maze_line(size) for _ in range(size)]


maze = ['...x.....',
        '.x.xx..x.',
        '.x...x.x.',
        '.xxx.x.x.',
        '...x...x.',
        '.x.x.x.x.',
        '.x.....x.',
        '..xxxxx..',
        '.........']
maze = make_maze(60)


for line in maze:
    print(line)
start, stop = (3, 2), (42, 54)


def main():
    route_1 = MinTurns(start, stop, maze).search()
    route_2 = ShortestRoute(start, stop, maze).search()
    print(route_1)
    print(route_2)


if __name__ == "__main__":
    main()