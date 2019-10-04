from heapq import heappop, heappush


class AStar(object):

    def __init__(self, start_pos: tuple, end_pos: tuple, grid: list):
        """Instantiate AStar object.

        start_pos and end_pos are tuples of coordinates.
        (0, 2), (0, 4)

        grid is a list of length n of strings of length n where 'x' indicates a barrier.

        i.e.
        ['...x...',
        '.xxxx..',
        '.x.....',
        '.x.xxx.',
        '.x...x.',
        '..xx.x.',
        '.......']

        :param start_pos: Coordinates of start position.
        :param end_pos: Coordinates of end position.
        :param grid: List containing square grid of strings with 'x' denoting barriers.
        """
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.grid = grid


class MinTurns(AStar):

    """A star algorithm for navigating a map in the fewest turns.

    Given start and end points for a grid style maze, outputs a tuple containing; a list of coordinates, number of
    turns, and length of route from start to end for the route with the fewest turns.

    """

    def find_neighbors(self, curr_pos: tuple, prev_pos: tuple) -> list:
        """Finds neighbors of curr_pos.

        Adds coordinates of neighbors to list along with boolean(is_turn) representing whether a turn occurs to reach
        the neighbor given path from prev_pos to curr_pos.

        :param curr_pos: curr_pos of self.search().
        :param prev_pos: position prior to curr_pos.
        :return: list of tuples [((pos of neighbor), is_turn)].
        """
        limit = len(self.grid) - 1
        neighbors = []
        # Check each position adjacent to curr_pos.
        for y, x in zip([0, -1, 0, 1], [1, 0, -1, 0]):
            # If adjacent position is outside of grid, skip it.
            if (curr_pos[0] + y) > limit or (curr_pos[0] + y) < 0 or (curr_pos[1] + x) > limit or (curr_pos[1] + x) < 0:
                continue
            # If adjacent position is a barrier, skip it.
            if self.grid[curr_pos[0] + y][curr_pos[1] + x] == 'x':
                continue
            else:
                # Ensure we do not count a first step as a turn.
                if curr_pos == self.start_pos:
                    is_turn = False
                else:
                    # Determine direction from prev_pos to curr_pos.
                    if curr_pos != prev_pos[1] and curr_pos[0] == prev_pos[1][0]:
                        direction = 'horizontal'
                    else:
                        direction = 'vertical'

                    # Determine movement from curr_pos to new position.
                    if y != 0:
                        movement = 'vertical'
                    else:
                        movement = 'horizontal'

                    # If we're not still moving in the same direction, we have turned.
                    is_turn = not (movement == direction)
                neighbors.append(((curr_pos[0] + y, curr_pos[1] + x), is_turn))
        return neighbors

    def make_path(self, curr_pos: tuple, route: dict) -> tuple:
        """Creates list of coordinates representing turns in path.

        :param curr_pos: end point of path.
        :param route: dict of coordinates leading to curr_pos
        :return: tuple containing list of coordinates of turns, len(list of coordinates)
        """
        turn_coords = []
        x, path_length = 0, 1
        prev_pos = curr_pos

        if route.get(self.end_pos) is None:
            return 'No path found.', []

        while route[curr_pos] is not None:
            curr_pos = route[curr_pos][1]

            if curr_pos[x] != prev_pos[x]:
                if x == 0:
                    x = 1
                else:
                    x = 0

                if prev_pos is not self.end_pos:
                    turn_coords.append(prev_pos)

            prev_pos = curr_pos
            path_length += 1

        turn_coords.reverse()
        return turn_coords, path_length

    def search(self):
        """Finds path through self.grid in fewest number of turns.

        Uses a priority queue to sort nodes by least number of turns required to reach it.
        Continually updates number of turns needed to reach any given position if a better path is found.

        :return: self.make_path().
        """
        # Keep track of where we've been.
        visited = set()

        # We'll keep track of the route and the number of turns to reach the curr_pos with a dict.
        # {(position): (turns_count, (previous-position))}
        route = {self.start_pos: None}

        # turn_count is used to promote routes with fewer turns.
        turn_count = {self.start_pos: 0}

        open_pos = []
        heappush(open_pos, (0, self.start_pos))

        while open_pos:
            # Routes with fewest turns_so_far are up first in the priority queue.
            turns_so_far, curr_pos = heappop(open_pos)

            if curr_pos in visited:
                continue

            prev = route[curr_pos]  # Always remember where you came from so we know if we've turned.
            visited.add(curr_pos)  # But keep moving forward. Never go back!

            neighbors_list = self.find_neighbors(curr_pos, prev)
            for pos, did_turn in neighbors_list:
                if pos in visited:
                    continue

                if turn_count.get(pos):  # Have we been here before?
                    # If so, lets update our turn_count with the route containing the fewest turns.
                    turn_count[pos] = min(turn_count[pos], turns_so_far + int(did_turn))
                else:
                    turn_count[pos] = turns_so_far + int(did_turn)

                # In any case add this place to the list of places to explore.
                heappush(open_pos, (turn_count[pos], pos))

                old_route = route.get(pos)  # Do we know of another way to get here?
                # If so, does the old_route take more turns than the current route to get to pos?
                if old_route and turn_count[pos] < old_route[0]:
                    # If pos can be reached in fewer turns by the current route, we overwrite the old route.
                    route[pos] = (turn_count[pos], curr_pos)
                if not old_route:
                    route[pos] = (turn_count[pos], curr_pos)

        # Wait until open_pos is exhausted to ensure a shorter path doesn't end our search prematurely.
        return self.make_path(self.end_pos, route)


class ShortestRoute(AStar):

    """Algorithm to find shortest path between coordinates in a grid style maze

    """

    def find_neighbors(self, curr_pos: tuple) -> list:
        """Finds neighbors of curr_pos.

        Adds coordinates of neighbors to list.

        :param curr_pos: curr_pos of self.search().
        :return: list of tuples [((pos of neighbor), is_turn)].
        """
        limit = len(self.grid) - 1
        neighbors = []
        # Check each position adjacent to curr_pos
        for y, x in zip([0, -1, 0, 1], [1, 0, -1, 0]):
            # If adjacent position is outside of grid, skip it.
            if (curr_pos[0] + y) > limit or (curr_pos[0] + y) < 0 or (curr_pos[1] + x) > limit or (curr_pos[1] + x) < 0:
                continue
            # If adjacent position is a barrier, skip it.
            if self.grid[curr_pos[0] + y][curr_pos[1] + x] == 'x':
                continue
            neighbors.append((curr_pos[0] + y, curr_pos[1] + x))
        return neighbors

    def make_path(self, route: dict):
        """Creates list of coordinates in path.

        :param route: dict of coordinates leading to curr_pos.
        :return: tuple containing list of coordinates.
        """
        path = []
        curr_pos = self.end_pos
        while curr_pos is not None:
            path.append(curr_pos)
            curr_pos = route[curr_pos][1]
        path.reverse()
        return len(path)

    def search(self):
        """Finds shortest path through self.grid.

        Uses a deque to hold coordinates of positions to explore.
        Continually updates length to reach any given position if a shorter path to that position is found.

        :return: self.make_path().
        """
        visited = set()
        route = {self.start_pos: (1, None)}
        open_pos = []
        heappush(open_pos, (1, self.start_pos))
        while open_pos:
            length, curr_pos = heappop(open_pos)
            if curr_pos == self.end_pos:
                return self.make_path(route)
            if curr_pos in visited:
                continue
            visited.add(curr_pos)
            neighbors = self.find_neighbors(curr_pos)
            for neighbor in neighbors:
                if neighbor in visited:
                    continue
                # if neighbor not in open_pos:
                length += 1  # neighbor is one step farther than curr_pos.
                heappush(open_pos, (length, neighbor))
                old_route = route.get(neighbor)  # Do we know of another way to get here?
                # If so, is it shorter than the current route to get to neighbor?
                if old_route and length < old_route[0]:
                    # If neighbor can be reached faster by the current route, we overwrite the old route.
                    route[neighbor] = (length, curr_pos)
                if not old_route:
                    route[neighbor] = (length, curr_pos)
        return 'No path found.', []