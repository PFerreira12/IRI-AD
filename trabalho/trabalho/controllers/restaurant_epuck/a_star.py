import heapq

class AStar:

    def __init__(self, known_map):
        self.map = known_map


    def plan(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct(came_from, current)

            for nb in self.get_neighbors(current):

                r, c = nb

                if not self.map.in_bounds(r, c):
                    continue

                if self.map.grid[r][c] == 1:
                    continue

                tentative = g[current] + 1

                if nb not in g or tentative < g[nb]:
                    g[nb] = tentative
                    f = tentative + self.heuristic(nb, goal)
                    heapq.heappush(open_set, (f, nb))
                    came_from[nb] = current

        return []


    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])


    def get_neighbors(self, node):
        x, y = node
        return [
            (x+1, y), (x-1, y),
            (x, y+1), (x, y-1)
        ]


    def reconstruct(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]