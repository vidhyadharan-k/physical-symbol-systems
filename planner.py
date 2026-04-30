"""
Symbolic Planner Simulation
============================
A representation-driven agent: classical GOFAI in miniature.

Where the Heideggerian agent has only two scalar sensor readings per frame,
this planner has the ENTIRE WORLD as an internal symbolic model:
  - The continuous environment is discretised onto a grid (rasterisation).
  - Every cell is symbolically labelled `wall` or `open`.
  - The planner is told where the start and goal are, in grid coordinates.
  - A* searches the model and returns an optimal path.
  - The "robot" then mechanically executes the path, cell by cell.

There is no sensing during execution. There is no learning. There is no
adaptation. The agent does all of its cognitive work upfront, in symbol
space, then plays back the result.

This is the architecture Dreyfus and Brooks criticised: a system that
operates on an internal model of the world rather than coping with the
world itself. The model is handed to it by the designer (you).

The rasterisation step at startup — converting circular obstacles into
wall-cells on a grid — is itself a representational commitment. The
planner does not perceive the continuous world; it perceives a symbolic
discretisation of it that the programmer constructed.

Dependencies: pip install pygame
"""

import math
import random
import sys
import heapq
import time
import pygame

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
WIDTH, HEIGHT   = 800, 600
FPS             = 60
BG_COLOR        = (10, 10, 18)

CELL_SIZE       = 20                          # grid cell in pixels
GRID_W          = WIDTH  // CELL_SIZE
GRID_H          = HEIGHT // CELL_SIZE

AGENT_RADIUS    = 8
AGENT_COLOR     = (255, 180, 100)             # warm — contrasts with the
                                              # blue Heideggerian agent
AGENT_SPEED     = 3.0                         # pixels per frame along path

OBSTACLE_COLOR  = (80, 70, 120)
WALL_CELL_COLOR = (45, 40, 70)
GRID_LINE_COLOR = (28, 28, 40)
EXPLORED_COLOR  = (70, 50, 110)
PATH_COLOR      = (100, 220, 200)
START_COLOR     = (60, 180, 240)
GOAL_COLOR      = (60, 230, 140)

NUM_OBSTACLES   = 35
OBS_R_MIN       = 14
OBS_R_MAX       = 30
CLEAR_MARGIN    = 55


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def gen_obstacles(start_px, goal_px, n=NUM_OBSTACLES):
    """Generate circular obstacles in pixel space (same format as the
    Heideggerian agent's world)."""
    obstacles = []
    for _ in range(n * 3):
        r = random.uniform(OBS_R_MIN, OBS_R_MAX)
        x = random.uniform(r, WIDTH  - r)
        y = random.uniform(r, HEIGHT - r)

        ok = all(math.hypot(x - o[0], y - o[1]) > r + o[2] + 10
                 for o in obstacles)
        ok = ok and math.hypot(x - start_px[0], y - start_px[1]) > r + CLEAR_MARGIN
        ok = ok and math.hypot(x - goal_px[0],  y - goal_px[1])  > r + CLEAR_MARGIN

        if ok:
            obstacles.append((x, y, r))
        if len(obstacles) >= n:
            break
    return obstacles


def rasterise(obstacles):
    """Convert continuous circular obstacles into a discrete grid of
    wall/open cells. THIS IS THE REPRESENTATIONAL COMMITMENT — the
    planner does not see circles; it sees this grid that we hand it.

    Cells are marked 'wall' if their centre lies inside any obstacle,
    or within the agent's radius of any obstacle (so the agent can fit)."""
    grid = [[False] * GRID_H for _ in range(GRID_W)]   # False = open
    for gx in range(GRID_W):
        for gy in range(GRID_H):
            cx = gx * CELL_SIZE + CELL_SIZE / 2
            cy = gy * CELL_SIZE + CELL_SIZE / 2
            for ox, oy, orad in obstacles:
                if math.hypot(cx - ox, cy - oy) < orad + AGENT_RADIUS:
                    grid[gx][gy] = True
                    break
    return grid


def px_to_cell(px, py):
    return (int(px // CELL_SIZE), int(py // CELL_SIZE))


def cell_to_px(gx, gy):
    return (gx * CELL_SIZE + CELL_SIZE / 2,
            gy * CELL_SIZE + CELL_SIZE / 2)


# ---------------------------------------------------------------------------
# A* search
# ---------------------------------------------------------------------------
def astar(grid, start_cell, goal_cell):
    """Classical A* on a 4-connected grid. Returns:
        path: list of (gx, gy) from start to goal (or None if unreachable)
        explored: set of (gx, gy) cells popped from the open set (for vis)
    """
    sx, sy = start_cell
    gx, gy = goal_cell

    if grid[sx][sy] or grid[gx][gy]:
        return None, set()

    def h(c):
        # Manhattan distance — admissible for 4-connected grids.
        return abs(c[0] - gx) + abs(c[1] - gy)

    open_heap = []
    heapq.heappush(open_heap, (h(start_cell), 0, start_cell, None))
    came_from = {}
    g_score   = {start_cell: 0}
    explored  = set()

    while open_heap:
        f, g, cur, parent = heapq.heappop(open_heap)
        if cur in came_from:
            continue
        came_from[cur] = parent
        explored.add(cur)

        if cur == goal_cell:
            # reconstruct path
            path = []
            node = cur
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            return path, explored

        cx, cy = cur
        for dx, dy in ((1,0), (-1,0), (0,1), (0,-1)):
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < GRID_W and 0 <= ny < GRID_H):
                continue
            if grid[nx][ny]:
                continue
            ng = g + 1
            nbr = (nx, ny)
            if ng < g_score.get(nbr, float('inf')):
                g_score[nbr] = ng
                heapq.heappush(open_heap, (ng + h(nbr), ng, nbr, cur))

    return None, explored


# ---------------------------------------------------------------------------
# Path-following "agent" — purely cosmetic
# ---------------------------------------------------------------------------
class PathFollower:
    """Walks along a precomputed list of cells. Has no sensors. Has no
    awareness of its environment. It is the inverse of the Heideggerian
    agent: it cannot perceive, only execute."""
    def __init__(self, path_cells):
        self.path = [cell_to_px(*c) for c in path_cells]
        self.idx = 0
        self.x, self.y = self.path[0]
        self.done = False

    def update(self, speed=AGENT_SPEED):
        if self.done:
            return
        tx, ty = self.path[self.idx]
        dx, dy = tx - self.x, ty - self.y
        d = math.hypot(dx, dy)
        if d < speed:
            self.x, self.y = tx, ty
            self.idx += 1
            if self.idx >= len(self.path):
                self.done = True
        else:
            self.x += speed * dx / d
            self.y += speed * dy / d


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------
def draw_grid(surface, grid, explored=None, path=None):
    # walls (the symbolic representation the planner operates on)
    for gx in range(GRID_W):
        for gy in range(GRID_H):
            if grid[gx][gy]:
                pygame.draw.rect(surface, WALL_CELL_COLOR,
                                 (gx*CELL_SIZE, gy*CELL_SIZE,
                                  CELL_SIZE, CELL_SIZE))

    # explored cells (made the search legible)
    if explored:
        for (gx, gy) in explored:
            if not grid[gx][gy]:
                s = pygame.Surface((CELL_SIZE, CELL_SIZE), pygame.SRCALPHA)
                s.fill((*EXPLORED_COLOR, 90))
                surface.blit(s, (gx*CELL_SIZE, gy*CELL_SIZE))

    # the planned path
    if path and len(path) > 1:
        pts = [cell_to_px(*c) for c in path]
        pygame.draw.lines(surface, PATH_COLOR, False,
                          [(int(p[0]), int(p[1])) for p in pts], 2)

    # grid lines (subtle — show the discretisation)
    for gx in range(GRID_W + 1):
        x = gx * CELL_SIZE
        pygame.draw.line(surface, GRID_LINE_COLOR, (x, 0), (x, HEIGHT), 1)
    for gy in range(GRID_H + 1):
        y = gy * CELL_SIZE
        pygame.draw.line(surface, GRID_LINE_COLOR, (0, y), (WIDTH, y), 1)


def draw_obstacles_outline(surface, obstacles):
    """Show the 'real' (continuous) obstacles faintly underneath the grid,
    so viewers can see what the rasterisation discarded."""
    for ox, oy, orad in obstacles:
        pygame.draw.circle(surface, OBSTACLE_COLOR,
                           (int(ox), int(oy)), int(orad), 1)


def draw_endpoints(surface, start_px, goal_px):
    pygame.draw.circle(surface, START_COLOR,
                       (int(start_px[0]), int(start_px[1])), 6)
    pygame.draw.circle(surface, GOAL_COLOR,
                       (int(goal_px[0]),  int(goal_px[1])),  6)


def draw_agent(surface, agent):
    if agent is None:
        return
    pygame.draw.circle(surface, AGENT_COLOR,
                       (int(agent.x), int(agent.y)), AGENT_RADIUS)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def setup_world():
    """Build a fresh world: obstacles, grid, plan."""
    START = (60, HEIGHT - 60)
    GOAL  = (WIDTH - 100, 80)
    obstacles = gen_obstacles(START, GOAL)
    grid = rasterise(obstacles)

    start_cell = px_to_cell(*START)
    goal_cell  = px_to_cell(*GOAL)

    t0 = time.perf_counter()
    path, explored = astar(grid, start_cell, goal_cell)
    plan_ms = (time.perf_counter() - t0) * 1000

    follower = PathFollower(path) if path else None

    stats = {
        "plan_ms": plan_ms,
        "explored": len(explored),
        "path_len": len(path) if path else 0,
        "reachable": path is not None,
    }
    return START, GOAL, obstacles, grid, explored, path, follower, stats


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Symbolic Planner — A* on a discretised grid")
    clock  = pygame.time.Clock()
    font   = pygame.font.SysFont("monospace", 13)

    START, GOAL, obstacles, grid, explored, path, follower, stats = setup_world()
    speed = AGENT_SPEED

    running = True
    paused  = False

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_n:
                    (START, GOAL, obstacles, grid, explored,
                     path, follower, stats) = setup_world()
                elif event.key == pygame.K_r:
                    follower = PathFollower(path) if path else None
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_UP:
                    speed = min(8.0, speed + 0.5)
                elif event.key == pygame.K_DOWN:
                    speed = max(0.5, speed - 0.5)

        if not paused and follower is not None:
            follower.update(speed)

        # -- draw --
        screen.fill(BG_COLOR)
        draw_obstacles_outline(screen, obstacles)
        draw_grid(screen, grid, explored=explored, path=path)
        draw_endpoints(screen, START, GOAL)
        draw_agent(screen, follower)

        # HUD
        if stats["reachable"]:
            hud = (f"plan: {stats['plan_ms']:.1f}ms  |  "
                   f"explored: {stats['explored']} cells  |  "
                   f"path: {stats['path_len']} cells  |  "
                   f"speed: {speed:.1f}  |  "
                   f"[N] new  [R] re-execute  [SPACE] pause  [↑↓] speed")
        else:
            hud = ("UNREACHABLE — no path exists in the rasterised grid.  "
                   "[N] new maze")
        txt = font.render(hud, True, (170, 170, 200))
        screen.blit(txt, (10, HEIGHT - 22))

        if paused:
            pt = font.render("PAUSED", True, (220, 180, 80))
            screen.blit(pt, (WIDTH // 2 - 30, 12))

        if follower is not None and follower.done:
            dt = font.render("GOAL REACHED  —  press N for new maze",
                             True, (100, 220, 200))
            screen.blit(dt, (WIDTH // 2 - 160, 12))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()