"""
Heideggerian Agent Simulation
==============================
A reactive agent in the spirit of Brooks's subsumption architecture.

The agent has NO map, NO goal coordinates, NO plan. It has only:
  - A gradient-field sensor: samples a scalar field at two points slightly
    left and right of its heading, and steers toward the stronger side.
    The field is peaked at the goal and decays linearly with distance.
    The agent never reads the goal's position; it only feels the local
    gradient. This models phenomena like chemotaxis or pheromone-following.
  - A proximity sensor: detects obstacles within a fixed radius.
  - An avoidance reflex: rotates clockwise when an obstacle is sensed,
    holding the turn for a few frames before resuming gradient-following.

Known limitation (intentionally retained):
  Pure gradient-following exhibits LOCAL MINIMA — concave obstacle
  configurations (U-shaped traps facing the goal) cause the agent to
  oscillate or stall, because every locally-sensed direction leads to
  a lower field value. This is the classical limitation of reactive
  navigation (Koren & Borenstein, 1991) and is precisely the kind of
  ceiling Dreyfus identified for purely non-representational systems.

Dependencies: pip install pygame numpy
"""

import math
import random
import sys
import pygame

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
WIDTH, HEIGHT   = 800, 600
FPS             = 60
BG_COLOR        = (10, 10, 18)

AGENT_RADIUS    = 8
AGENT_COLOR     = (100, 180, 255)
AGENT_SPEED     = 2.0

SENSOR_RADIUS       = 50           # proximity detection range (pixels)
GRADIENT_PROBE_DIST = 14           # distance ahead to sample the field
GRADIENT_PROBE_HALF = math.radians(25)  # ± angle from heading for L/R probes

TURN_MIN_DEG    = 30
TURN_MAX_DEG    = 40

ATTRACTOR_COLOR = (60, 230, 140)
OBSTACLE_COLOR  = (80, 70, 120)
OBSTACLE_HOT    = (220, 80, 60)
TRAIL_MAX       = 500

NUM_OBSTACLES   = 35
OBS_R_MIN       = 14
OBS_R_MAX       = 30
CLEAR_MARGIN    = 55

# Field parameters: linear decay, capped non-negative.
# field(p) = max(0, FIELD_MAX - distance(p, goal))
FIELD_MAX       = math.hypot(WIDTH, HEIGHT)   # ensures field reaches everywhere

SHOW_FIELD      = False  # toggle with F to visualise the gradient


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def angle_diff(target, current):
    d = target - current
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def field_value(x, y, goal):
    """Scalar field: peaks at goal, decays linearly. Pure sensor input
    for the agent — note the agent never sees `goal` directly, only the
    field value at points it samples."""
    d = math.hypot(x - goal[0], y - goal[1])
    return max(0.0, FIELD_MAX - d)


def gen_obstacles(start, goal, n=NUM_OBSTACLES):
    obstacles = []
    for _ in range(n * 3):
        r  = random.uniform(OBS_R_MIN, OBS_R_MAX)
        x  = random.uniform(r, WIDTH  - r)
        y  = random.uniform(r, HEIGHT - r)

        ok = all(math.hypot(x - o[0], y - o[1]) > r + o[2] + 10
                 for o in obstacles)
        ok = ok and math.hypot(x - start[0], y - start[1]) > r + CLEAR_MARGIN
        ok = ok and math.hypot(x - goal[0],  y - goal[1])  > r + CLEAR_MARGIN

        if ok:
            obstacles.append((x, y, r))
        if len(obstacles) >= n:
            break
    return obstacles


# ---------------------------------------------------------------------------
# Agent
# ---------------------------------------------------------------------------
class Agent:
    def __init__(self, x, y, angle=0.0):
        self.x = float(x)
        self.y = float(y)
        self.angle = float(angle)
        self.avoid_timer = 0
        self.avoid_angle = 0.0

    # -- Gradient-field sensor ---------------------------------------------
    # Pure sensing: returns scalar readings from two probe points slightly
    # left and right of the agent's heading. No coordinates, no distance,
    # no direction-to-goal — only two field samples. This is the
    # philosophically critical method.
    def _sample_gradient(self, goal):
        left_a  = self.angle - GRADIENT_PROBE_HALF
        right_a = self.angle + GRADIENT_PROBE_HALF

        lx = self.x + math.cos(left_a)  * GRADIENT_PROBE_DIST
        ly = self.y + math.sin(left_a)  * GRADIENT_PROBE_DIST
        rx = self.x + math.cos(right_a) * GRADIENT_PROBE_DIST
        ry = self.y + math.sin(right_a) * GRADIENT_PROBE_DIST

        return field_value(lx, ly, goal), field_value(rx, ry, goal)

    # Goal-seeking behaviour: differential steering on gradient samples.
    # FIX 2: steering cap is higher when no obstacle is nearby, so the
    # agent recovers heading quickly in open space.
    def _seek(self, f_left, f_right, obstacle_nearby):
        diff = f_left - f_right
        cap = 0.07 if obstacle_nearby else 0.15
        steer = clamp(diff * 0.002, -cap, cap)
        # Convention: positive angle = clockwise. Steer toward stronger side.
        self.angle -= steer

    # -- Proximity sensor & avoidance reflex -------------------------------
    # FIX 1: avoidance now consults the gradient probes to turn toward the
    # goal-side rather than always clockwise. Still no map, no memory of
    # past obstacles — just two sensor channels (proximity + gradient)
    # cooperating in the moment. This is the kind of behavioural
    # composition subsumption architectures support.
    # FIX 3: avoidance hold reduced from 12 frames to 7.
    def _sense_and_avoid(self, obstacles, f_left, f_right):
        sensed = [o for o in obstacles
                  if math.hypot(self.x - o[0], self.y - o[1])
                     < SENSOR_RADIUS + o[2]]

        if sensed:
            deg = random.uniform(TURN_MIN_DEG, TURN_MAX_DEG)
            # Turn toward the side with stronger gradient — toward goal-side.
            sign = -1.0 if f_left > f_right else 1.0
            self.avoid_angle = sign * math.radians(deg) / 7
            self.avoid_timer = 7

        if self.avoid_timer > 0:
            self.angle += self.avoid_angle
            self.avoid_timer -= 1

        return sensed

    # -- Movement & collision check ----------------------------------------
    def _move(self, obstacles, speed):
        nx = self.x + math.cos(self.angle) * speed
        ny = self.y + math.sin(self.angle) * speed

        blocked = any(math.hypot(nx - o[0], ny - o[1])
                      < AGENT_RADIUS + o[2] - 2
                      for o in obstacles)

        if blocked:
            deg = random.uniform(TURN_MIN_DEG, TURN_MAX_DEG)
            self.angle += math.radians(deg)
            self.avoid_timer = 15
            return

        if not (AGENT_RADIUS < nx < WIDTH  - AGENT_RADIUS):
            self.angle = math.pi - self.angle
        if not (AGENT_RADIUS < ny < HEIGHT - AGENT_RADIUS):
            self.angle = -self.angle

        self.x = clamp(nx, AGENT_RADIUS, WIDTH  - AGENT_RADIUS)
        self.y = clamp(ny, AGENT_RADIUS, HEIGHT - AGENT_RADIUS)

    # -- Master update step ------------------------------------------------
    # Behaviour priority (subsumption-style):
    #   1. Sample gradient (always — sensing is cheap and shared).
    #   2. If avoidance reflex is active, it sets the angle change and
    #      suppresses goal-seeking entirely.
    #   3. Otherwise, gradient-following sets the angle change.
    #   4. Movement applies, with hard collision check overriding both.
    def update(self, goal, obstacles, speed=AGENT_SPEED):
        f_left, f_right = self._sample_gradient(goal)

        obstacle_nearby = any(
            math.hypot(self.x - o[0], self.y - o[1]) < SENSOR_RADIUS + o[2]
            for o in obstacles
        )

        # Goal-seeking runs only when avoidance is not active.
        # This is the suppression: avoidance, when active, fully overrides
        # the goal-seeking layer.
        if self.avoid_timer == 0:
            self._seek(f_left, f_right, obstacle_nearby)

        sensed = self._sense_and_avoid(obstacles, f_left, f_right)
        self._move(obstacles, speed)
        return sensed


# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------
def draw_field(surface, goal):
    """Optional visualisation of the scalar field (toggle with F).
    Slow — samples on a coarse grid."""
    step = 20
    max_f = FIELD_MAX
    for gx in range(0, WIDTH, step):
        for gy in range(0, HEIGHT, step):
            f = field_value(gx, gy, goal)
            intensity = int(60 * (f / max_f))
            pygame.draw.rect(surface, (intensity, intensity // 2, 0),
                             (gx, gy, step, step))


def draw_trail(surface, trail):
    if len(trail) < 2:
        return
    n = len(trail)
    for i in range(1, n):
        pygame.draw.line(surface, (60, 120, 200),
                         (int(trail[i-1][0]), int(trail[i-1][1])),
                         (int(trail[i][0]),   int(trail[i][1])), 1)


def draw_obstacles(surface, obstacles, hot_set):
    for o in obstacles:
        x, y, r = int(o[0]), int(o[1]), int(o[2])
        col = OBSTACLE_HOT if o in hot_set else OBSTACLE_COLOR
        pygame.draw.circle(surface, col, (x, y), r)
        border = tuple(min(255, c + 60) for c in col)
        pygame.draw.circle(surface, border, (x, y), r, 1)


def draw_attractor(surface, pos):
    x, y = int(pos[0]), int(pos[1])
    for ring_r, alpha in [(22, 40), (14, 80), (8, 200)]:
        s = pygame.Surface((ring_r*2, ring_r*2), pygame.SRCALPHA)
        pygame.draw.circle(s, (*ATTRACTOR_COLOR, alpha),
                           (ring_r, ring_r), ring_r)
        surface.blit(s, (x - ring_r, y - ring_r))
    pygame.draw.circle(surface, (255, 255, 255), (x, y), 3)


def draw_agent(surface, agent):
    x, y = int(agent.x), int(agent.y)

    # proximity sensor ring
    s = pygame.Surface((SENSOR_RADIUS*2, SENSOR_RADIUS*2), pygame.SRCALPHA)
    pygame.draw.circle(s, (100, 180, 255, 25),
                       (SENSOR_RADIUS, SENSOR_RADIUS), SENSOR_RADIUS)
    pygame.draw.circle(s, (100, 180, 255, 60),
                       (SENSOR_RADIUS, SENSOR_RADIUS), SENSOR_RADIUS, 1)
    surface.blit(s, (x - SENSOR_RADIUS, y - SENSOR_RADIUS))

    # gradient probe points (visualise where the agent is sampling)
    left_a  = agent.angle - GRADIENT_PROBE_HALF
    right_a = agent.angle + GRADIENT_PROBE_HALF
    lx = x + int(math.cos(left_a)  * GRADIENT_PROBE_DIST)
    ly = y + int(math.sin(left_a)  * GRADIENT_PROBE_DIST)
    rx = x + int(math.cos(right_a) * GRADIENT_PROBE_DIST)
    ry = y + int(math.sin(right_a) * GRADIENT_PROBE_DIST)
    pygame.draw.circle(surface, (255, 200, 100), (lx, ly), 2)
    pygame.draw.circle(surface, (255, 200, 100), (rx, ry), 2)
    pygame.draw.line(surface, (255, 200, 100, 80), (x, y), (lx, ly), 1)
    pygame.draw.line(surface, (255, 200, 100, 80), (x, y), (rx, ry), 1)

    # body
    pygame.draw.circle(surface, AGENT_COLOR, (x, y), AGENT_RADIUS)

    # heading arrow
    ex = x + int(math.cos(agent.angle) * (AGENT_RADIUS + 7))
    ey = y + int(math.sin(agent.angle) * (AGENT_RADIUS + 7))
    pygame.draw.line(surface, (255, 255, 255), (x, y), (ex, ey), 2)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    global SHOW_FIELD

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Heideggerian Agent — gradient-field sensor")
    clock  = pygame.time.Clock()
    font   = pygame.font.SysFont("monospace", 13)

    START = (60, HEIGHT - 60)
    GOAL  = (WIDTH - 100, 80)
    obstacles  = gen_obstacles(START, GOAL)
    agent      = Agent(*START, angle=-math.pi / 4)
    trail      = []
    avoidances = 0
    speed      = AGENT_SPEED
    sensed     = []

    # Stall detection — for the "agent got stuck in local minimum" feature
    stall_window = []
    STALL_FRAMES = 180   # 3 seconds at 60 FPS

    running = True
    paused  = False

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    agent = Agent(*START, angle=-math.pi / 4)
                    trail.clear()
                    stall_window.clear()
                    avoidances = 0
                elif event.key == pygame.K_n:
                    obstacles = gen_obstacles(START, GOAL)
                    agent = Agent(*START, angle=-math.pi / 4)
                    trail.clear()
                    stall_window.clear()
                    avoidances = 0
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_f:
                    SHOW_FIELD = not SHOW_FIELD
                elif event.key == pygame.K_UP:
                    speed = clamp(speed + 0.25, 0.5, 6.0)
                elif event.key == pygame.K_DOWN:
                    speed = clamp(speed - 0.25, 0.5, 6.0)

        if not paused:
            prev_avoid = agent.avoid_timer
            sensed = agent.update(GOAL, obstacles, speed)

            if sensed and prev_avoid == 0:
                avoidances += 1

            trail.append((agent.x, agent.y))
            if len(trail) > TRAIL_MAX:
                trail.pop(0)

            # track recent positions to detect stalls (local minima)
            stall_window.append((agent.x, agent.y))
            if len(stall_window) > STALL_FRAMES:
                stall_window.pop(0)

            if math.hypot(agent.x - GOAL[0], agent.y - GOAL[1]) < AGENT_RADIUS + 14:
                obstacles = gen_obstacles(START, GOAL)
                agent = Agent(*START, angle=-math.pi / 4)
                trail.clear()
                stall_window.clear()
                avoidances = 0

        # detect if agent has been thrashing in the same area
        stalled = False
        if len(stall_window) == STALL_FRAMES:
            xs = [p[0] for p in stall_window]
            ys = [p[1] for p in stall_window]
            if max(xs) - min(xs) < 40 and max(ys) - min(ys) < 40:
                stalled = True

        screen.fill(BG_COLOR)
        if SHOW_FIELD:
            draw_field(screen, GOAL)
        draw_trail(screen, trail)
        draw_obstacles(screen, obstacles, set(sensed if not paused else []))
        draw_attractor(screen, GOAL)
        draw_agent(screen, agent)

        dist = math.hypot(agent.x - GOAL[0], agent.y - GOAL[1])
        hud = (f"dist: {int(dist)}px  |  avoidances: {avoidances}  |  "
               f"speed: {speed:.1f}  |  "
               f"[SPACE] pause [R] reset [N] new maze [F] field [↑↓] speed")
        txt = font.render(hud, True, (120, 160, 220))
        screen.blit(txt, (10, HEIGHT - 22))

        if paused:
            pt = font.render("PAUSED", True, (220, 180, 80))
            screen.blit(pt, (WIDTH // 2 - 30, 12))

        if stalled:
            st = font.render("LOCAL MINIMUM — agent stalled (press N for new maze)",
                             True, (220, 100, 100))
            screen.blit(st, (WIDTH // 2 - 200, 12))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()