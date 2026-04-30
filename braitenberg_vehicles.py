import pygame
import random
import math
import time

# ================= CONFIG =================
WIDTH, HEIGHT = 900, 600
FPS = 60

LIGHT_LIFETIME = 6
LIGHT_SPAWN_INTERVAL = 2

# ================= INIT =================
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Braitenberg Vehicles Simulation")

clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 12)

# ================= UTIL =================
def distance(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)

# ================= ENVIRONMENT =================
class Light:
    def __init__(self):
        self.x = random.randint(50, WIDTH - 50)
        self.y = random.randint(50, HEIGHT - 50)
        self.spawn_time = time.time()

    def alive(self):
        return time.time() - self.spawn_time < LIGHT_LIFETIME

    def draw(self):
        pygame.draw.circle(screen, (255, 255, 100), (int(self.x), int(self.y)), 8)


class Environment:
    def __init__(self):
        self.lights = []
        self.last_spawn = time.time()

    def update(self):
        # spawn lights
        if time.time() - self.last_spawn > LIGHT_SPAWN_INTERVAL:
            self.lights.append(Light())
            self.last_spawn = time.time()

        # remove expired lights
        self.lights = [l for l in self.lights if l.alive()]

    def draw(self):
        for l in self.lights:
            l.draw()

    def reset_lights(self):
        self.lights.clear()


# ================= SENSOR =================
class LightSensor:
    def __init__(self, offset_angle):
        self.offset_angle = offset_angle

    def read(self, agent, lights):
        sx = agent.x + math.cos(agent.angle + self.offset_angle) * 10
        sy = agent.y + math.sin(agent.angle + self.offset_angle) * 10

        intensity = 0
        for l in lights:
            d = distance(sx, sy, l.x, l.y)
            if d > 0:
                intensity += 1 / d

        return intensity


# ================= ACTUATOR =================
class DifferentialDrive:
    def __init__(self, speed=4):  # increased speed
        self.speed = speed

    def move(self, agent, v_l, v_r):
        # clamp to avoid instability
        v_l = max(min(v_l, 2), -2)
        v_r = max(min(v_r, 2), -2)

        turn = (v_r - v_l) * 2
        agent.angle += turn

        velocity = v_l + v_r
        agent.x += math.cos(agent.angle) * velocity * self.speed
        agent.y += math.sin(agent.angle) * velocity * self.speed

        # wrap around
        agent.x %= WIDTH
        agent.y %= HEIGHT


# ================= BEHAVIORS =================
class Behavior:
    name = "Unknown"
    def act(self, left, right):
        raise NotImplementedError


class FearBehavior(Behavior):
    name = "Fear"
    def act(self, left, right):
        return left, right


class AggressionBehavior(Behavior):
    name = "Aggression"
    def act(self, left, right):
        return right, left


class LoveBehavior(Behavior):
    name = "Love"
    def act(self, left, right):
        return right * 0.5, left * 0.5


class ExplorerBehavior(Behavior):
    name = "Explorer"
    def act(self, left, right):
        return right + random.random(), left + random.random()


# ================= AGENT =================
class Agent:
    def __init__(self, x, y, behavior, color):
        self.x = x
        self.y = y
        self.angle = random.uniform(0, 2 * math.pi)

        self.left_sensor = LightSensor(+0.3)
        self.right_sensor = LightSensor(-0.3)

        self.behavior = behavior
        self.actuator = DifferentialDrive()

        self.color = color

    def update(self, env):
        left = self.left_sensor.read(self, env.lights)
        right = self.right_sensor.read(self, env.lights)

        v_l, v_r = self.behavior.act(left, right)

        self.actuator.move(self, v_l, v_r)

    def draw(self):
        # body
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), 6)

        # orientation line
        dx = math.cos(self.angle) * 10
        dy = math.sin(self.angle) * 10
        pygame.draw.line(
            screen,
            (255, 255, 255),
            (self.x, self.y),
            (self.x + dx, self.y + dy),
            2
        )

        # label
        label = font.render(self.behavior.name, True, (255, 255, 255))
        screen.blit(label, (self.x - 25, self.y - 18))


# ================= FACTORY =================
def create_agent():
    behavior_map = [
        (FearBehavior(), (255, 100, 100)),
        (AggressionBehavior(), (255, 0, 0)),
        (LoveBehavior(), (100, 255, 100)),
        (ExplorerBehavior(), (100, 100, 255))
    ]

    behavior, color = random.choice(behavior_map)

    return Agent(
        random.randint(0, WIDTH),
        random.randint(0, HEIGHT),
        behavior,
        color
    )


# ================= MAIN =================
env = Environment()
agents = [create_agent() for _ in range(15)]

running = True
while running:
    screen.fill((0, 0, 0))

    # events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                env.reset_lights()

    # update environment
    env.update()
    env.draw()

    # update agents
    for agent in agents:
        agent.update(env)
        agent.draw()

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()