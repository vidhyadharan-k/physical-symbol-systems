# Heideggerian AI Maze Solver

An exploration of embodied cognition through three complementary agent simulations that demonstrate different approaches to navigation and intelligence — from pure reflexes to symbolic planning.

## Overview

This project implements three distinct agent simulations that explore different approaches to navigation and intelligence, inspired by philosophical and psychological thought experiments about the nature of understanding and agency.

## The Three Simulations

### 1. Heideggerian Agent (`heideggerian_agent.py`)

A reactive agent in the spirit of Brooks's subsumption architecture, inspired by Martin Heidegger's philosophy of being-in-the-world.

**Key Characteristics:**
- **No map, no goal coordinates, no plan** — the agent doesn't "know" where it's going
- **Gradient-field sensor** — samples a scalar field at two points slightly left and right of its heading, steering toward the stronger signal
- The field peaks at the goal and decays linearly with distance — modeling chemotaxis or pheromone-following
- **Proximity sensor** — detects obstacles within a fixed radius
- **Avoidance reflex** — rotates clockwise when an obstacle is sensed

**The Insight:** This is intentionally limited. Pure gradient-following exhibits **local minima** — concave obstacle configurations (U-shaped traps facing the goal) cause the agent to oscillate or stall. This mirrors philosopher Hubert Dreyfus's critique of purely non-representational AI systems and represents the classical limitation identified by Koren & Borenstein (1991).

### 2. Braitenberg Vehicles (`braitenberg_vehicles.py`)

An implementation of Valentino Braitenberg's thought experiments from *Vehicles: Experiments in Synthetic Psychology*.

**Key Characteristics:**
- **Light sources** spawn randomly and fade over time
- **Light sensors** (left and right) detect light intensity
- **Differential drive** — wheel speeds on left/right sides are controlled by sensor readings

**The Concept:** Simple sensor-to-motor connections create complex behaviors. Depending on how sensors are wired to motors:
- Cross-connected → "fear" or "aggression" behaviors
- Directly connected → "love" or "exploration" behaviors

The agent "flees" from or "approaches" light purely based on its sensor wiring — no planning required.

## Connection Between Them

Both explore **embodied cognition** — intelligence without explicit planning or maps:

| Aspect | Heideggerian Agent | Braitenberg Vehicle | Symbolic Planner |
|--------|-------------------|---------------------|------------------|
| Sensing | Gradient field (distance-based) | Light intensity | None (uses internal model) |
| Navigation | Gradient-following | Sensor-driven reflexes | A* path search |
| Philosophy | Heidegger/Dreyfus | Braitenberg | Classical GOFAI |
| World Model | None | None | Full grid discretization |
| Limitation | Local minima traps | Can get stuck in light-rich areas | Model mismatch with reality |

Together, they model the spectrum from pure reactivity to symbolic planning — showing how different architectures handle the same navigation problem.

---

### 3. Symbolic Planner (`planner.py`)

A representation-driven agent: classical GOFAI (Good Old-Fashioned AI) in miniature.

**Key Characteristics:**
- **Complete world model** — The entire environment is discretized onto a grid
- **Symbolic labels** — Every cell is labeled `wall` or `open`
- **Explicit goal coordinates** — The planner is told where start and goal are in grid coordinates
- **A* search** — Finds optimal path through the symbolic model
- **Mechanical execution** — The "robot" then executes the path cell by cell

**The Insight:** This is the architecture Dreyfus and Brooks criticized — a system that operates on an internal model of the world rather than coping with the world itself. The model is handed to it by the designer. There's no sensing during execution, no learning, no adaptation. All cognitive work happens upfront in symbol space, then plays back the result.

The rasterization step — converting circular obstacles into wall cells on a grid — is itself a representational commitment. The planner does not perceive the continuous world; it perceives a symbolic discretization that the programmer constructed.

---

## Connection Between Them

### Requirements
```bash
pip install pygame numpy
```

### Run Heideggerian Agent
```bash
python heideggerian_agent.py
```

### Run Braitenberg Vehicles
```bash
python braitenberg_vehicles.py
```

### Run Symbolic Planner
```bash
python planner.py
```

## Controls

### Heideggerian Agent
- **F** — Toggle gradient field visualization
- **R** — Reset agent position

### Braitenberg Vehicles
- **Space** — Pause/Resume
- **R** — Reset simulation

### Symbolic Planner
- **Space** — Pause/Resume
- **R** — Generate new maze and replan

## Philosophical Context

This project is inspired by a critique of classical AI's assumption that intelligence requires explicit representation and planning. The three simulations demonstrate the spectrum of approaches:

| Approach | Philosophy | Key Insight |
|----------|-----------|-------------|
| **Reactive** | Heidegger/Brooks | Intelligence from sensorimotor coupling |
| **Braitenberg** | Synthetic psychology | Complex behavior from simple wiring |
| **Symbolic** | Classical GOFAI | All cognition happens in symbol space |

This aligns with:
- **Heidegger's** critique of Cartesian representation
- **Rodney Brooks'** subsumption architecture
- **Hubert Dreyfus'** arguments against symbolic AI
- **Valentino Braitenberg's** experiments in synthetic psychology

## Files

```
Maze_solver/
├── heideggerian_agent.py    # Gradient-following reactive agent
├── braitenberg_vehicles.py  # Light-following vehicle simulation
├── planner.py               # A* symbolic path planner (GOFAI)
├── readme.md                # This file
└── requirements.txt        # Python dependencies
```

## Future Work

- Hybrid architecture combining reactive + deliberative layers
- Add sensing to the planner for real-time obstacle avoidance
- Compare path efficiency vs. reactive robustness empirically
- Explore more complex sensor fields (e.g., pheromone diffusion)