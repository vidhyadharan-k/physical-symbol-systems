# Heideggerian AI Maze Solver

An exploration of embodied cognition through two complementary agent simulations that demonstrate how complex behavior can emerge from simple reactive mechanisms — without explicit planning, maps, or goal representations.

## Overview

This project implements two distinct agent simulations that explore different approaches to navigation and intelligence, both inspired by philosophical and psychological thought experiments about the nature of understanding and agency.

## The Two Simulations

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

| Aspect | Heideggerian Agent | Braitenberg Vehicle |
|--------|-------------------|---------------------|
| Sensing | Gradient field (distance-based) | Light intensity |
| Navigation | Gradient-following | Sensor-driven reflexes |
| Philosophy | Heidegger/Dreyfus | Braitenberg |
| Limitation | Local minima traps | Can get stuck in light-rich areas |

Together, they model how simple reactive systems can produce sophisticated-looking behavior purely from local sensing, without central planning.

## Running the Simulations

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

## Controls

### Heideggerian Agent
- **F** — Toggle gradient field visualization
- **R** — Reset agent position

### Braitenberg Vehicles
- **Space** — Pause/Resume
- **R** — Reset simulation

## Philosophical Context

This project is inspired by a critique of classical AI's assumption that intelligence requires explicit representation and planning. Both approaches demonstrate that:

1. **Situatedness** — Agents can act effectively in the world without internal models of that world
2. **Embodiment** — Intelligence emerges from sensorimotor coupling with the environment
3. **Emergence** — Complex behaviors arise from simple rules

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
├── planner.py               # (Reserved for future path-planning extension)
├── readme.md                # This file
└── requirements.txt         # Python dependencies
```

## Future Work

- Implement a path-planning layer to overcome local minima
- Hybrid architecture combining reactive + deliberative layers
- Explore more complex sensor fields (e.g., pheromone diffusion)