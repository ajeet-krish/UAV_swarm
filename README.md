# Distributed UAV Swarm Coordination & Kinetic Collision Avoidance

Decentralized multi-agent swarm simulation with APF guidance, LQR optimal control, and graph Laplacian consensus. Seven autonomous quadcopters navigate a constrained urban canyon with wind gust disturbances.

## Quick Start

```bash
uv sync
uv run python run_simulation.py
uv run python -m http.server -d docs 8765
# Open http://localhost:8765
```

## Architecture

- **Python engine**: 6-DOF state-space dynamics, LQR via CARE, APF velocity-command guidance, Dryden wind turbulence
- **Web viewer**: Three.js WebGL 3D scene with orbit controls, drone trails, obstacle rendering, vector field arrows
- **Telemetry**: Plotly.js charts for formation error, control effort, connectivity, and wind magnitude

## Pages

- **Overview** -- Project summary, validation metrics, GNC portfolio value
- **Theory & Math** -- KaTeX equations for state-space, LQR, APF, Laplacian, Lyapunov, Dryden
- **3D Simulation** -- Interactive Three.js viewer with synchronized metric charts
- **Methodology** -- Algorithm deep-dive: consensus, APF, morphing, obstacle distance functions
- **Implementation** -- Code architecture, how to run, expandable source blocks

## Key Results

- 7-drone decentralized formation with wedge-to-diamond reconfiguration
- Formation error converges to bounded neighborhood under wind gusts (sigma=2 m/s)
- Algebraic connectivity lambda_2 > 0 throughout (fully connected topology)
- Control effort respects T_max = 15N per-axis thrust limits
- Smooth formation morph over 2.0s at mid-trajectory

## GNC Competencies Demonstrated

- Multi-agent consensus via graph Laplacian
- LQR optimal control synthesis (CARE)
- Artificial Potential Field guidance
- Dryden wind turbulence modeling
- 6-DOF state-space flight dynamics
- Formation reconfiguration under decentralized control
