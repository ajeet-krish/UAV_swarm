# Distributed UAV Swarm Coordination in 3D Orbital Debris Field

Decentralized multi-agent swarm simulation with 3D Direction Cosine Matrix (DCM) heading-aligned consensus, LQR optimal control, and 3D Artificial Potential Field (APF) guidance. Seven autonomous quadcopters navigate a fixed 3D cube space populated with 6 spherical floating asteroids/debris obstacles.

## Quick Start

```bash
uv sync
uv run python run_simulation.py              # Generate 900-frame JSON (~5s)
uv run python src/viz/animate_3d.py          # Render 3D MP4 with camera orbit (~3 min)
uv run python src/viz/animate_2d.py          # Render 2D animated GIFs (~30s)
uv run python -m http.server -d docs 8765    # Preview site
# Open http://localhost:8765
```

## Architecture

- **Python engine**: 6-DOF state-space dynamics, LQR via CARE, 3D DCM heading-aligned guidance, 3D APF sphere avoidance, Dryden wind turbulence
- **Web viewer**: Matplotlib 3D MP4 video (360 camera orbit), Plotly.js interactive 3D scatter viewer (drag-to-orbit + play/slider controls), 2D animated GIFs
- **Telemetry**: Plotly.js charts for formation error, control effort, algebraic connectivity, and wind magnitude

## Pages

- **Overview** - Project summary, validation metrics, GNC portfolio value
- **Theory & Math** - KaTeX equations for state-space, LQR, 3D DCM, APF, Laplacian, Lyapunov, Dryden
- **3D Simulation** - Animated 3D video with synchronized metric charts
- **Methodology** - Algorithm deep dive: 3D DCM math, consensus, 3D APF spherical distance functions
- **Implementation** - Code architecture, how to run, expandable source blocks

## Key Results

- 7-drone decentralized wedge formation with 3D DCM velocity-vector alignment
- Drones climb, dive, and loop back and forth through a fixed 3D cube space
- Clean avoidance of 6 floating spherical obstacles in any flight direction
- Formation error converges asymptotically and recovers after obstacle clearance and wind gusts (sigma=2 m/s)
- Control effort respects physical saturation limits (T_max = 15N per axis)

## GNC Competencies Demonstrated

- Multi-agent consensus via graph Laplacian
- 3D Direction Cosine Matrix (DCM) coordination
- LQR optimal control synthesis (CARE)
- 3D Artificial Potential Field (APF) guidance
- Dryden wind turbulence modeling
- 6-DOF state-space flight dynamics
