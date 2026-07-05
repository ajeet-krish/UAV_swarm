# Distributed UAV Swarm Coordination in 3D Orbital Debris Field

Decentralized multi-agent swarm simulation with 3D Direction Cosine Matrix (DCM) heading-aligned consensus, LQR optimal control, and 3D Artificial Potential Field (APF) guidance. Seven autonomous quadcopters navigate a fixed 3D cube space populated with 15 spherical floating asteroids/debris obstacles.

## Quick Start

```bash
uv sync
uv run python run_simulation.py              # Generate 4500-frame JSON (~5s)
uv run python src/viz/animate_3d.py          # Render 3D MP4 with camera orbit (~3 min)
uv run python src/viz/animate_2d.py          # Render 2D animated GIFs (~30s)
uv run python run_simulation_2d.py           # Run 2D APF sim + export JSON (~20s)
uv run python src/viz/animate_2d_sim.py       # Render 2D APF MP4 (~15s)
uv run python -m http.server -d docs 8765    # Preview site
# Open http://localhost:8765
```

## Architecture

- **Python engine**: 6-DOF state-space dynamics, LQR via CARE, 3D DCM heading-aligned guidance, 3D APF sphere avoidance, Dryden wind turbulence
- **Web viewer (3D)**: Three.js interactive viewport with Tron/mecha aesthetic -- emissive cubes, heading cones, CSS2DRenderer neon drone labels, UnrealBloomPass post-processing, vertex-color gradient trails, starfield, auto-orbit on pause
- **Web viewer (2D)**: Plotly.js gain sweep overlay, force breakdown (stacked area), speed profile, MC stress sensitivity grid, layout difficulty bar chart, faceted 2x3 trajectory comparison
- **Telemetry (3D)**: Plotly.js metrics panels for formation error, algebraic connectivity &lambda;<sub>2</sub>, max control effort, and obstacle proximity events

## Pages

- **Overview** - Project summary, validation metrics, GNC portfolio value
- **Theory & Math** - KaTeX equations for state-space, LQR, 3D DCM, APF, Laplacian, Lyapunov, Dryden
- **3D Simulation** - Three.js interactive viewer + Plotly metric panels (formation error, &lambda;<sub>2</sub>, control effort, proximity)
- **2D Path Planning** - 2D APF corridor analysis: gain sweep overlay, MC stats, stress sensitivity heatmap, force breakdown, layout difficulty
- **Methodology** - Algorithm deep dive: 3D DCM math, consensus, 3D APF spherical distance functions
- **Implementation** - Code architecture, GitHub repo link

## Key Results

- 7-drone decentralized wedge formation with 3D DCM velocity-vector alignment through 90s figure-8 trajectory
- Drones climb, dive, and loop through 15 spherical obstacles with 8 waypoints
- APF repulsion achieves clean obstacle avoidance while maintaining formation cohesion
- Formation error converges asymptotically and recovers after wind gusts (sigma=2 m/s) and obstacle negotiation
- Control effort respects physical saturation limits (T_max = 15N per axis)
- 2D MC sensitivity analysis reveals failure modes at extreme parameter combinations (k_avoid > 6, rho0 < 1.0)

## GNC Competencies Demonstrated

- Multi-agent consensus via graph Laplacian
- 3D Direction Cosine Matrix (DCM) coordination
- LQR optimal control synthesis (CARE)
- 3D Artificial Potential Field (APF) guidance
- Dryden wind turbulence modeling
- 6-DOF state-space flight dynamics
- Monte Carlo parameter sensitivity analysis
- Interactive 3D visualization (Three.js) and telemetry analytics (Plotly)
