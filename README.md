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
- **Web viewer (2D)**: Canvas-based interactive player with selectable k_avoid gain, play/pause/slider controls, gradient trail, force vector arrows, real-time APF potential field (cool blue/purple/red nebula glow), Tron neon aesthetic
- **Telemetry (3D)**: Plotly.js metrics panels for formation error, algebraic connectivity &lambda;<sub>2</sub>, max control effort, and obstacle proximity events
- **Analysis suite (2D)**: Plotly.js gain sweep overlay, faceted trajectory grid, clearance trade-study heatmap, speed profile, speed-space phase portrait, gate performance dashboard, gain bifurcation, parameter space design envelope

## Pages

- **Overview** - Project summary with 2D and 3D teaser cards, hierarchical sidebar with scrollspy navigation
- **Theory & Math** - KaTeX equations for state-space, LQR, 3D DCM, APF, Laplacian, Lyapunov, Dryden
- **3D Simulation** - Three.js interactive viewer + Plotly metric panels (formation error, &lambda;<sub>2</sub>, control effort, proximity)
- **2D Path Planning** - Interactive canvas player + full Plotly analysis suite: gain sweep, faceted comparison, clearance/speed analysis, gate dashboard, bifurcation, parameter space envelope
- **Implementation** - Code architecture, GitHub repo link

## Key Results

- 7-drone decentralized wedge formation with 3D DCM velocity-vector alignment through 90s figure-8 trajectory
- Drones climb, dive, and loop through 15 spherical obstacles with 8 waypoints
- APF repulsion achieves clean obstacle avoidance while maintaining formation cohesion
- Formation error converges asymptotically and recovers after wind gusts (sigma=2 m/s) and obstacle negotiation
- Control effort respects physical saturation limits (T_max = 15N per axis)
- 2D MC sensitivity analysis reveals failure modes at extreme parameter combinations (k_avoid > 6, rho0 < 1.0)
- Interactive 2D player analyzes 10 gain values showing four distinct behavioral regimes: corner-cutting, stall, optimal slalom, pushback

## GNC Competencies Demonstrated

- Multi-agent consensus via graph Laplacian
- 3D Direction Cosine Matrix (DCM) coordination
- LQR optimal control synthesis (CARE)
- 3D Artificial Potential Field (APF) guidance
- Dryden wind turbulence modeling
- 6-DOF state-space flight dynamics
- Monte Carlo parameter sensitivity analysis
- Interactive 3D visualization (Three.js) and 2D canvas rendering with telemetry analytics (Plotly)
