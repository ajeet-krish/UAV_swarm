# Project Context - UAV Swarm Coordination

## Style Rules
- **No em dashes** in any file in this project. No `---`, no `&mdash;`, no `&ndash;`, no literal Unicode em dash (U+2014).
- Use `uv` for all Python dependency management and script execution.

## Goal
Build a decentralized multi-agent UAV swarm simulation with APF guidance, LQR optimal control, and graph Laplacian consensus in a 3D orbital debris field. Deliver a military-green-themed multi-page HTML portfolio demonstrating GNC engineering competencies to aerospace hiring managers.

## Current Status (2026-07-03)

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Project scaffolding (pyproject.toml, src/ layout, .gitignore) | ✅ |
| 2 | Python simulation engine (models, dynamics, consensus, wind, obstacles) | ✅ |
| 3 | Simulation runner + JSON export (900 frames, 18s, 7 drones, 6 spherical obstacles) | ✅ |
| 4 | CSS theme - military green/amber terminal (sidebar nav layout) | ✅ |
| 5 | index.html landing page (hero, stats, card grid, validation metrics) | ✅ |
| 6 | theory.html (KaTeX: state-space, LQR, APF, Laplacian, Lyapunov, Dryden) | ✅ |
| 7 | simulation.html (Matplotlib 3D MP4 viewer + Plotly metric charts + controls) | ✅ |
| 8 | methodology.html (APF/consensus deep dive, 3D DCM formation math, 2D animations) | ✅ |
| 9 | implementation.html (code architecture, how to run, source blocks) | ✅ |
| 10 | AGENTS.md + README.md | ✅ |

## File Layout

```
UAV_swarm/
  AGENTS.md                 # This file
  README.md                 # Project overview
  pyproject.toml             # Dependencies (numpy, scipy, matplotlib)
  uv.lock                    # Lockfile
  run_simulation.py          # Entrypoint: build env -> run sim -> export JSON

  src/
    core/
      __init__.py
      models.py              # QuadcopterAgent (state, parameters, RK4)
      dynamics.py            # LQR gain synthesis (CARE), combined control law
      consensus.py           # Graph Laplacian, APF guidance, 3D DCM rotation
      wind.py                # DrydenGustModel (Gauss-Markov colored noise)
      obstacles.py           # SphereObstacle, BoxObstacle, Environment
    viz/
      __init__.py
      export.py              # FrameExporter, NumpyEncoder
      animate_3d.py          # Matplotlib 3D -> MP4 animation renderer
      animate_2d.py          # Top-down + side-view animated GIF renderer

  docs/                      # GitHub Pages static site
    .nojekyll
    index.html               # Landing page: hero, stats, card grid, GNC value
    theory.html              # Full KaTeX math (6 sections, Lyapunov proof)
    simulation.html          # MP4 animation + Plotly metric charts
    methodology.html         # Algorithm walkthroughs, 3D DCM tables
    implementation.html      # Code architecture, module ref, source blocks
    css/
      style.css              # Military terminal theme (#0d1117, #3fb950, #d29922)
    assets/
      data/
        swarm_simulation.json  # 3.2MB, 900 frames, 7 drones, 3D trajectory
      videos/
        swarm_3d.mp4         # 11MB, 18s, 50fps matplotlib 3D animation
      images/
        top_down.gif         # 1.7MB animated top-down trajectory
        side_view.gif        # 1.2MB animated side-view trajectory
```

## Simulation Architecture

### QuadcopterAgent (models.py)
- State: [x, y, z, vx, vy, vz] in R^6
- Params: m=1.2 kg, b=0.15 Ns/m drag, T_max=15.0 N per axis
- RK4 integration at 50 Hz (dt=0.02s)

### State-Space Dynamics (dynamics.py)
- A matrix: double integrator with drag damping (-b/m = -0.125)
- B matrix: thrust input (1/m = 0.833)
- LQR via scipy.linalg.solve_continuous_are (CARE solver)
- Q=diag(10,10,10,1,1,1), R=0.1*I_3x3

### Velocity-Command Guidance (consensus.py)
- Leader: 3D unit vector to active waypoint * v_cruise
- Followers: leader_vel + k_follow * (target_pos - pos)
- Obstacle repulsion: 3D signed-distance field to spheres with eta * (1/rho - 1/rho0) / rho^2
- Consensus: -kc * sum[(ri - rj) - (di - dj)] within follow_dist

### 3D DCM Formation Rotation
As the leader climbs, dives, and loops through the debris field, the wedge offsets are rotated in 3D using a Direction Cosine Matrix (DCM) aligned with the leader's velocity vector:
- Forward Axis (Ux) = velocity / ||velocity||
- Lateral Axis (Uz) = Ux cross [0, 1, 0] (normalized)
- Vertical Axis (Uy) = Uz cross Ux
- Rotation Matrix R = [Ux, Uy, Uz]
- Rotated Offset d_ij = R @ d_ij_nominal

### Wind Model (wind.py)
- Dryden spectrum via first-order Gauss-Markov process
- sigma=2.0 m/s (light), tau=5.0s correlation, independent 3-axis
- Discrete: w(t+dt) = exp(-dt/tau)*w(t) + sigma*sqrt(1-exp(-2dt/tau))*n

### Obstacles (obstacles.py)
- SphereObstacle: analytical 3D signed distance to center, exact gradient vector
- Environment: aggregates obstacle forces, serializes for web

## Formation Definitions

### Wedge (nominal)
| D0 | D1 | D2 | D3 | D4 | D5 | D6 |
|----|----|----|----|----|----|----|
| (0,0,0) | (-1.5,0,1.5) | (-1.5,0,-1.5) | (-3,0,3) | (-3,0,1) | (-3,0,-1) | (-3,0,-3) |

Wedge offsets are continuously rotated in 3D relative to the direction of travel to stay aligned with the heading.

## Environment
- Fixed 3D Cube: x, z in [-10, 10], y in [1, 11]
- 6 spherical asteroids placed strategically in the 3D space:
  - Sphere 1: Center [4.0, 5.0, 3.0], radius 1.5
  - Sphere 2: Center [-4.0, 4.0, -3.0], radius 1.8
  - Sphere 3: Center [2.0, 8.0, -4.0], radius 1.2
  - Sphere 4: Center [-2.0, 7.0, 4.0], radius 1.4
  - Sphere 5: Center [5.0, 2.0, -5.0], radius 1.6
  - Sphere 6: Center [-5.0, 9.0, 2.0], radius 1.3
- 5 Looping Waypoints:
  - WP1: [-8.0, 2.0, -8.0]
  - WP2: [8.0, 9.0, -8.0]
  - WP3: [8.0, 2.0, 8.0]
  - WP4: [-8.0, 9.0, 8.0]
  - WP5: [0.0, 5.0, 0.0]

## Simulation Output (swarm_simulation.json)
- 900 frames at dt=0.02s = 18s total
- Per frame: drone states (pos, vel, u, d_ij), metrics (formation_error, lambda_2, max_control_effort), wind vector
- File size: ~3.2 MB

## Reference Commands
```bash
uv sync                                      # Install dependencies
uv run python run_simulation.py              # Run sim + export JSON (~5s)
uv run python src/viz/animate_3d.py          # Render 3D MP4 (~3 min)
uv run python src/viz/animate_2d.py          # Render 2D GIFs (~30s)
uv run python -m http.server -d docs 8765    # Preview site
open http://localhost:8765                    # View in browser
```
