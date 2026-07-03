# Project Context -- UAV Swarm Coordination

## Style Rules
- **No em dashes** in any file in this project. No `---`, no `&mdash;`, no `&ndash;`, no literal Unicode em dash (U+2014).
- Use `uv` for all Python dependency management and script execution.

## Goal
Build a decentralized multi-agent UAV swarm simulation with APF guidance, LQR optimal control, and graph Laplacian consensus. Deliver a military-green-themed multi-page HTML portfolio demonstrating GNC engineering competencies to aerospace hiring managers.

## Current Status (2026-07-03)

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Project scaffolding (pyproject.toml, src/ layout, .gitignore) | ✅ |
| 2 | Python simulation engine (models, dynamics, consensus, wind, obstacles) | ✅ |
| 3 | Simulation runner + JSON export (900 frames, 18s, 7 drones, 3+ obstacles) | ✅ |
| 4 | CSS theme -- military green/amber terminal (sidebar nav layout) | ✅ |
| 5 | index.html landing page (hero, stats, card grid, validation metrics) | ✅ |
| 6 | theory.html (KaTeX: state-space, LQR, APF, Laplacian, Lyapunov, Dryden) | ✅ |
| 7 | simulation.html (Three.js 3D viewer + Plotly metric charts + controls) | ✅ |
| 8 | methodology.html (APF/consensus deep dive, formation tables) | ✅ |
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
      consensus.py           # Graph Laplacian, APF guidance, formation error
      wind.py                # DrydenGustModel (Gauss-Markov colored noise)
      obstacles.py           # BoxObstacle, CylinderObstacle, Environment
    viz/
      __init__.py
      export.py              # FrameExporter, NumpyEncoder, vector field sampling
      animate_3d.py          # Matplotlib 3D -> MP4 animation renderer
      animate_2d.py          # Top-down + side-view animated GIF renderer

  docs/                      # GitHub Pages static site
    .nojekyll
    index.html               # Landing page: hero, stats, card grid, GNC value
    theory.html              # Full KaTeX math (6 sections, Lyapunov proof)
    simulation.html          # MP4 animation + Plotly metric charts
    methodology.html         # Algorithm walkthroughs, formation tables
    implementation.html      # Code architecture, module ref, source blocks
    css/
      style.css              # Military terminal theme (#0d1117, #3fb950, #d29922)
    assets/
      data/
        swarm_simulation.json  # 3.2MB, 900 frames, 7 drones, vector fields
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
- Leader: unit vector to target * v_cruise, damped by k_follow
- Followers: leader_vel + k_follow * (target_pos - pos)
- Obstacle repulsion: signed-distance fields with eta * (1/rho - 1/rho0) / rho^2
- Consensus: -kc * sum[(ri - rj) - (di - dj)] within follow_dist

### Wind Model (wind.py)
- Dryden spectrum via first-order Gauss-Markov process
- sigma=2.0 m/s (light), tau=5.0s correlation, independent 3-axis
- Discrete: w(t+dt) = exp(-dt/tau)*w(t) + sigma*sqrt(1-exp(-2dt/tau))*n

### Obstacles (obstacles.py)
- BoxObstacle: signed distance to AABB, negative interior = hard repulsion
- CylinderObstacle: radial + axial signed distance, capped ends
- Environment: aggregates obstacle forces, serializes for web

## Formation Definitions

### Wedge (initial, t < 8.0s)
| D0 | D1 | D2 | D3 | D4 | D5 | D6 |
|----|----|----|----|----|----|----|
| (0,0,0) | (-1.5,0,1.5) | (-1.5,0,-1.5) | (-3,0,3) | (-3,0,1) | (-3,0,-1) | (-3,0,-3) |

### Diamond (morphed, t > 10.0s)
| D0 | D1 | D2 | D3 | D4 | D5 | D6 |
|----|----|----|----|----|----|----|
| (0,1.5,0) | (-1.5,0.5,1.5) | (-1.5,0.5,-1.5) | (-3,-0.5,2) | (-3,-0.5,0.5) | (-3,-0.5,-0.5) | (-3,-0.5,-2) |

Smooth linear morph over 2.0s starting at t=8.0s.

## Environment
- Canyon corridor: two box buildings at z=-8..-3.5 (left) and z=3.5..8 (right)
  - Width: 7m corridor (-3.5 to 3.5), Length: 33m (-8 to 25), Height: 11m (-1 to 10)
- Three cylinder pillars inside corridor:
  - (3, 2, -1.5), (7, 2, 1.5), (11, 2, -0.5) -- all r=0.4m, h=6m
- Target waypoint: (20, 3, 0)
- Start: leader at (-6, 2.5, 0), followers in wedge behind

## Simulation Output (swarm_simulation.json)
- 900 frames at dt=0.02s = 18s total
- Per frame: drone states (pos, vel, u, d_ij), metrics (formation_error, lambda_2, max_control_effort), wind vector
- Vector field sampled at 5x3x5 lattice every 10 frames (~1300 vectors total)
- File size: ~3.2 MB

## Website Pages

### CSS Theme (style.css)
- Military terminal: bg #0d1117, green #3fb950, amber #d29922, red #f85149
- Monospace throughout (JetBrains Mono)
- Fixed 240px sidebar with section navigation
- Responsive: sidebar collapses on mobile (<900px)
- Component styles: cards, stats, equation boxes, module table, simulation layout

### index.html
- Hero with status badge (pulsing green dot)
- Stats bar: 7 drones, 6-DOF, LQR, APF, 3 obstacles
- 4-card grid linking to Theory, Simulation, Methodology, Implementation
- GNC validation metrics section
- Portfolio value checklist for hiring managers

### theory.html (KaTeX heavy)
1. State-Space Dynamics (A, B matrices)
2. LQR Optimal Control (cost function, CARE, K matrix)
3. APF (target, formation, obstacle potentials)
4. Graph Laplacian (adjacency, L, lambda_2 properties)
5. Lyapunov Stability Proof (V-dot negative definite)
6. Dryden Wind Model (PSD, Gauss-Markov discrete update)

### simulation.html (main interactive page)
- Split layout: 3D viewport (60%) + chart panel (40%)
- MP4 video animation generated by matplotlib (50fps, 18s, dark theme)
  - 7 drone spheres (leader amber, followers green) with fading trails
  - Formation ghost targets at d_ij offsets
  - Semi-transparent canyon walls + pillar cylinders
  - HUD overlay: time, formation state, error, wind
  - Panning camera follows swarm center
- Plotly charts (4 in sidebar, 4 below):
  - Formation error vs time (with morph event dashed line)
  - Control effort vs time (with T_max=15N saturation line)
  - Algebraic connectivity lambda_2
  - Wind gust magnitude
- Metric explanation cards below

### methodology.html
- Decentralized consensus algorithm explanation
- Velocity-command guidance architecture (leader vs follower)
- Obstacle distance functions (box, cylinder)
- Formation morphing strategy with offset tables
- Wind gust rejection mechanics

### implementation.html
- Full file tree
- Module reference table (7 modules)
- How to run: uv sync, uv run sim, http server
- Simulation parameters table
- 3 expandable source code blocks (Prism.js: LQR, guidance, wind)

## Known Issues
1. **JSON file size**: 3.2 MB for 900 frames. Acceptable for local serve and GitHub Pages. If this is too large, reduce to 450 frames (skip every other).
2. **Simulation determinism**: The Dryden wind model uses a seeded RNG. Running the sim multiple times produces identical results. To change behavior, modify the seed or sigma parameters.
3. **Vector field sampling**: Sampled every 10 frames at a 5x3x5 lattice (75 points). Adds ~25% to file size. Can be reduced by sampling every 20 frames or using a coarser grid.

## Reference Commands
```bash
uv sync                                      # Install dependencies
uv run python run_simulation.py              # Run sim + export JSON (~5s)
uv run python src/viz/animate_3d.py          # Render 3D MP4 (~3 min)
uv run python src/viz/animate_2d.py          # Render 2D GIFs (~30s)
uv run python -m http.server -d docs 8765    # Preview site
open http://localhost:8765                    # View in browser
```

## Key Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| dt | 0.02 s | Integration timestep |
| Total time | 18.0 s | Simulation duration |
| Num agents | 7 | Swarm size |
| m | 1.2 kg | Per-agent mass |
| b | 0.15 Ns/m | Drag coefficient |
| T_max | 15.0 N | Per-axis thrust limit |
| v_cruise | 3.0 m/s | Nominal speed |
| R_comm | 8.0 m | Communication radius |
| sigma_wind | 2.0 m/s | Gust intensity |
| morph_time | 8.0 s | Formation morph trigger |
| morph_duration | 2.0 s | Morph transition time |

## Future Work
- Add user-triggered formation morph button in the 3D viewer
- Implement "lost drone" scenario (temporarily disable one agent's comms)
- Add actuator dynamics (second-order response model)
- Extend to MPC formulation instead of LQR
- Add Rosetta/SU2 CFD validation for individual drone aerodynamics
- Implement collision-free trajectory planning with Voronoi diagrams
