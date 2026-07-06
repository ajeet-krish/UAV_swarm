# Project Context - UAV Swarm Coordination

## Style Rules
- **No em dashes** in any file in this project. No `---`, no `&mdash;`, no `&ndash;`, no literal Unicode em dash (U+2014).
- Use `uv` for all Python dependency management and script execution.

## Goal
Build a decentralized multi-agent UAV swarm simulation with APF guidance, LQR optimal control, and graph Laplacian consensus in a 3D orbital debris field. Deliver a military-green-themed multi-page HTML portfolio demonstrating GNC engineering competencies to aerospace hiring managers.

## Current Status (2026-07-05)

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Project scaffolding (pyproject.toml, src/ layout, .gitignore) | ✅ |
| 2 | Python simulation engine (models, dynamics, consensus, wind, obstacles) | ✅ |
| 3 | Simulation runner + JSON export (4500 frames, 90s, 7 drones, 15 spherical obstacles) | ✅ |
| 4 | CSS theme - military green/amber terminal (sidebar nav layout) | ✅ |
| 5 | index.html landing page (hero, stats, card grid, validation metrics) | ✅ |
| 6 | theory.html (KaTeX: state-space, LQR, APF, Laplacian, Lyapunov, Dryden) | ✅ |
| 7 | simulation.html (Three.js interactive 3D viewer + Plotly metric charts) | ✅ |
| 8 | implementation.html (code architecture, how to run, source blocks) | ✅ |
| 9 | 2D APF path planning simulation (run_simulation_2d.py, animate_2d_sim.py) | ✅ |
| 10 | Interactive 3D viewer (Three.js importmap, cubes+cones, starfield, gradient trails, auto-rotate) | ✅ |
| 11 | 2D simulation HTML page (gain overlay, MC stats, sensitivity heatmap, Plotly charts) | ✅ |
| 12 | Tron/mecha Three.js viewer (UnrealBloomPass, CSS2DRenderer neon IDs, rainbow colors) | ✅ |
| 13 | 3D Plotly metrics panels (formation error, lambda_2, control effort, obstacle proximity) | ✅ |
| 14 | 2D sim enhancement (MC stress sensitivity, speed/force telemetry, heavy-density obstacles) | ✅ |
| 15 | Cleanup (removed plot_2d_analysis.py, static PNGs, AGENTS.md/README.md/.gitignore update) | ✅ |
| 16 | Homepage revamp (hero + teaser cards + hierarchical scrollspy sidebar) | ✅ |
| 17 | Interactive 2D player (canvas-based, gain selector, APF field, Tron neon aesthetic) | ✅ |
| 18 | Player fixes (space aesthetic background, repulsive-only field, cool colors, blocky obstacle fix) | ✅ |

## File Layout

```
UAV_swarm/
  AGENTS.md                 # This file
  README.md                 # Project overview
  pyproject.toml             # Dependencies (numpy, scipy, matplotlib)
  uv.lock                    # Lockfile
  run_simulation.py          # Entrypoint: build env -> run sim -> export JSON
  run_simulation_2d.py       # 2D APF corridor path planning simulation

  src/
    core/
      __init__.py
      models.py              # QuadcopterAgent (state, parameters, RK4)
      dynamics.py            # LQR gain synthesis (CARE), combined control law
      consensus.py           # Graph Laplacian, APF guidance, 3D DCM rotation
      wind.py                # DrydenGustModel (Gauss-Markov colored noise)
      obstacles.py           # SphereObstacle, Environment
    viz/
      __init__.py
      export.py              # FrameExporter, NumpyEncoder
      animate_3d.py          # Matplotlib 3D -> MP4 animation renderer
      animate_2d.py          # Top-down + side-view animated GIF renderer
      animate_2d_sim.py      # 2D APF corridor MP4 animation renderer

  docs/                      # GitHub Pages static site
    .nojekyll
    index.html               # Landing page: hero + teaser cards + scrollspy sidebar
    theory.html              # Full KaTeX math (6 sections, Lyapunov proof)
    simulation.html          # Three.js viewer + Plotly metric panels
    2d-simulation.html       # Interactive 2D player + Plotly chart analysis suite
    implementation.html      # Code architecture, module ref, source blocks
    css/
      style.css              # Military terminal theme (#0d1117, #3fb950, #d29922)
    assets/
      data/
        swarm_simulation.json  # 15MB, 4500 frames, 90s, 7 drones, 15 obstacles
        simulation_2d.json     # 4.0MB, 2D APF data (full frame fields for all 10 gains)
      js/
        three_viewer.js      # Three.js viewer module (bloom, CSS2DRenderer labels, trails)
        player_2d.js         # Interactive 2D canvas player (gain selector, APF field, playback)
      videos/
        apf_path_2d.mp4      # 1.1MB, 30s, 25fps 2D APF corridor path planning animation
      images/
        teaser_2d.jpg        # 28KB, screenshot of slalom layout for homepage teaser card
        trajectory_facets.png # 233KB, static 3x2 faceted trajectory grid
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
- 15 spherical asteroids: 6 hand-placed + 9 procedural with hand-tuned adjustments for narrow passages:
  - Asteroid 1: Center [4.0, 5.0, 3.0], radius 1.5
  - Asteroid 2: Center [-4.0, 4.0, -3.0], radius 1.8
  - Asteroid 3: Center [2.0, 8.0, -4.0], radius 1.2
  - Asteroid 4: Center [-2.0, 7.0, 4.0], radius 1.4
  - Asteroid 5: Center [5.0, 2.0, -5.0], radius 1.6
  - Asteroid 6: Center [-5.0, 9.0, 2.0], radius 1.3
  - Asteroids 7-15: Placed procedurally (seed=42) then hand-adjusted to create narrow corridors, flanking obstacles, and approach obstacles
- 8 Looping Waypoints (figure-8 with vertical variation):
  - WP1: [8.0, 9.0, -8.0]      High right
  - WP2: [8.0, 2.0, 8.0]       Low opposite corner
  - WP3: [-2.0, 8.0, 6.0]      High center-right gap
  - WP4: [-8.0, 9.0, 8.0]      High left
  - WP5: [-3.0, 3.0, -3.0]     Low center-left through obstacles
  - WP6: [0.0, 5.0, 0.0]       Center pass
  - WP7: [5.0, 7.0, -5.0]      Mid right figure-8 crossover
  - WP8: [-8.0, 2.0, -8.0]     Start (loop back)

## Simulation Output (swarm_simulation.json)
- 4500 frames at dt=0.02s = 90s total
- Per frame: drone states (pos, vel, u, d_ij), metrics (formation_error, lambda_2, max_control_effort), wind vector
- File size: ~15 MB

## Key Visuals

- **Three.js Interactive 3D**: Drag-to-orbit viewport with OrbitControls. White wireframe+transparent sphere obstacles (15 total), drone markers as emissive cubes with heading cones oriented along velocity, vertex-colored gradient trail lines (60 pts per drone), starfield background with scene fog, auto-rotate camera when paused. Play/pause/slider controls and HUD overlay. Uses importmap (three@0.170.0 from jsdelivr) with no build tooling.
- **Tron/Mecha Aesthetic**: Rainbow neon DRONE_COLORS (magenta, cyan, orange, lime, electric blue, hot pink, yellow) applied to cubes, heading cones, trail gradients, and CSS2DRenderer labels. UnrealBloomPass (strength=0.6, radius=0.3, threshold=0.5) creates glow on emissive surfaces only. Wireframe EdgesGeometry on each cube. Starfield (3000 pts) with scene fog, Reinhard tone mapping, dim ambient (0.3).
- **CSS2DRenderer Drone IDs**: Neon `[D0]`-`[D6]` labels float above each drone, styled with the drone's rainbow color and text-shadow glow. Rendered via CSS2DRenderer for crisp text at any zoom.
- **Plotly Metrics Panels (3D)**: Four single-column panels in simulation.html show formation error, algebraic connectivity lambda_2, max control effort, and obstacle proximity events. Proximity chart samples up to 200 events colored by drone index.
- **Interactive 2D Player**: Canvas-based playback with neon Tron aesthetic. Selectable k_avoid gain via 10 colored chips, play/pause/slider controls, force vector arrows (F_att green, F_rep red, F_wall amber), speed-colored gradient trail, APF repulsive potential field as a cool blue/purple/red nebula glow overlay, toggle checkboxes for layers.
- **2D APF Analysis Suite (Plotly)**: Gain sweep overlay (10 gains), faceted 2x3 trajectory grid, clearance per gain scatter, clearance trade-study heatmap, speed profile overlay, speed-space phase portrait, gate performance dashboard (reachability, crossing speed, zone clearance, transit time), gain bifurcation chart, parameter space design envelope (deterministic + MC stress), results table.

## Recent Changes (2026-07-05)

| Change | Details |
|--------|---------|
| Homepage revamp | index.html: replaced full content with hero + 2D teaser card (screenshot + text) + 3D teaser card (text + placeholder) + stats bar. Added hierarchical sidebar with scrollspy IntersectionObserver. Removed old card grid, validation metrics, GNC value list. |
| 2D interactive player | player_2d.js (new, 671 lines): canvas-based interactive simulation viewer. Gain selector chips (10 neon colors), play/pause/slider, speed buttons, 4 toggle checkboxes. Drone rendered as oriented triangle with shadowBlur glow. Gradient trail. Force vector arrows. Real-time APF repulsive potential field. Terminal HUD overlay. |
| Full frame data export | run_simulation_2d.py: gain_sweep now stores all frame fields (vel, F_att, F_rep, F_wall, dist_to_goal) instead of only t/pos/speed. simulation_2d.json grew from 3.2MB to 4.0MB. |
| Player visual fixes | Space aesthetic: dark background fill, repulsive-only APF field with cool blue/purple/red nebula gradient, solid obstacle base to hide blocky field underneath, GRID_RES 8->4 for smoother field rendering. |
| Page cleanup | Removed methodology.html (content folded into theory + simulation pages). Removed swarm_3d.mp4, top_down.gif, side_view.gif (generated on demand). Updated sidebar nav across all pages. |

## Reference Commands
```bash
uv sync                                      # Install dependencies
uv run python run_simulation.py              # Run sim + export JSON (~5s)
uv run python src/viz/animate_3d.py          # Render 3D MP4 (~3 min)
uv run python src/viz/animate_2d.py          # Render 2D GIFs (~30s)
uv run python -m http.server -d docs 8765    # Preview site
open http://localhost:8765                    # View in browser
uv run python src/viz/animate_2d_sim.py       # Render 2D APF MP4 (~15s)
uv run python run_simulation_2d.py            # Run 2D sim + export JSON (~20s)
```
