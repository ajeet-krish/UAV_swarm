# Project Context - SwarmGNC

## Style Rules
- **No em dashes** in any file in this project. No `---`, no `&mdash;`, no `&ndash;`, no literal Unicode em dash (U+2014). Use `&#8211;` (en dash) or `&#8212;` (em dash) entities instead.
- Use `uv` for all Python dependency management and script execution.

## Goal
Build and deploy a decentralized multi-agent UAV swarm simulation portfolio with 2D APF path planning, 3D formation flight visualization, and interactive HTML analysis pages. Deliver a military-green-themed multi-page HTML portfolio demonstrating GNC engineering competencies to aerospace hiring managers.

## Current Status (2026-07-06)

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
| 19 | Visual refinement (gate dashed lines, HUD text repositioning, full-width player, DPR rounding, font loading) | ✅ |
| 20 | Sidebar cleanup (consistent labels, moved Theory to Reference, removed decorative subheadings, 2D before 3D order) | ✅ |
| 21 | Player polish (larger start/goal icons/text, force vector rework, speed fix, field overlay removal) | ✅ |
| 22 | Homepage redesign (removed stats bar, added intro paragraph, shortened teasers) | ✅ |
| 23 | Rebrand to SwarmGNC (title, sidebar brand, page titles, footer across all pages, collapsible sidebar) | ✅ |

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
      plot_2d_facets.py      # Faceted 2D trajectory grid PNG generator

  docs/                      # GitHub Pages static site
    .nojekyll
    index.html               # Landing page: hero + intro paragraph + teaser cards
    theory.html              # Full KaTeX math (6 sections, Lyapunov proof)
    simulation.html          # Three.js viewer + Plotly metric panels
    2d-simulation.html       # Interactive 2D player + Plotly chart analysis suite
    implementation.html      # Code architecture, module ref, source blocks
    css/
      style.css              # Military terminal theme (#0d1117, #3fb950, #d29922)
    assets/
      data/
        swarm_simulation.json  # 15MB, 4500 frames, 90s, 7 drones, 15 obstacles
        simulation_2d.json     # 6.7MB, 2D APF data (full frame fields for all 10 gains)
      js/
        three_viewer.js      # Three.js viewer module (bloom, CSS2DRenderer labels, trails)
        player_2d.js         # Interactive 2D canvas player (gain selector, playback, force vectors)
      videos/
        apf_path_2d.mp4      # 1.1MB, 30s, 25fps 2D APF corridor path planning animation
      images/
        teaser_2d.png        # 62KB, screenshot of 2D slalom corridor for homepage teaser card
        teaser_3d.png        # 205KB, screenshot of 3D swarm viewer for homepage teaser card
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
- 15 spherical asteroids: 6 hand-placed + 9 procedural with hand-tuned adjustments for narrow passages
- 8 Looping Waypoints (figure-8 with vertical variation)

## Key Visuals

- **Three.js Interactive 3D**: White wireframe+transparent sphere obstacles (15 total), drone markers as emissive cubes with heading cones, vertex-colored gradient trail lines, starfield background, auto-rotate. Uses importmap from jsdelivr, no build tooling.
- **Tron/Mecha Aesthetic**: Rainbow neon DRONE_COLORS. UnrealBloomPass (strength=0.6, radius=0.3, threshold=0.5). Wireframe EdgesGeometry. CSS2DRenderer neon `[D0]`-`[D6]` labels. Reinhard tone mapping, dim ambient (0.3).
- **CSS2DRenderer Drone IDs**: Neon labels float above each drone with text-shadow glow.
- **Plotly Metrics (3D)**: Four panels showing formation error, lambda_2, max control effort, obstacle proximity.
- **Interactive 2D Player**: Full-width canvas player at 3:1 corridor aspect ratio. Neon gain selector chips (10 colors), play/pause/slider, directional force vectors (F_att green, F_rep red, F_wall amber), speed-controlled playback, speed-colored gradient trail, terminal HUD overlay (no background).
- **2D APF Analysis Suite (Plotly)**: Gain sweep overlay, faceted 2x3 trajectory grid, clearance heatmap, speed profile, speed-space phase portrait, gate dashboard, bifurcation chart, parameter space envelope.

## Recent Changes (2026-07-06)

| Change | Details |
|--------|---------|
| Full-width player | Removed max-width constraint on 2D page; player fills viewport width with 3:1 aspect ratio. Corridor scales up proportionally with no empty space. |
| Resolution & font | Canvas dimensions rounded to integer pixels. Font increased to 13px. `document.fonts.ready` re-render for JetBrains Mono load. HUD text moved to x=32 with no background box. |
| Gate visuals | Replaced amber gate-fill overlays with vertical dashed lines at G1-G4. |
| Padding reduced | Canvas padding from 20 to 8 for more effective corridor area. |
| Sidebar cleanup | Consistent labels across all 5 pages: Home, 2D Path Planning, 3D Swarm in Project; Theory & Methodology, Implementation in Reference. Moved Theory out of Project into Reference. Removed decorative subheadings from homepage sidebar. 2D Path Planning ordered before 3D Swarm. |
| File cleanup | Removed `.mypy_cache/` and `__pycache__/` directories. Un-ignored `trajectory_facets.png` in `.gitignore` (needed by 2D page). |
| Start/goal icons | Doubled size of start/goal squares (min 6->12, scale multiplier 0.2->0.4) and font (min 8->14, scale multiplier 0.25->0.5) in player_2d.js. |
| Homepage redesign | Removed stats bar. Added intro paragraph explaining 2D/3D approach, what each provides, and real-world applications. Shortened teaser descriptions to single purpose sentences. |
| Screenshot images | Added teaser_2d.png (62K) and teaser_3d.png (205K) to docs/assets/images/ for homepage cards. Updated .gitignore exceptions. |
| Rebrand to SwarmGNC | Project renamed "SwarmGNC: Decentralized UAV Formation Control". Sidebar brand, page titles, meta descriptions, and footer updated across all 5 pages. Footer now shows "SwarmGNC &middot; Built by Ajeet Krishnasamy &middot; GitHub &middot; LinkedIn". |
| Collapsible sidebar | Sidebar now expandable/collapsible on all screen sizes via hamburger toggle. Slides with transform transition. Content area shifts with padding-left transition. Desktop: sidebar pushes content. Mobile (<900px): sidebar overlays content. |

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
uv run python src/viz/plot_2d_facets.py       # Regenerate faceted trajectory PNG
```
