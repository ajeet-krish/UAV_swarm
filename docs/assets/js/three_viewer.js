import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';

const DATA_URL = 'assets/data/swarm_simulation.json';

const BG_COLOR = 0x0a0e14;
const WHITE = 0xc9d1d9;
const OBSTACLE_OPACITY = 0.35;

// Tron-style neon rainbow by drone index
const DRONE_COLORS = [
  0xff00ff,  // D0 magenta (leader)
  0x00ffff,  // D1 cyan
  0xff8800,  // D2 orange
  0x88ff00,  // D3 lime
  0x0088ff,  // D4 electric blue
  0xff0088,  // D5 hot pink
  0xffee00,  // D6 yellow
];

const TRAIL_LEN = 60;
const SKIP = 10;
const AUTO_ROTATE_SPEED = 1.5;

export function initViewer(containerId) {
  const container = document.getElementById(containerId);
  if (!container) return;

  const width = container.clientWidth;
  const height = container.clientHeight || 600;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(BG_COLOR);
  scene.fog = new THREE.Fog(BG_COLOR, 35, 55);

  const camera = new THREE.PerspectiveCamera(40, width / height, 0.1, 100);
  camera.position.set(18, 10, 18);
  camera.lookAt(0, 5, 0);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(width, height);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.toneMapping = THREE.ReinhardToneMapping;
  renderer.toneMappingExposure = 1.0;
  container.appendChild(renderer.domElement);

  // CSS2D overlay for drone ID labels
  const labelRenderer = new CSS2DRenderer();
  labelRenderer.setSize(width, height);
  labelRenderer.domElement.style.position = 'absolute';
  labelRenderer.domElement.style.top = '0';
  labelRenderer.domElement.style.left = '0';
  labelRenderer.domElement.style.pointerEvents = 'none';
  container.appendChild(labelRenderer.domElement);

  // Bloom post-processing
  const composer = new EffectComposer(renderer);
  const renderPass = new RenderPass(scene, camera);
  composer.addPass(renderPass);

  const bloomPass = new UnrealBloomPass(
    new THREE.Vector2(width, height),
    0.6,   // strength - moderate glow
    0.3,   // radius - smooth spread
    0.5    // threshold - only emissive/edge surfaces bloom
  );
  composer.addPass(bloomPass);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 5, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 5;
  controls.maxDistance = 50;
  controls.autoRotate = true;
  controls.autoRotateSpeed = AUTO_ROTATE_SPEED;
  controls.update();

  // Starfield
  const starCount = 3000;
  const starGeo = new THREE.BufferGeometry();
  const starPos = new Float32Array(starCount * 3);
  for (let i = 0; i < starCount; i++) {
    const theta = Math.random() * Math.PI * 2;
    const phi = Math.acos(2 * Math.random() - 1);
    const r = 30 + Math.random() * 70;
    starPos[i * 3] = r * Math.sin(phi) * Math.cos(theta);
    starPos[i * 3 + 1] = r * Math.cos(phi);
    starPos[i * 3 + 2] = r * Math.sin(phi) * Math.sin(theta);
  }
  starGeo.setAttribute('position', new THREE.BufferAttribute(starPos, 3));
  const starMat = new THREE.PointsMaterial({
    color: 0xffffff,
    size: 0.15,
    transparent: true,
    opacity: 0.7,
    sizeAttenuation: true,
  });
  const stars = new THREE.Points(starGeo, starMat);
  scene.add(stars);

  // Dim ambient lighting so emissive materials pop
  const ambient = new THREE.AmbientLight(0x202040, 0.3);
  scene.add(ambient);
  const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
  dirLight.position.set(10, 20, 5);
  scene.add(dirLight);
  const fillLight = new THREE.DirectionalLight(0x8888ff, 0.2);
  fillLight.position.set(-10, 5, -10);
  scene.add(fillLight);

  const grid = new THREE.GridHelper(30, 20, 0x21262d, 0x21262d);
  grid.position.y = 0;
  scene.add(grid);

  let frameData = [];
  let numDrones = 0;
  let obstacles = [];
  let droneGroups = [];
  let droneLabels = [];
  let trailLines = [];
  let trailPositions = [];
  let isPlaying = false;
  let currentFrame = 0;
  let playInterval = null;
  let totalTime = 90;

  fetch(DATA_URL)
    .then(r => r.json())
    .then(data => {
      frameData = data.frames.filter((_, i) => i % SKIP === 0);
      numDrones = data.meta.num_agents;
      obstacles = data.meta.obstacles || [];
      totalTime = data.meta.total_time || 90;
      buildScene(container, data);
    })
    .catch(() => {
      container.innerHTML = '<div style="display:flex;align-items:center;justify-content:center;height:100%;color:#f85149;font-family:monospace;font-size:14px;">Could not load simulation data. Run the simulation first.</div>';
    });

  function buildScene(container, data) {
    for (const obs of obstacles) {
      const c = obs.center;
      const r = obs.radius;
      const geo = new THREE.SphereGeometry(r, 24, 18);
      const mat = new THREE.MeshPhongMaterial({
        color: WHITE,
        transparent: true,
        opacity: OBSTACLE_OPACITY,
        wireframe: false,
        side: THREE.DoubleSide,
        depthWrite: false,
      });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.position.set(c[0], c[1], c[2]);
      scene.add(mesh);

      const wireMat = new THREE.MeshBasicMaterial({
        color: WHITE,
        wireframe: true,
        transparent: true,
        opacity: 0.4,
      });
      const wire = new THREE.Mesh(geo.clone(), wireMat);
      wire.position.copy(mesh.position);
      scene.add(wire);
    }

    const firstDrones = frameData[0].drones;
    for (let di = 0; di < numDrones; di++) {
      const color = DRONE_COLORS[di];
      const size = di === 0 ? 0.35 : 0.25;
      const group = new THREE.Group();

      // Cube body with high emissive for bloom
      const geo = new THREE.BoxGeometry(size * 1.5, size * 1.5, size * 1.5);
      const mat = new THREE.MeshPhongMaterial({
        color: color,
        emissive: color,
        emissiveIntensity: 0.8,
        specular: color,
        shininess: 60,
      });
      const mesh = new THREE.Mesh(geo, mat);
      group.add(mesh);

      // Bright edge wireframe (thick, high contrast)
      const edgeGeo = new THREE.EdgesGeometry(geo);
      const edgeMat = new THREE.LineBasicMaterial({
        color: color,
        transparent: true,
        opacity: 0.85,
      });
      const edgeLine = new THREE.LineSegments(edgeGeo, edgeMat);
      group.add(edgeLine);

      // Heading cone with glow
      const coneGeo = new THREE.ConeGeometry(size * 0.4, size * 1.0, 6);
      const coneMat = new THREE.MeshPhongMaterial({
        color: color,
        emissive: color,
        emissiveIntensity: 0.6,
      });
      const cone = new THREE.Mesh(coneGeo, coneMat);
      cone.position.x = size * 1.2;
      cone.rotation.z = -Math.PI / 2;
      group.add(cone);

      const pos = firstDrones[di].pos;
      group.position.set(pos[0], pos[1], pos[2]);
      scene.add(group);
      droneGroups.push(group);

      // CSS2D label
      const hexStr = '#' + new THREE.Color(color).getHexString();
      const el = document.createElement('span');
      el.textContent = `[D${di}]`;
      el.style.color = hexStr;
      el.style.fontFamily = 'JetBrains Mono, monospace';
      el.style.fontSize = '11px';
      el.style.fontWeight = '700';
      el.style.textShadow = `0 0 8px ${hexStr}, 0 0 20px ${hexStr}`;
      el.style.background = 'rgba(10,14,20,0.6)';
      el.style.padding = '2px 6px';
      el.style.borderRadius = '3px';
      el.style.border = `1px solid ${hexStr}`;
      el.style.letterSpacing = '0.5px';
      const label = new CSS2DObject(el);
      label.position.set(pos[0], pos[1] + 0.8, pos[2]);
      scene.add(label);
      droneLabels.push(label);

      // Trail with vertex color gradient
      const trailMat = new THREE.LineBasicMaterial({
        vertexColors: true,
        transparent: true,
        opacity: 0.5,
      });
      const trailGeo = new THREE.BufferGeometry();
      const positions = new Float32Array(TRAIL_LEN * 3);
      const colors = new Float32Array(TRAIL_LEN * 3);
      trailGeo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      trailGeo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
      trailGeo.setDrawRange(0, 0);
      const line = new THREE.Line(trailGeo, trailMat);
      scene.add(line);
      trailLines.push(line);
      trailPositions.push([]);
    }

    buildControls(container);
    updateFrame(0);
    renderLoop();
  }

  function buildControls(container) {
    const hud = document.createElement('div');
    hud.className = 'sim-hud';
    hud.innerHTML = '<div class="sim-hud-line"><span class="label">> t = </span><span class="value" id="hudTime">0.0s</span></div>';
    container.style.position = 'relative';
    container.appendChild(hud);

    const ctrlDiv = document.createElement('div');
    ctrlDiv.className = 'sim-controls';

    const playBtn = document.createElement('button');
    playBtn.textContent = '\u25B6';
    playBtn.id = 'threePlayBtn';

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = 0;
    slider.max = frameData.length - 1;
    slider.value = 0;
    slider.id = 'threeSlider';

    const timeLabel = document.createElement('span');
    timeLabel.className = 'time-display';
    timeLabel.id = 'threeTimeLabel';
    timeLabel.textContent = '0.0s / ' + totalTime.toFixed(1) + 's';

    playBtn.onclick = () => {
      if (isPlaying) {
        isPlaying = false;
        playBtn.textContent = '\u25B6';
        clearInterval(playInterval);
        controls.autoRotate = true;
      } else {
        isPlaying = true;
        controls.autoRotate = false;
        playBtn.textContent = '\u25A0';
        playInterval = setInterval(() => {
          let next = currentFrame + 1;
          if (next >= frameData.length) next = 0;
          slider.value = next;
          updateFrame(next);
          const t = frameData[next].t;
          document.getElementById('hudTime').textContent = t.toFixed(1) + 's';
          document.getElementById('threeTimeLabel').textContent = t.toFixed(1) + 's / ' + totalTime.toFixed(1) + 's';
        }, 60);
      }
    };

    slider.oninput = () => {
      const fi = parseInt(slider.value);
      if (isPlaying) {
        isPlaying = false;
        playBtn.textContent = '\u25B6';
        clearInterval(playInterval);
      }
      controls.autoRotate = true;
      updateFrame(fi);
      const t = frameData[fi].t;
      document.getElementById('hudTime').textContent = t.toFixed(1) + 's';
      document.getElementById('threeTimeLabel').textContent = t.toFixed(1) + 's / ' + totalTime.toFixed(1) + 's';
    };

    ctrlDiv.appendChild(playBtn);
    ctrlDiv.appendChild(slider);
    ctrlDiv.appendChild(timeLabel);
    container.parentNode.insertBefore(ctrlDiv, container.nextSibling);
  }

  function updateFrame(fi) {
    if (fi < 0 || fi >= frameData.length) return;
    currentFrame = fi;
    const frame = frameData[fi];
    const drones = frame.drones;

    for (let di = 0; di < numDrones && di < drones.length; di++) {
      const d = drones[di];
      const pos = d.pos;
      const vel = d.vel;
      const group = droneGroups[di];

      group.position.set(pos[0], pos[1], pos[2]);

      // Update CSS2D label
      droneLabels[di].position.set(pos[0], pos[1] + 0.8, pos[2]);

      const speed = Math.sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]);
      if (speed > 0.01) {
        const dir = new THREE.Vector3(vel[0], vel[1], vel[2]).normalize();
        const forward = new THREE.Vector3(1, 0, 0);
        const quat = new THREE.Quaternion().setFromUnitVectors(forward, dir);
        group.quaternion.copy(quat);
      }

      // Trail
      const trail = trailPositions[di];
      trail.push([pos[0], pos[1], pos[2]]);
      if (trail.length > TRAIL_LEN) trail.shift();

      const line = trailLines[di];
      const posAttr = line.geometry.attributes.position;
      const colAttr = line.geometry.attributes.color;
      const array = posAttr.array;
      const colArray = colAttr.array;
      const baseColor = new THREE.Color(DRONE_COLORS[di]);

      for (let j = 0; j < trail.length; j++) {
        array[j * 3] = trail[j][0];
        array[j * 3 + 1] = trail[j][1];
        array[j * 3 + 2] = trail[j][2];
        const tVal = j / Math.max(trail.length - 1, 1);
        const c = baseColor.clone().multiplyScalar(0.08 + 0.92 * tVal);
        colArray[j * 3] = c.r;
        colArray[j * 3 + 1] = c.g;
        colArray[j * 3 + 2] = c.b;
      }
      posAttr.needsUpdate = true;
      colAttr.needsUpdate = true;
      line.geometry.setDrawRange(0, trail.length);
    }
  }

  function renderLoop() {
    requestAnimationFrame(renderLoop);
    controls.update();
    composer.render();
    labelRenderer.render(scene, camera);
  }

  window.addEventListener('resize', () => {
    const w = container.clientWidth;
    const h = container.clientHeight || 600;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
    composer.setSize(w, h);
    labelRenderer.setSize(w, h);
  });
}
