(function(global) {
'use strict';

var GAIN_KEYS = ['0.05','0.1','0.2','0.5','1','2','3','4','6','10'];
var GAIN_COLORS = ['#ff00ff','#00ffff','#ff8800','#88ff00','#0088ff','#ff0088','#ffee00','#3fb950','#58a6ff','#bc8cff'];
var GAIN_RGB = GAIN_COLORS.map(hexToRgb);

var BG_COLOR = '#0a0e14';
var X_MIN = -15, X_MAX = 15, Y_MIN = -5, Y_MAX = 5;
var K_ATT = 0.375;
var K_WALL = 3.0;
var WALL_MARGIN = 2.0;
var RHO0 = 3.5;
var TRAIL_LEN = 150;
var GRID_RES = 4;

var state = {
  data: null, meta: null, gainKey: '4', gainFrames: null,
  currentFrame: 0, totalFrames: 1500, isPlaying: false,
  playInterval: null, speed: 1, trail: [],
  showForces: true, showField: true, showTrail: true, showRadii: false,
  dirty: true,
  fieldCache: null, fieldGrid: null,
};

var container, canvas, ctx, width, height, dpr = 1;
var padding = 20, scale = 1;
var gainChips = {}, playBtn, slider, timeLabel;

function hexToRgb(h) {
  var r = parseInt(h.slice(1,3), 16);
  var g = parseInt(h.slice(3,5), 16);
  var b = parseInt(h.slice(5,7), 16);
  return [r, g, b];
}

function toCanvasX(x) { return padding + (x - X_MIN) * scale; }
function toCanvasY(y) { return padding + (Y_MAX - y) * scale; }
function toWorldX(cx) { return X_MIN + (cx - padding) / scale; }
function toWorldY(cy) { return Y_MAX - (cy - padding) / scale; }

function computeScale() {
  var aw = width - padding * 2;
  var ah = height - padding * 2;
  var corAspect = (X_MAX - X_MIN) / (Y_MAX - Y_MIN);
  var canAspect = aw / ah;
  scale = canAspect > corAspect ? ah / (Y_MAX - Y_MIN) : aw / (X_MAX - X_MIN);
}

function markDirty() { state.dirty = true; }

function resize() {
  var rect = container.getBoundingClientRect();
  dpr = window.devicePixelRatio || 1;
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  canvas.style.width = rect.width + 'px';
  canvas.style.height = rect.height + 'px';
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  width = rect.width;
  height = rect.height;
  computeScale();
  state.fieldCache = null;
  markDirty();
}

function updateFrame(fi) {
  if (!state.gainFrames || fi < 0 || fi >= state.gainFrames.length) return;
  state.currentFrame = fi;
  var start = Math.max(0, fi - TRAIL_LEN);
  state.trail = [];
  for (var i = start; i <= fi; i++) {
    state.trail.push(state.gainFrames[i].pos);
  }
  if (state.showField) queueFieldCompute();
  var t = state.gainFrames[fi].t;
  var totalT = state.totalFrames * 0.02;
  if (timeLabel) timeLabel.textContent = t.toFixed(1) + 's / ' + totalT.toFixed(1) + 's';
  markDirty();
}

function queueFieldCompute() {
  state.fieldCache = null;
}

function computePotential(x, y, obstacles) {
  var dx, dy, d, rho, mag;
  var total = 0;
  var ka = parseFloat(state.gainKey);
  for (var i = 0; i < obstacles.length; i++) {
    var o = obstacles[i];
    dx = x - o.center[0];
    dy = y - o.center[1];
    d = Math.sqrt(dx * dx + dy * dy);
    rho = Math.max(d - o.radius, 0.05);
    if (rho < RHO0) {
      mag = 1.0 / rho - 1.0 / RHO0;
      total += ka * mag * mag;
    }
  }
  return total;
}

function computeField() {
  if (!state.showField) return;
  var obstacles = state.meta.obstacles;
  var gw = Math.ceil(width / GRID_RES);
  var gh = Math.ceil(height / GRID_RES);
  var grid = new Float32Array(gw * gh);
  var maxVal = 0;

  for (var gy = 0; gy < gh; gy++) {
    for (var gx = 0; gx < gw; gx++) {
      var cx = gx * GRID_RES + GRID_RES / 2;
      var cy = gy * GRID_RES + GRID_RES / 2;
      var wx = toWorldX(cx), wy = toWorldY(cy);
      if (wx < X_MIN || wx > X_MAX || wy < Y_MIN || wy > Y_MAX) {
        grid[gy * gw + gx] = -1;
        continue;
      }
      var v = computePotential(wx, wy, obstacles);
      grid[gy * gw + gx] = v;
      if (v > maxVal) maxVal = v;
    }
  }

  var offscreen = document.createElement('canvas');
  offscreen.width = gw;
  offscreen.height = gh;
  var octx = offscreen.getContext('2d');
  var imgData = octx.createImageData(gw, gh);
  var data = imgData.data;

  for (var py = 0; py < gh; py++) {
    for (var px = 0; px < gw; px++) {
      var idx = (py * gw + px) * 4;
      var v = grid[py * gw + px];
      if (v < 0) {
        data[idx] = 10; data[idx+1] = 14; data[idx+2] = 20; data[idx+3] = 0;
        continue;
      }
      var n = maxVal > 0 ? Math.min(v / maxVal, 1) : 0;
      if (n < 0.02) {
        data[idx] = 10; data[idx+1] = 14; data[idx+2] = 20; data[idx+3] = 0;
      } else {
        var r, g, b, a;
        if (n < 0.15) {
          var t = n / 0.15;
          r = Math.round(0);
          g = Math.round(40 + t * 40);
          b = Math.round(120 + t * 80);
          a = Math.round(t * 40);
        } else if (n < 0.35) {
          var t = (n - 0.15) / 0.2;
          r = Math.round(40 * t);
          g = Math.round(80 - t * 60);
          b = Math.round(200 - t * 60);
          a = Math.round(40 + t * 50);
        } else if (n < 0.6) {
          var t = (n - 0.35) / 0.25;
          r = Math.round(40 + t * 120);
          g = Math.round(20 - t * 20);
          b = Math.round(140 - t * 60);
          a = Math.round(90 + t * 50);
        } else {
          var t = (n - 0.6) / 0.4;
          r = Math.round(160 + t * 95);
          g = Math.round(0);
          b = Math.round(80 - t * 40);
          a = Math.round(140 + t * 60);
        }
        data[idx] = Math.min(r, 255);
        data[idx+1] = Math.max(g, 0);
        data[idx+2] = Math.max(b, 0);
        data[idx+3] = Math.min(a, 200);
      }
    }
  }

  octx.putImageData(imgData, 0, 0);
  state.fieldCache = offscreen;
  state.fieldGrid = { w: gw, h: gh };
}

function drawField() {
  if (!state.showField || !state.fieldCache) return;
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(state.fieldCache, 0, 0, width, height);
  ctx.imageSmoothingEnabled = true;
}

function drawCorridor() {
  ctx.setLineDash([6, 4]);
  ctx.strokeStyle = 'rgba(210,153,34,0.6)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(toCanvasX(X_MIN), toCanvasY(Y_MAX));
  ctx.lineTo(toCanvasX(X_MAX), toCanvasY(Y_MAX));
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(toCanvasX(X_MIN), toCanvasY(Y_MIN));
  ctx.lineTo(toCanvasX(X_MAX), toCanvasY(Y_MIN));
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawGates() {
  var gates = [-7, -2, 3, 8];
  var labels = ['G1','G2','G3','G4'];
  for (var i = 0; i < gates.length; i++) {
    var cx = toCanvasX(gates[i]);
    ctx.fillStyle = 'rgba(210,153,34,0.05)';
    ctx.fillRect(cx - 20 * scale, 0, 40 * scale, height);
    ctx.fillStyle = 'rgba(210,153,34,0.5)';
    ctx.font = Math.min(10, scale * 0.3) + 'px JetBrains Mono, monospace';
    ctx.textAlign = 'center';
    ctx.fillText(labels[i], cx, 12);
  }
}

function drawObstacles() {
  var obs = state.meta.obstacles;
  for (var i = 0; i < obs.length; i++) {
    var cx = toCanvasX(obs[i].center[0]);
    var cy = toCanvasY(obs[i].center[1]);
    var r = obs[i].radius * scale;
    ctx.beginPath();
    ctx.arc(cx, cy, Math.max(r, 4), 0, 2 * Math.PI);
    ctx.fillStyle = BG_COLOR;
    ctx.fill();
    ctx.fillStyle = 'rgba(255,255,255,0.12)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(255,255,255,0.5)';
    ctx.lineWidth = 0.8;
    ctx.stroke();
    ctx.fillStyle = 'rgba(255,255,255,0.6)';
    ctx.font = Math.min(8, scale * 0.25) + 'px JetBrains Mono, monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('O' + (i + 1), cx, cy);
  }
}

function drawRadii() {
  if (!state.showRadii) return;
  var obs = state.meta.obstacles;
  ctx.setLineDash([3, 4]);
  for (var i = 0; i < obs.length; i++) {
    var cx = toCanvasX(obs[i].center[0]);
    var cy = toCanvasY(obs[i].center[1]);
    var r = (obs[i].radius + RHO0) * scale;
    ctx.beginPath();
    ctx.arc(cx, cy, Math.max(r, 4), 0, 2 * Math.PI);
    ctx.strokeStyle = 'rgba(210,153,34,0.2)';
    ctx.lineWidth = 0.5;
    ctx.stroke();
  }
  ctx.setLineDash([]);
}

function drawStartGoal() {
  var sx = toCanvasX(-12), sy = toCanvasY(0);
  var gx = toCanvasX(12), gy = toCanvasY(0);
  var s = Math.max(6, scale * 0.2);
  ctx.fillStyle = '#3fb950';
  ctx.fillRect(sx - s/2, sy - s/2, s, s);
  ctx.fillRect(gx - s/2, gy - s/2, s, s);
  ctx.fillStyle = 'rgba(63,185,80,0.6)';
  ctx.font = Math.min(8, scale * 0.25) + 'px JetBrains Mono, monospace';
  ctx.textAlign = 'center';
  ctx.fillText('Start', sx, sy - s - 4);
  ctx.fillText('Goal', gx, gy - s - 4);
}

function drawTrail() {
  if (!state.showTrail || state.trail.length < 2) return;
  var rgb = GAIN_RGB[GAIN_KEYS.indexOf(state.gainKey)];
  for (var i = 1; i < state.trail.length; i++) {
    var alpha = i / state.trail.length;
    ctx.beginPath();
    ctx.moveTo(toCanvasX(state.trail[i-1][0]), toCanvasY(state.trail[i-1][1]));
    ctx.lineTo(toCanvasX(state.trail[i][0]), toCanvasY(state.trail[i][1]));
    ctx.strokeStyle = 'rgba(' + rgb[0] + ',' + rgb[1] + ',' + rgb[2] + ',' + (alpha * 0.7) + ')';
    ctx.lineWidth = Math.max(1, 2 * alpha);
    ctx.stroke();
  }
}

function drawDrone() {
  var frame = state.gainFrames[state.currentFrame];
  if (!frame) return;
  var cx = toCanvasX(frame.pos[0]), cy = toCanvasY(frame.pos[1]);
  var vel = frame.vel;
  var spd = frame.speed;
  var rgb = GAIN_RGB[GAIN_KEYS.indexOf(state.gainKey)];
  var color = GAIN_COLORS[GAIN_KEYS.indexOf(state.gainKey)];

  if (spd < 0.01) {
    ctx.beginPath();
    ctx.arc(cx, cy, 5, 0, 2 * Math.PI);
    ctx.fillStyle = color;
    ctx.fill();
    return;
  }

  var angle = Math.atan2(-vel[1], vel[0]);
  var sz = Math.max(6, scale * 0.25);
  var hw = sz * 0.45;
  var nose = sz;
  var tail = -sz * 0.4;

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(angle);

  ctx.beginPath();
  ctx.moveTo(nose * 1.25, 0);
  ctx.lineTo(tail * 1.25, -hw * 1.25);
  ctx.lineTo(tail * 1.25, hw * 1.25);
  ctx.closePath();
  ctx.fillStyle = 'rgba(' + rgb[0] + ',' + rgb[1] + ',' + rgb[2] + ',0.15)';
  ctx.fill();

  ctx.beginPath();
  ctx.moveTo(nose, 0);
  ctx.lineTo(tail, -hw);
  ctx.lineTo(tail, hw);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.shadowBlur = 8;
  ctx.shadowColor = color;
  ctx.fill();
  ctx.shadowBlur = 0;

  ctx.restore();
}

function drawForceVectors() {
  if (!state.showForces) return;
  var frame = state.gainFrames[state.currentFrame];
  if (!frame) return;
  var cx = toCanvasX(frame.pos[0]), cy = toCanvasY(frame.pos[1]);
  var maxF = Math.max(frame.F_att, frame.F_rep, frame.F_wall, 0.01);

  var forces = [
    { mag: frame.F_att, color: '#3fb950', label: 'F_att' },
    { mag: frame.F_rep, color: '#f85149', label: 'F_rep' },
    { mag: frame.F_wall, color: '#d29922', label: 'F_wall' },
  ];

  ctx.lineWidth = 1.2;
  for (var i = 0; i < forces.length; i++) {
    if (forces[i].mag < 0.01) continue;
    var len = Math.max(4, (forces[i].mag / maxF) * 60);
    var dir = [0, (i === 0 ? -1 : 1)];
    var ex = cx + dir[0] * len * 3;
    var ey = cy + dir[1] * len;

    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(ex, ey);
    ctx.strokeStyle = forces[i].color;
    ctx.stroke();

    var asz = 5;
    var ang = Math.atan2(ey - cy, ex - cx);
    ctx.beginPath();
    ctx.moveTo(ex, ey);
    ctx.lineTo(ex - asz * Math.cos(ang - 0.5), ey - asz * Math.sin(ang - 0.5));
    ctx.lineTo(ex - asz * Math.cos(ang + 0.5), ey - asz * Math.sin(ang + 0.5));
    ctx.closePath();
    ctx.fillStyle = forces[i].color;
    ctx.fill();
  }
}

function drawHUD() {
  var frame = state.gainFrames[state.currentFrame];
  if (!frame) return;
  var t = frame.t;
  var spd = frame.speed;
  var px = frame.pos[0], py = frame.pos[1];
  var totalT = state.totalFrames * 0.02;
  var ka = state.gainKey;

  var lines = [
    '> t = ' + t.toFixed(1) + 's / ' + totalT.toFixed(1) + 's',
    '> pos = (' + px.toFixed(1) + ', ' + py.toFixed(1) + ') m',
    '> speed = ' + spd.toFixed(2) + ' m/s',
    '> k_avoid = ' + ka,
  ];

  if (state.showForces) {
    lines.push('> F_att = ' + frame.F_att.toFixed(2) + '  F_rep = ' + frame.F_rep.toFixed(2) + '  F_wall = ' + frame.F_wall.toFixed(2));
  }

  ctx.font = '11px JetBrains Mono, monospace';
  ctx.textBaseline = 'top';
  for (var i = 0; i < lines.length; i++) {
    ctx.fillStyle = 'rgba(0,0,0,0.6)';
    ctx.fillRect(6, height - 16 * (lines.length - i) - 4, ctx.measureText(lines[i]).width + 12, 16);
    ctx.fillStyle = '#39d353';
    ctx.fillText(lines[i], 12, height - 16 * (lines.length - i));
  }
}

function renderFrame() {
  ctx.fillStyle = BG_COLOR;
  ctx.fillRect(0, 0, width, height);
  drawField();
  drawCorridor();
  drawGates();
  drawObstacles();
  drawRadii();
  drawStartGoal();
  drawTrail();
  drawDrone();
  drawForceVectors();
  drawHUD();
}

function render() {
  if (state.fieldCache === null && state.showField) {
    computeField();
  }
  renderFrame();
}

function renderLoop() {
  requestAnimationFrame(renderLoop);
  if (!state.dirty) return;
  state.dirty = false;
  if (!state.gainFrames || width === 0) return;
  render();
}

function gainSelect(key) {
  if (key === state.gainKey) return;
  state.gainKey = key;
  state.gainFrames = state.data.gain_sweep[key].frames;
  state.totalFrames = state.gainFrames.length;
  state.currentFrame = 0;
  state.trail = [];
  state.fieldCache = null;

  slider.max = state.totalFrames - 1;
  slider.value = 0;

  for (var k in gainChips) {
    gainChips[k].style.borderColor = k === key ? '#ffffff' : 'transparent';
    gainChips[k].style.boxShadow = k === key ? '0 0 6px rgba(255,255,255,0.5)' : 'none';
  }

  if (state.isPlaying) {
    clearInterval(state.playInterval);
    state.isPlaying = false;
    playBtn.textContent = '\u25B6';
  }

  updateFrame(0);
}

function togglePlay() {
  if (!state.gainFrames) return;
  if (state.isPlaying) {
    clearInterval(state.playInterval);
    state.isPlaying = false;
    playBtn.textContent = '\u25B6';
  } else {
    if (state.currentFrame >= state.totalFrames - 1) {
      updateFrame(0);
    }
    state.isPlaying = true;
    playBtn.textContent = '\u25A0';
    state.playInterval = setInterval(function() {
      var next = state.currentFrame + 1;
      if (next >= state.totalFrames) {
        clearInterval(state.playInterval);
        state.isPlaying = false;
        playBtn.textContent = '\u25B6';
        return;
      }
      slider.value = next;
      updateFrame(next);
    }, 60 / state.speed);
  }
}

function setSpeed(s) {
  state.speed = s;
  if (state.isPlaying) {
    clearInterval(state.playInterval);
    state.isPlaying = false;
    playBtn.textContent = '\u25B6';
    togglePlay();
  }
}

function buildControls() {
  var ctrlDiv = document.createElement('div');
  ctrlDiv.className = 'sim-controls';
  ctrlDiv.style.flexDirection = 'column';
  ctrlDiv.style.alignItems = 'stretch';
  ctrlDiv.style.padding = '8px 12px';
  ctrlDiv.style.gap = '6px';

  var row1 = document.createElement('div');
  row1.style.display = 'flex';
  row1.style.alignItems = 'center';
  row1.style.gap = '8px';

  playBtn = document.createElement('button');
  playBtn.textContent = '\u25B6';
  playBtn.style.fontSize = '14px';
  playBtn.style.width = '32px';
  playBtn.style.textAlign = 'center';
  playBtn.onclick = togglePlay;
  row1.appendChild(playBtn);

  slider = document.createElement('input');
  slider.type = 'range';
  slider.min = 0;
  slider.max = state.totalFrames - 1;
  slider.value = 0;
  slider.style.flex = '1';
  slider.style.accentColor = '#3fb950';
  slider.style.height = '4px';
  slider.oninput = function() {
    if (state.isPlaying) {
      clearInterval(state.playInterval);
      state.isPlaying = false;
      playBtn.textContent = '\u25B6';
    }
    updateFrame(parseInt(slider.value));
  };
  row1.appendChild(slider);

  timeLabel = document.createElement('span');
  timeLabel.style.fontSize = '11px';
  timeLabel.style.color = '#8b949e';
  timeLabel.style.minWidth = '100px';
  timeLabel.style.textAlign = 'center';
  row1.appendChild(timeLabel);

  var speeds = [1, 2, 4];
  for (var si = 0; si < speeds.length; si++) {
    (function(s) {
      var btn = document.createElement('button');
      btn.textContent = s + 'x';
      btn.style.fontSize = '10px';
      btn.style.padding = '2px 6px';
      btn.style.border = '1px solid ' + (s === 1 ? '#3fb950' : '#21262d');
      btn.style.color = s === 1 ? '#3fb950' : '#8b949e';
      btn.onclick = function() { setSpeed(s); };
      row1.appendChild(btn);
    })(speeds[si]);
  }

  ctrlDiv.appendChild(row1);

  var row2 = document.createElement('div');
  row2.className = 'player-gain-chips';
  row2.style.display = 'flex';
  row2.style.flexWrap = 'wrap';
  row2.style.gap = '4px';

  for (var gi = 0; gi < GAIN_KEYS.length; gi++) {
    (function(key, idx) {
      var chip = document.createElement('button');
      chip.textContent = key;
      chip.className = 'player-gain-chip';
      chip.style.backgroundColor = GAIN_COLORS[idx] + '33';
      chip.style.borderColor = key === '4' ? '#ffffff' : 'transparent';
      chip.style.boxShadow = key === '4' ? '0 0 6px rgba(255,255,255,0.5)' : 'none';
      chip.onclick = function() { gainSelect(key); };
      row2.appendChild(chip);
      gainChips[key] = chip;
    })(GAIN_KEYS[gi], gi);
  }

  ctrlDiv.appendChild(row2);

  var row3 = document.createElement('div');
  row3.className = 'player-check-row';
  row3.style.display = 'flex';
  row3.style.flexWrap = 'wrap';
  row3.style.gap = '14px';
  row3.style.padding = '2px 0';

  var toggles = [
    { key: 'showForces', label: 'Force Vectors' },
    { key: 'showField', label: 'Potential Field' },
    { key: 'showTrail', label: 'Trail' },
    { key: 'showRadii', label: 'Influence Radii' },
  ];

  for (var ti = 0; ti < toggles.length; ti++) {
    (function(t) {
      var label = document.createElement('label');
      label.style.cssText = 'font-size:10px;color:#8b949e;cursor:pointer;display:flex;align-items:center;gap:4px;font-family:JetBrains Mono,monospace';

      var cb = document.createElement('input');
      cb.type = 'checkbox';
      cb.checked = state[t.key];
      cb.style.accentColor = '#3fb950';

      cb.onchange = function() {
        state[t.key] = cb.checked;
        if (t.key === 'showField') state.fieldCache = null;
        markDirty();
      };

      label.appendChild(cb);
      label.appendChild(document.createTextNode(t.label));
      row3.appendChild(label);
    })(toggles[ti]);
  }

  ctrlDiv.appendChild(row3);

  container.parentNode.insertBefore(ctrlDiv, container.nextSibling);
}

function buildHUD() {
  var hud = document.createElement('div');
  hud.className = 'sim-hud';
  hud.style.pointerEvents = 'none';
  container.style.position = 'relative';
  container.appendChild(hud);
}

function initPlayer2D(containerId, simData) {
  container = document.getElementById(containerId);
  if (!container) return;

  state.data = simData;
  state.meta = simData.meta;
  state.gainKey = '4';
  state.gainFrames = simData.gain_sweep['4'].frames;
  state.totalFrames = state.gainFrames.length;

  canvas = document.createElement('canvas');
  canvas.style.display = 'block';
  canvas.style.width = '100%';
  canvas.style.height = '100%';
  container.appendChild(canvas);
  ctx = canvas.getContext('2d');

  buildControls();
  buildHUD();
  resize();
  updateFrame(0);
  renderLoop();

  window.addEventListener('resize', function() {
    resize();
    markDirty();
  });
}

global.initPlayer2D = initPlayer2D;

})(window);
