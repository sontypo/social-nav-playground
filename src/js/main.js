// SocialNav Studio Main Controller & UI Bindings

import { simTheoryData } from './data.js';
import { SocialNavSimulator } from './simulator.js';
import { initInteractiveTerminal } from './terminal.js';
import { ros2BridgeInstance } from './ros2Bridge.js';
import { telemetryAnalytics } from './analyticsCharts.js';
import { gazeboExporterInstance } from './gazeboExporter.js';

let simulator = null;

document.addEventListener('DOMContentLoaded', () => {
  window.scrollTo(0, 0);
  initRetroBootScreen();
  initBackgroundCanvas();
  initMouseGlow();
  initNavbarBrand();
  initSimulatorSuite();
  telemetryAnalytics.init();
  initROS2BridgeControls();
  initInteractiveTerminal(() => simulator);
  initTheoryModal();
  initThemeSwitcher();
});

// 0. Retro Minimalist Boot Sequence
function initRetroBootScreen() {
  window.scrollTo(0, 0);
  const bootScreen = document.getElementById('retro-boot-screen');
  if (!bootScreen) return;

  const bar = document.getElementById('boot-progress-bar');
  const percentText = document.getElementById('boot-percent-text');
  const statusText = document.getElementById('boot-status-text');

  const steps = [
    { text: 'LOADING PROXEMICS ENGINE...', pct: 35 },
    { text: 'CALIBRATING 360° LIDAR...', pct: 70 },
    { text: 'MOUNTING ROS2 BRIDGE...', pct: 90 },
    { text: 'READY', pct: 100 }
  ];

  let currentStep = 0;
  let isDone = false;

  function dismissBoot() {
    if (isDone) return;
    isDone = true;
    window.scrollTo(0, 0);
    bootScreen.classList.add('fade-out');
    setTimeout(() => {
      bootScreen.remove();
    }, 450);
  }

  bootScreen.addEventListener('click', dismissBoot);
  window.addEventListener('keydown', dismissBoot, { once: true });

  const interval = setInterval(() => {
    if (isDone) {
      clearInterval(interval);
      return;
    }

    if (currentStep < steps.length) {
      const s = steps[currentStep];
      if (bar) bar.style.width = `${s.pct}%`;
      if (percentText) percentText.textContent = `${s.pct}%`;
      if (statusText) statusText.textContent = s.text;
      currentStep++;
    } else {
      clearInterval(interval);
      setTimeout(dismissBoot, 220);
    }
  }, 200);
}

// 1. Interactive Subtle Background Particles
function initBackgroundCanvas() {
  const canvas = document.getElementById('bg-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  let width = (canvas.width = window.innerWidth);
  let height = (canvas.height = window.innerHeight);

  window.addEventListener('resize', () => {
    width = canvas.width = window.innerWidth;
    height = canvas.height = window.innerHeight;
  });

  const particles = Array.from({ length: 45 }, () => ({
    x: Math.random() * width,
    y: Math.random() * height,
    vx: (Math.random() - 0.5) * 0.35,
    vy: (Math.random() - 0.5) * 0.35,
    size: Math.random() * 1.8 + 0.6,
    alpha: Math.random() * 0.35 + 0.08
  }));

  function animate() {
    ctx.clearRect(0, 0, width, height);

    for (let p of particles) {
      p.x += p.vx;
      p.y += p.vy;

      if (p.x < 0) p.x = width;
      if (p.x > width) p.x = 0;
      if (p.y < 0) p.y = height;
      if (p.y > height) p.y = 0;

      ctx.beginPath();
      ctx.arc(p.x, p.y, p.size, 0, Math.PI * 2);
      ctx.fillStyle = `rgba(0, 255, 157, ${p.alpha})`;
      ctx.fill();
    }

    requestAnimationFrame(animate);
  }

  animate();
}

// 2. Mouse Tracking Glow
function initMouseGlow() {
  const glow = document.querySelector('.mouse-glow-light');
  if (!glow) return;

  window.addEventListener('mousemove', (e) => {
    glow.style.left = `${e.clientX}px`;
    glow.style.top = `${e.clientY}px`;
  });
}

// 3. Simulator Controls & Tool Palette Binding
function initSimulatorSuite() {
  simulator = new SocialNavSimulator('sim-canvas');

  // Algorithm Selector Tabs
  const algoTabs = document.querySelectorAll('#algo-selection-tabs .algo-tab-btn');
  algoTabs.forEach(tab => {
    tab.addEventListener('click', () => {
      algoTabs.forEach(t => t.classList.remove('active'));
      tab.classList.add('active');
      const algo = tab.getAttribute('data-algo');
      simulator.setAlgorithm(algo);
      showToast(`Planner algorithm: ${tab.textContent.trim()}`);
    });
  });

  // Scenario Selector Dropdown
  const datasetSelect = document.getElementById('dataset-scenario-select');
  datasetSelect?.addEventListener('change', (e) => {
    const scenario = e.target.value;
    simulator.loadScenario(scenario);
    showToast(`Loaded scenario: ${e.target.options[e.target.selectedIndex].text}`);
  });

  // Upload Custom Trajectory File
  const btnUpload = document.getElementById('btn-upload-dataset');
  const fileInput = document.getElementById('input-dataset-file');
  btnUpload?.addEventListener('click', () => fileInput?.click());
  fileInput?.addEventListener('change', (e) => {
    const file = e.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (event) => {
      const res = simulator.loadCustomDataset(event.target.result);
      if (res.success) {
        showToast(`Loaded custom trajectory dataset (${res.pedestrianCount} agents)`);
      } else {
        showToast(`Dataset format error: ${res.error}`);
      }
    };
    reader.readAsText(file);
  });

  // Interactive Tool Palette
  const toolBtns = document.querySelectorAll('#sim-tool-palette .tool-btn[data-tool]');
  const toolHint = document.getElementById('active-tool-hint');

  const toolHints = {
    drag: 'Active Tool: Drag / Select • Drag pillars, boxes, polygons, waypoints, robot, or goal flag',
    rotate: 'Active Tool: Rotate Entity • Drag around robot, human, or obstacle to rotate orientation',
    spawn_ped: 'Active Tool: Spawn Human • Click anywhere on canvas to create moving pedestrian',
    add_object: 'Active Tool: Add Object • Click canvas to place Pillar, Box barrier, or Random Polygon',
    set_goal: 'Active Tool: Set Goal • Click canvas to relocate single goal or append sequential waypoints',
    set_robot: 'Active Tool: Set Robot • Click canvas to relocate AMR robot spawn',
    delete: 'Active Tool: Delete Entity • Click on any human pedestrian, obstacle, or waypoint to remove it'
  };

  simulator.onEntityDeleted = (entity) => {
    if (entity.type === 'pedestrian') {
      showToast('Deleted pedestrian from simulation world');
    } else {
      showToast(`Deleted obstacle: ${entity.label}`);
    }
  };

  simulator.onObjectPlaced = (obs) => {
    showToast(`Placed ${obs.label} into simulation world`);
  };

  // Add Object Subpanel Interaction
  const tabObjCircle = document.getElementById('tab-obj-circle');
  const tabObjRect = document.getElementById('tab-obj-rect');
  const tabObjPoly = document.getElementById('tab-obj-poly');
  const tabObjDrawPoly = document.getElementById('tab-obj-draw-poly');
  const subpanelPillarInputs = document.getElementById('subpanel-pillar-inputs');
  const subpanelBoxInputs = document.getElementById('subpanel-box-inputs');
  const subpanelPolyInputs = document.getElementById('subpanel-poly-inputs');
  const subpanelDrawPolyInputs = document.getElementById('subpanel-draw-poly-inputs');
  const subpanelPolyVertexCount = document.getElementById('subpanel-poly-vertex-count');
  const btnDrawPolyClear = document.getElementById('btn-draw-poly-clear');
  const btnDrawPolyFinish = document.getElementById('btn-draw-poly-finish');
  const inputObjRadius = document.getElementById('input-obj-radius');
  const inputObjWidth = document.getElementById('input-obj-width');
  const inputObjHeight = document.getElementById('input-obj-height');
  const btnToolAddObject = document.getElementById('btn-tool-add-object');

  const updateObjectConfig = () => {
    let type = 'circle';
    if (tabObjRect?.classList.contains('active')) type = 'rect';
    else if (tabObjPoly?.classList.contains('active')) type = 'poly';
    else if (tabObjDrawPoly?.classList.contains('active')) type = 'draw_poly';

    const radius = Math.max(5, parseFloat(inputObjRadius?.value) || 22);
    const width = Math.max(10, parseFloat(inputObjWidth?.value) || 60);
    const height = Math.max(10, parseFloat(inputObjHeight?.value) || 30);

    simulator.setCustomObjectConfig({
      type,
      radius,
      width,
      height
    });
  };

  tabObjCircle?.addEventListener('click', (e) => {
    e.stopPropagation();
    tabObjCircle.classList.add('active');
    tabObjRect?.classList.remove('active');
    tabObjPoly?.classList.remove('active');
    tabObjDrawPoly?.classList.remove('active');
    if (subpanelPillarInputs) subpanelPillarInputs.style.display = 'flex';
    if (subpanelBoxInputs) subpanelBoxInputs.style.display = 'none';
    if (subpanelPolyInputs) subpanelPolyInputs.style.display = 'none';
    if (subpanelDrawPolyInputs) subpanelDrawPolyInputs.style.display = 'none';
    btnToolAddObject?.click();
    updateObjectConfig();
    showToast('Add Object: Pillar Cylinder mode');
  });

  tabObjRect?.addEventListener('click', (e) => {
    e.stopPropagation();
    tabObjRect.classList.add('active');
    tabObjCircle?.classList.remove('active');
    tabObjPoly?.classList.remove('active');
    tabObjDrawPoly?.classList.remove('active');
    if (subpanelPillarInputs) subpanelPillarInputs.style.display = 'none';
    if (subpanelBoxInputs) subpanelBoxInputs.style.display = 'flex';
    if (subpanelPolyInputs) subpanelPolyInputs.style.display = 'none';
    if (subpanelDrawPolyInputs) subpanelDrawPolyInputs.style.display = 'none';
    btnToolAddObject?.click();
    updateObjectConfig();
    showToast('Add Object: Box Barrier mode');
  });

  tabObjPoly?.addEventListener('click', (e) => {
    e.stopPropagation();
    tabObjPoly.classList.add('active');
    tabObjCircle?.classList.remove('active');
    tabObjRect?.classList.remove('active');
    tabObjDrawPoly?.classList.remove('active');
    if (subpanelPillarInputs) subpanelPillarInputs.style.display = 'none';
    if (subpanelBoxInputs) subpanelBoxInputs.style.display = 'none';
    if (subpanelPolyInputs) subpanelPolyInputs.style.display = 'flex';
    if (subpanelDrawPolyInputs) subpanelDrawPolyInputs.style.display = 'none';
    btnToolAddObject?.click();
    updateObjectConfig();
    showToast('Add Object: Random Geometric Polygon mode');
  });

  tabObjDrawPoly?.addEventListener('click', (e) => {
    e.stopPropagation();
    tabObjDrawPoly.classList.add('active');
    tabObjCircle?.classList.remove('active');
    tabObjRect?.classList.remove('active');
    tabObjPoly?.classList.remove('active');
    if (subpanelPillarInputs) subpanelPillarInputs.style.display = 'none';
    if (subpanelBoxInputs) subpanelBoxInputs.style.display = 'none';
    if (subpanelPolyInputs) subpanelPolyInputs.style.display = 'none';
    if (subpanelDrawPolyInputs) subpanelDrawPolyInputs.style.display = 'flex';
    btnToolAddObject?.click();
    updateObjectConfig();
    showToast('Add Object: Draw Custom Boundary mode (Click canvas to add vertices)');
  });

  btnDrawPolyClear?.addEventListener('click', (e) => {
    e.stopPropagation();
    simulator.clearCustomPolygonDraft();
    showToast('Cleared polygon draft points');
  });

  btnDrawPolyFinish?.addEventListener('click', (e) => {
    e.stopPropagation();
    if (simulator.customPolygonDraftPoints.length < 3) {
      showToast('⚠️ Click at least 3 vertices on canvas before finishing');
      return;
    }
    const obs = simulator.finishCustomPolygonDrawing();
    if (obs) {
      showToast(`✅ Created custom polygon with ${obs.points.length} vertices`);
    }
  });

  simulator.onDraftPointsUpdated = (count) => {
    if (subpanelPolyVertexCount) {
      subpanelPolyVertexCount.textContent = `${count} points`;
    }
  };

  window.addEventListener('keydown', (e) => {
    if (simulator.activeTool === 'add_object' && simulator.customObjectConfig.type === 'draw_poly') {
      if (e.key === 'Enter' || e.key === ' ') {
        if (simulator.customPolygonDraftPoints.length >= 3) {
          e.preventDefault();
          const obs = simulator.finishCustomPolygonDrawing();
          if (obs) {
            showToast(`✅ Created custom polygon with ${obs.points.length} vertices`);
          }
        }
      } else if (e.key === 'Escape') {
        simulator.clearCustomPolygonDraft();
        showToast('Cancelled polygon drawing');
      }
    }
  });

  inputObjRadius?.addEventListener('input', updateObjectConfig);
  inputObjWidth?.addEventListener('input', updateObjectConfig);
  inputObjHeight?.addEventListener('input', updateObjectConfig);

  const btnObjClose = document.getElementById('btn-obj-subpanel-close');
  const btnObjCancel = document.getElementById('btn-obj-cancel');
  const btnObjConfirm = document.getElementById('btn-obj-confirm');
  const btnToolDrag = document.querySelector('#sim-tool-palette .tool-btn[data-tool="drag"]');

  const closeSubpanelAndCancel = () => {
    simulator.clearCustomPolygonDraft();
    if (addObjSubpanel) addObjSubpanel.classList.remove('visible');
    if (btnToolDrag) {
      btnToolDrag.click();
    }
  };

  btnObjClose?.addEventListener('click', (e) => {
    e.stopPropagation();
    closeSubpanelAndCancel();
    showToast('Add Object cancelled');
  });

  btnObjCancel?.addEventListener('click', (e) => {
    e.stopPropagation();
    closeSubpanelAndCancel();
    showToast('Add Object cancelled');
  });

  btnObjConfirm?.addEventListener('click', (e) => {
    e.stopPropagation();
    updateObjectConfig();
    let label = 'Pillar';
    if (tabObjRect?.classList.contains('active')) label = 'Box Barrier';
    else if (tabObjPoly?.classList.contains('active')) label = 'Random Geometric Polygon';
    else if (tabObjDrawPoly?.classList.contains('active')) {
      if (simulator.customPolygonDraftPoints.length >= 3) {
        simulator.finishCustomPolygonDrawing();
        if (addObjSubpanel) addObjSubpanel.classList.remove('visible');
        return;
      }
      label = 'Custom Polygon (Click canvas to add vertices)';
    }
    if (addObjSubpanel) addObjSubpanel.classList.remove('visible');
    showToast(`Ready: ${label}`);
  });

  const addObjSubpanel = document.getElementById('add-object-subpanel');
  const setGoalSubpanel = document.getElementById('set-goal-subpanel');
  const tabGoalSingle = document.getElementById('tab-goal-single');
  const tabGoalMulti = document.getElementById('tab-goal-multi');
  const subpanelSingleGoal = document.getElementById('subpanel-single-goal-inputs');
  const subpanelMultiGoal = document.getElementById('subpanel-multi-goal-inputs');
  const subpanelWpCount = document.getElementById('subpanel-wp-count');
  const chkWpLoop = document.getElementById('chk-wp-loop');
  const btnWpClear = document.getElementById('btn-wp-clear');
  const btnWpResetActive = document.getElementById('btn-wp-reset-active');
  const btnGoalClose = document.getElementById('btn-goal-subpanel-close');
  const btnGoalCancel = document.getElementById('btn-goal-cancel');
  const btnGoalConfirm = document.getElementById('btn-goal-confirm');

  const updateGoalSubpanelUI = () => {
    if (subpanelWpCount) {
      subpanelWpCount.textContent = `${simulator.waypoints.length} points`;
    }
  };

  simulator.onGoalUpdated = () => {
    updateGoalSubpanelUI();
  };

  simulator.onWaypointReached = (cur, next, total, isLooped) => {
    if (isLooped) {
      showToast(`🏁 Completed patrol loop! Returning to WP #1 (Total: ${total} points)`);
    } else {
      showToast(`🎯 Reached WP #${cur} → Heading to WP #${next} (Total: ${total} points)`);
    }
  };

  tabGoalSingle?.addEventListener('click', (e) => {
    e.stopPropagation();
    tabGoalSingle.classList.add('active');
    tabGoalMulti?.classList.remove('active');
    if (subpanelSingleGoal) subpanelSingleGoal.style.display = 'flex';
    if (subpanelMultiGoal) subpanelMultiGoal.style.display = 'none';
    simulator.setGoalMode('single');
    showToast('Goal Mode: Single Target Flag');
  });

  tabGoalMulti?.addEventListener('click', (e) => {
    e.stopPropagation();
    tabGoalMulti.classList.add('active');
    tabGoalSingle?.classList.remove('active');
    if (subpanelSingleGoal) subpanelSingleGoal.style.display = 'none';
    if (subpanelMultiGoal) subpanelMultiGoal.style.display = 'flex';
    simulator.setGoalMode('multi');
    updateGoalSubpanelUI();
    showToast('Goal Mode: Multi-Waypoint Sequential Patrol');
  });

  chkWpLoop?.addEventListener('change', () => {
    simulator.setWaypointLoop(chkWpLoop.checked);
    showToast(`Waypoint Loop Patrol: ${chkWpLoop.checked ? 'ENABLED (Continuous)' : 'DISABLED (Stop at end)'}`);
  });

  btnWpClear?.addEventListener('click', (e) => {
    e.stopPropagation();
    simulator.clearWaypoints();
    updateGoalSubpanelUI();
    showToast('Cleared all waypoints from patrol sequence');
  });

  btnWpResetActive?.addEventListener('click', (e) => {
    e.stopPropagation();
    simulator.resetActiveWaypoint(0);
    showToast('Reset target to Waypoint #1');
  });

  const closeGoalSubpanelAndCancel = () => {
    if (setGoalSubpanel) setGoalSubpanel.classList.remove('visible');
    if (btnToolDrag) {
      btnToolDrag.click();
    }
  };

  btnGoalClose?.addEventListener('click', (e) => {
    e.stopPropagation();
    closeGoalSubpanelAndCancel();
    showToast('Set Goal cancelled');
  });

  btnGoalCancel?.addEventListener('click', (e) => {
    e.stopPropagation();
    closeGoalSubpanelAndCancel();
    showToast('Set Goal cancelled');
  });

  btnGoalConfirm?.addEventListener('click', (e) => {
    e.stopPropagation();
    if (setGoalSubpanel) setGoalSubpanel.classList.remove('visible');
    const isMulti = simulator.goalMode === 'multi';
    showToast(`Ready: Click canvas to ${isMulti ? 'append waypoints to sequence' : 'relocate single goal'}`);
  });

  toolBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      toolBtns.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      const tool = btn.getAttribute('data-tool');
      simulator.setActiveTool(tool);

      // Show subpanel ONLY when Add Object or Set Goal tool is selected
      if (addObjSubpanel) {
        if (tool === 'add_object') {
          addObjSubpanel.classList.add('visible');
        } else {
          addObjSubpanel.classList.remove('visible');
        }
      }

      if (setGoalSubpanel) {
        if (tool === 'set_goal') {
          setGoalSubpanel.classList.add('visible');
          updateGoalSubpanelUI();
        } else {
          setGoalSubpanel.classList.remove('visible');
        }
      }

      if (toolHint && toolHints[tool]) {
        toolHint.textContent = toolHints[tool];
      }
      showToast(`Tool active: ${btn.querySelector('span')?.textContent || tool}`);
    });
  });

  // Toolbar Toggle Buttons
  const btnCostmap = document.getElementById('btn-toggle-costmap');
  btnCostmap?.addEventListener('click', () => {
    const state = simulator.toggleCostmap();
    btnCostmap.classList.toggle('active', state);
    showToast(`Social Costmap Layer: ${state ? 'ENABLED (nav_msgs/OccupancyGrid)' : 'DISABLED'}`);
  });

  const btnLidar = document.getElementById('btn-toggle-lidar');
  btnLidar?.addEventListener('click', () => {
    const state = simulator.toggleLidar();
    btnLidar.classList.toggle('active', state);
    showToast(`LiDAR 360° Raycasting: ${state ? 'ENABLED' : 'DISABLED'}`);
  });

  const btnHeatmap = document.getElementById('btn-toggle-heatmap');
  btnHeatmap?.addEventListener('click', () => {
    const state = simulator.toggleHeatmap();
    btnHeatmap.classList.toggle('active', state);
    showToast(`Hall's Proxemics Heatmap: ${state ? 'ENABLED' : 'DISABLED'}`);
  });

  const btnVectors = document.getElementById('btn-toggle-vectors');
  btnVectors?.addEventListener('click', () => {
    const state = simulator.toggleVectors();
    btnVectors.classList.toggle('active', state);
    showToast(`Velocity & Force Vectors: ${state ? 'ENABLED' : 'DISABLED'}`);
  });

  const btnPause = document.getElementById('btn-sim-pause');
  btnPause?.addEventListener('click', () => {
    const isPaused = simulator.togglePause();
    if (isPaused) {
      btnPause.innerHTML = `<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="5 3 19 12 5 21 5 3"></polygon></svg> <span>Resume Loop</span>`;
      btnPause.classList.add('active');
      showToast('Simulation PAUSED');
    } else {
      btnPause.innerHTML = `<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="6" y="4" width="4" height="16"></rect><rect x="14" y="4" width="4" height="16"></rect></svg> <span>Pause Loop</span>`;
      btnPause.classList.remove('active');
      showToast('Simulation RESUMED');
    }
  });

  const btnReset = document.getElementById('btn-sim-reset');
  btnReset?.addEventListener('click', () => {
    simulator.reset();
    showToast('Simulation world reset');
  });

  // Export Gazebo 3D World Buttons
  const btnExportClassic = document.getElementById('btn-export-gazebo-classic');
  btnExportClassic?.addEventListener('click', () => {
    if (!simulator) return;
    const res = gazeboExporterInstance.downloadWorldFile(simulator, 'classic');
    showToast(`📦 Exported Gazebo Classic 11 World (${res.filename})`);
  });

  const btnExportIgn = document.getElementById('btn-export-gazebo-ign');
  btnExportIgn?.addEventListener('click', () => {
    if (!simulator) return;
    const res = gazeboExporterInstance.downloadWorldFile(simulator, 'ignition');
    showToast(`📦 Exported Ignition Gazebo SDF (${res.filename})`);
  });

  // Parameter Sliders
  const sliderHumans = document.getElementById('slider-pedestrians');
  const labelHumans = document.getElementById('val-pedestrians');
  sliderHumans?.addEventListener('input', (e) => {
    const val = parseInt(e.target.value);
    if (labelHumans) labelHumans.textContent = val;
    simulator.setPedestrianCount(val);
  });

  const sliderSpeed = document.getElementById('slider-speed');
  const labelSpeed = document.getElementById('val-speed');
  sliderSpeed?.addEventListener('input', (e) => {
    const val = parseFloat(e.target.value);
    if (labelSpeed) labelSpeed.textContent = `${val.toFixed(1)} m/s`;
    simulator.setRobotSpeed(val);
  });

  const sliderCourtesy = document.getElementById('slider-courtesy');
  const labelCourtesy = document.getElementById('val-courtesy');
  sliderCourtesy?.addEventListener('input', (e) => {
    const val = parseFloat(e.target.value);
    if (labelCourtesy) labelCourtesy.textContent = val.toFixed(1);
    simulator.setCourtesyWeight(val);
  });

  const sliderProxemics = document.getElementById('slider-proxemics');
  const labelProxemics = document.getElementById('val-proxemics');
  sliderProxemics?.addEventListener('input', (e) => {
    const val = parseFloat(e.target.value);
    if (labelProxemics) labelProxemics.textContent = `${val.toFixed(1)} m`;
    simulator.proxemicRadius = val;
  });

  // -----------------------------------------------------------
  // LiDAR Sensor Customization Controls (Rays, Range, FoV, Beams/Hits)
  // -----------------------------------------------------------
  const popoverLidar = document.getElementById('lidar-settings-popover');
  const btnLidarSettings = document.getElementById('btn-lidar-settings');
  const btnSyncLidarPopover = document.getElementById('btn-sync-lidar-popover');
  const btnCloseLidarPopover = document.getElementById('btn-close-lidar-popover');

  const sliderLidarRays = document.getElementById('slider-lidar-rays');
  const labelLidarRays = document.getElementById('val-lidar-rays');
  const popSliderLidarRays = document.getElementById('pop-slider-lidar-rays');
  const popLabelLidarRays = document.getElementById('pop-val-lidar-rays');

  const popSliderLidarRange = document.getElementById('pop-slider-lidar-range');
  const popLabelLidarRange = document.getElementById('pop-val-lidar-range');
  const labelLidarRange = document.getElementById('val-lidar-range');
  const labelLidarFov = document.getElementById('val-lidar-fov');

  const fovBtns = document.querySelectorAll('.lidar-fov-btn');
  const chkLidarRays = document.getElementById('pop-chk-lidar-rays');
  const chkLidarPoints = document.getElementById('pop-chk-lidar-points');

  function updateLidarRays(rays) {
    const r = simulator.setLidarRays(rays);
    if (labelLidarRays) labelLidarRays.textContent = `${r} rays`;
    if (popLabelLidarRays) popLabelLidarRays.textContent = `${r} rays`;
    if (sliderLidarRays) sliderLidarRays.value = r;
    if (popSliderLidarRays) popSliderLidarRays.value = r;
  }

  function updateLidarRange(rangeM) {
    const rng = simulator.setLidarMaxRange(rangeM);
    if (labelLidarRange) labelLidarRange.textContent = `${rng.toFixed(1)} m`;
    if (popLabelLidarRange) popLabelLidarRange.textContent = `${rng.toFixed(1)} m`;
    if (popSliderLidarRange) popSliderLidarRange.value = rng;
  }

  function updateLidarFov(fovDeg) {
    const fov = simulator.setLidarFov(fovDeg);
    if (labelLidarFov) labelLidarFov.textContent = `${fov}°`;
    fovBtns.forEach(btn => {
      if (parseInt(btn.getAttribute('data-fov')) === fov) {
        btn.classList.add('active');
      } else {
        btn.classList.remove('active');
      }
    });
  }

  sliderLidarRays?.addEventListener('input', (e) => {
    updateLidarRays(parseInt(e.target.value));
  });

  popSliderLidarRays?.addEventListener('input', (e) => {
    updateLidarRays(parseInt(e.target.value));
  });

  popSliderLidarRange?.addEventListener('input', (e) => {
    updateLidarRange(parseFloat(e.target.value));
  });

  fovBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      const fov = parseInt(btn.getAttribute('data-fov'));
      updateLidarFov(fov);
      showToast(`LiDAR Field of View: ${fov}°`);
    });
  });

  chkLidarRays?.addEventListener('change', (e) => {
    simulator.toggleLidarRays(e.target.checked);
  });

  chkLidarPoints?.addEventListener('change', (e) => {
    simulator.toggleLidarPoints(e.target.checked);
  });

  btnLidarSettings?.addEventListener('click', (e) => {
    e.stopPropagation();
    popoverLidar?.classList.toggle('open');
  });

  btnSyncLidarPopover?.addEventListener('click', (e) => {
    e.stopPropagation();
    popoverLidar?.classList.toggle('open');
  });

  btnCloseLidarPopover?.addEventListener('click', () => {
    popoverLidar?.classList.remove('open');
  });

  document.addEventListener('click', (e) => {
    if (popoverLidar?.classList.contains('open') && !popoverLidar.contains(e.target) && e.target !== btnLidarSettings && e.target !== btnSyncLidarPopover) {
      popoverLidar.classList.remove('open');
    }
  });
}

// 4. ROS2 Bridge UI Controls
function initROS2BridgeControls() {
  const btnConnect = document.getElementById('btn-toggle-bridge');
  const wsInput = document.getElementById('bridge-ws-url');
  const statusBadge = document.getElementById('bridge-status-badge');
  const statusText = document.getElementById('bridge-status-text');

  btnConnect?.addEventListener('click', () => {
    if (!ros2BridgeInstance.isConnected) {
      const url = wsInput?.value || 'ws://localhost:9090';
      ros2BridgeInstance.connect(url);
      btnConnect.textContent = 'Disconnect';
      statusBadge?.classList.add('connected');
      if (statusText) statusText.textContent = `ROS2 BRIDGE: CONNECTED (${url})`;
      showToast(`Connecting to ROS2 Bridge: ${url}`);
    } else {
      ros2BridgeInstance.disconnect();
      btnConnect.textContent = 'Connect ROS2';
      statusBadge?.classList.remove('connected');
      if (statusText) statusText.textContent = 'ROS2 BRIDGE: STANDALONE (ws://localhost:9090)';
      showToast('Disconnected from ROS2 Bridge');
    }
  });

  // Hook up simulator costmap generation to ROS2 bridge publisher
  if (simulator) {
    simulator.onCostmapUpdate = (costmapData, resolution, width, height, originX, originY) => {
      if (ros2BridgeInstance.isConnected) {
        ros2BridgeInstance.publishCostmap(costmapData, resolution, width, height, originX, originY);
      }
    };
  }

  // Bind incoming ROS2 Navigation Goals (RViz2 2D Goal Pose / Clicked Point)
  ros2BridgeInstance.onGoalReceived = (rosX, rosY, source) => {
    if (simulator) {
      simulator.setGoalFromROS(rosX, rosY);
      if (simulator.goalMode === 'multi' && simulator.waypoints.length > 0) {
        const activeNum = simulator.activeWaypointIndex + 1;
        showToast(`🎯 Active WP #${activeNum} overwritten from ROS2 (${source}): (${rosX.toFixed(2)}m, ${rosY.toFixed(2)}m)`);
      } else {
        showToast(`🎯 Goal synced from ROS2 (${source}): x=${rosX.toFixed(2)}m, y=${rosY.toFixed(2)}m`);
      }
    }
  };

  // Bind incoming ROS2 Robot Initial Pose (RViz2 2D Pose Estimate)
  ros2BridgeInstance.onInitialPoseReceived = (rosX, rosY, yaw) => {
    if (simulator) {
      simulator.setRobotFromROS(rosX, rosY, yaw);
      showToast(`🤖 Robot pose synced from ROS2: x=${rosX.toFixed(2)}m, y=${rosY.toFixed(2)}m`);
    }
  };

  // Topic Preview Card Updates (Dynamic Live Streaming)
  setInterval(() => {
    if (!simulator) return;
    const stat = simulator.getStatus();
    const prevRobot = document.getElementById('preview-robot-pose');
    const prevOdom = document.getElementById('preview-odom');
    const prevHumans = document.getElementById('preview-humans-pose');
    const typeHumans = document.getElementById('type-tracked-humans');
    const prevScan = document.getElementById('preview-scan');
    const typeScan = document.getElementById('type-scan');
    const prevGoal = document.getElementById('preview-goal-pose');
    const prevCostmap = document.getElementById('preview-costmap');
    const prevCmdVel = document.getElementById('preview-cmd-vel');

    if (prevRobot) {
      const rx = parseFloat(stat.robotPose.x).toFixed(2);
      const ry = parseFloat(stat.robotPose.y).toFixed(2);
      const ryaw = parseFloat(stat.robotPose.yawDeg).toFixed(2);
      prevRobot.textContent = `x: ${rx}m, y: ${ry}m, yaw: ${ryaw}°`;
    }
    if (prevHumans) {
      prevHumans.textContent = `Tracking ${stat.pedestrianCount} active pedestrians`;
      if (typeHumans) typeHumans.textContent = `geometry_msgs/PoseArray (N=${stat.pedestrianCount})`;
    }
    if (prevScan) {
      const r = simulator.lidarRays || 360;
      const rng = parseFloat(simulator.lidarMaxRangeM || 6.0).toFixed(2);
      const fov = simulator.lidarFovDeg || 360;
      prevScan.textContent = `LiDAR: ${r} rays @ ${rng}m (${fov}°)`;
      if (typeScan) typeScan.textContent = `sensor_msgs/LaserScan (${r} rays)`;
    }
    if (prevGoal) {
      if (simulator.goalMode === 'multi' && simulator.waypoints.length > 0) {
        const activeIdx = simulator.activeWaypointIndex;
        const curWp = simulator.waypoints[activeIdx] || simulator.waypoints[0];
        const wpRos = ros2BridgeInstance.toROSCoords(curWp.x, curWp.y, simulator.canvas.width, simulator.canvas.height, simulator.scale);
        const gx = parseFloat(wpRos.x).toFixed(2);
        const gy = parseFloat(wpRos.y).toFixed(2);
        prevGoal.textContent = `WP #${activeIdx + 1}/${simulator.waypoints.length}: x: ${gx}m, y: ${gy}m`;
      } else {
        const goalRos = ros2BridgeInstance.toROSCoords(simulator.goal.x, simulator.goal.y, simulator.canvas.width, simulator.canvas.height, simulator.scale);
        const gx = parseFloat(goalRos.x).toFixed(2);
        const gy = parseFloat(goalRos.y).toFixed(2);
        prevGoal.textContent = `Target: x: ${gx}m, y: ${gy}m`;
      }
    }
    if (prevCostmap) {
      if (simulator.costmapInfo) {
        const { width, height, resolution } = simulator.costmapInfo;
        const res = parseFloat(resolution).toFixed(2);
        prevCostmap.textContent = `Grid: ${width}x${height} @ ${res}m (${width * height} cells)`;
      } else {
        prevCostmap.textContent = `Grid: 100x62 @ 0.20m res`;
      }
    }
  }, 100);
}

// 4.5 Header Brand Logo Click Action (Scroll to Top)
function initNavbarBrand() {
  const brandLogo = document.querySelector('.nav-brand') || document.getElementById('header-brand-logo');
  if (!brandLogo) return;

  const scrollToTop = (e) => {
    if (e) e.preventDefault();
    window.scrollTo({
      top: 0,
      left: 0,
      behavior: 'smooth'
    });
    if (window.location.hash) {
      history.pushState('', document.title, window.location.pathname + window.location.search);
    }
  };

  brandLogo.addEventListener('click', scrollToTop);
  brandLogo.addEventListener('keydown', (e) => {
    if (e.key === 'Enter' || e.key === ' ') {
      scrollToTop(e);
    }
  });
}

// 5. Theory & Math Modal
function initTheoryModal() {
  const backdrop = document.getElementById('theory-modal-backdrop');
  const btnOpen = document.getElementById('header-btn-theory');
  const btnClose = document.getElementById('modal-close-btn');
  const contentBody = document.getElementById('theory-content-body');
  const theoryTabs = document.querySelectorAll('.theory-nav-btn');

  function renderTheory(theoryId) {
    if (!contentBody) return;

    if (theoryId === 'proxemics') {
      const p = simTheoryData.proxemicsTheory;
      contentBody.innerHTML = `
        <h4 style="color: var(--neon-cyan); margin-bottom: 12px; font-size: 1.15rem;">${p.title}</h4>
        <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 12px; margin-bottom: 18px;">
          ${p.zones.map(z => `
            <div style="background: rgba(255,255,255,0.03); border: 1px solid var(--border-color); border-radius: 8px; padding: 12px;">
              <div style="color: ${z.color}; font-weight: 700; font-family: var(--font-mono); font-size: 12px; margin-bottom: 4px;">${z.name} (${z.radius})</div>
              <div style="color: var(--text-secondary); font-size: 12px;">${z.description}</div>
            </div>
          `).join('')}
        </div>
        <div style="font-family: var(--font-mono); font-size: 11px; color: var(--neon-green); margin-bottom: 6px;">// ASYMMETRIC GAUSSIAN POTENTIAL FORMULA</div>
        <div class="theory-math-block">${p.gaussianFormula}</div>
      `;
      return;
    }

    if (theoryId === 'benchmarks') {
      contentBody.innerHTML = `
        <h4 style="color: var(--neon-green); margin-bottom: 12px; font-size: 1.15rem;">Benchmark Results on ETH / UCY Datasets</h4>
        <table style="width: 100%; border-collapse: collapse; font-family: var(--font-mono); font-size: 12px; text-align: left;">
          <thead>
            <tr style="border-bottom: 1px solid rgba(255,255,255,0.15); color: var(--neon-cyan);">
              <th style="padding: 8px;">Algorithm</th>
              <th style="padding: 8px;">ADE ↓</th>
              <th style="padding: 8px;">FDE ↓</th>
              <th style="padding: 8px;">Compliance ↑</th>
              <th style="padding: 8px;">Violations ↓</th>
              <th style="padding: 8px;">Comfort ↑</th>
            </tr>
          </thead>
          <tbody>
            ${simTheoryData.benchmarks.map(b => `
              <tr style="border-bottom: 1px solid rgba(255,255,255,0.05); color: var(--text-primary);">
                <td style="padding: 8px; font-weight: 600;">${b.name}</td>
                <td style="padding: 8px; color: var(--neon-cyan);">${b.ade}</td>
                <td style="padding: 8px;">${b.fde}</td>
                <td style="padding: 8px; color: var(--neon-green);">${b.compliance}</td>
                <td style="padding: 8px; color: var(--neon-coral);">${b.violations}</td>
                <td style="padding: 8px;">${b.comfort}</td>
              </tr>
            `).join('')}
          </tbody>
        </table>
      `;
      return;
    }

    const algo = simTheoryData.algorithms.find(a => a.id === theoryId);
    if (algo) {
      contentBody.innerHTML = `
        <div style="display: flex; justify-content: space-between; align-items: baseline; margin-bottom: 10px;">
          <h4 style="color: var(--neon-green); font-size: 1.2rem;">${algo.name}</h4>
          <span style="font-family: var(--font-mono); font-size: 11px; color: var(--text-muted);">${algo.author}</span>
        </div>
        <p style="color: var(--text-secondary); font-size: 13px; margin-bottom: 14px;">${algo.description}</p>
        
        <div style="font-family: var(--font-mono); font-size: 11px; color: var(--neon-cyan); margin-bottom: 6px;">// MATHEMATICAL FORMULATION</div>
        <div class="theory-math-block">${algo.equation}</div>

        <div style="font-family: var(--font-mono); font-size: 11px; color: var(--neon-green); margin-bottom: 6px;">// C++ / PYTORCH IMPLEMENTATION SNIPPET</div>
        <pre class="theory-code-block"><code>${algo.codeSnippet}</code></pre>
      `;
    }
  }

  btnOpen?.addEventListener('click', () => {
    backdrop?.classList.add('open');
    renderTheory('proxemics');
  });

  btnClose?.addEventListener('click', () => backdrop?.classList.remove('open'));
  backdrop?.addEventListener('click', (e) => {
    if (e.target === backdrop) backdrop.classList.remove('open');
  });

  theoryTabs.forEach(tab => {
    tab.addEventListener('click', () => {
      theoryTabs.forEach(t => t.classList.remove('active'));
      tab.classList.add('active');
      renderTheory(tab.getAttribute('data-theory'));
    });
  });
}

// 6. Global Toast Helper
export function showToast(message) {
  const toast = document.getElementById('global-toast');
  const toastMsg = document.getElementById('toast-message');
  if (!toast || !toastMsg) return;

  toastMsg.textContent = message;
  toast.classList.add('show');

  setTimeout(() => {
    toast.classList.remove('show');
  }, 2400);
}

// 7. Floating Theme Customization Manager (Bottom-Right)
export function initThemeSwitcher() {
  const fabBtn = document.getElementById('theme-fab-btn');
  const popover = document.getElementById('theme-popover');
  const closeBtn = document.getElementById('btn-close-theme-popover');
  const themeBtns = document.querySelectorAll('.theme-option-btn');
  const activeBadge = document.getElementById('theme-active-badge');

  const THEMES = {
    // Cyber & Sci-Fi Dark
    obsidian: { name: 'Obsidian', label: 'Obsidian Matrix' },
    cyberpunk_neon: { name: 'Cyberpunk', label: 'Cyberpunk 2077' },
    tokyo: { name: 'Tokyo', label: 'Tokyo Cyber' },
    synthwave_sunset: { name: 'Synthwave', label: 'Synthwave Sunset' },
    apollo: { name: 'Apollo', label: 'Apollo Cosmos' },
    emerald: { name: 'Emerald', label: 'Emerald CRT' },
    crimson_void: { name: 'Crimson', label: 'Crimson Laser' },
    dracula_vampire: { name: 'Dracula', label: 'Dracula Pro' },
    // Technical & Minimalist
    nord_aurora: { name: 'Nord Frost', label: 'Nord Arctic Frost' },
    cobalt_blueprint: { name: 'Blueprint', label: 'CAD Blueprint' },
    monochrome_minimal: { name: 'Monochrome', label: 'Stark Monochrome' },
    // Light & Warm
    light: { name: 'Polar Lab', label: 'Polar Laboratory' },
    solar_light: { name: 'Solar Sand', label: 'Solar Sand' },
    coffee_latte: { name: 'Espresso', label: 'Espresso Latte' },
    sakura_light: { name: 'Neo Sakura', label: 'Neo Sakura' },
    mint_light: { name: 'Eco Mint', label: 'Eco Mint' }
  };

  function applyTheme(themeId, notify = true) {
    if (!THEMES[themeId]) themeId = 'dracula_vampire';

    document.documentElement.setAttribute('data-theme', themeId);
    localStorage.setItem('socialnav_studio_theme', themeId);

    // Update active button state
    themeBtns.forEach(btn => {
      if (btn.getAttribute('data-theme') === themeId) {
        btn.classList.add('active');
      } else {
        btn.classList.remove('active');
      }
    });

    if (activeBadge) {
      activeBadge.textContent = THEMES[themeId].name;
    }

    if (notify) {
      showToast(`🎨 Theme switched to: ${THEMES[themeId].label}`);
    }
  }

  // Load saved theme from storage (Default: Dracula Pro)
  const savedTheme = localStorage.getItem('socialnav_studio_theme') || 'dracula_vampire';
  applyTheme(savedTheme, false);

  // Toggle popover
  fabBtn?.addEventListener('click', (e) => {
    e.stopPropagation();
    popover?.classList.toggle('open');
  });

  closeBtn?.addEventListener('click', (e) => {
    e.stopPropagation();
    popover?.classList.remove('open');
  });

  // Theme option clicks
  themeBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      const themeId = btn.getAttribute('data-theme');
      applyTheme(themeId, true);
      popover?.classList.remove('open');
    });
  });

  // Close when clicking outside
  document.addEventListener('click', (e) => {
    if (popover && !popover.contains(e.target) && e.target !== fabBtn && !fabBtn?.contains(e.target)) {
      popover.classList.remove('open');
    }
  });

  // Expose global helper for terminal command
  window.setStudioTheme = applyTheme;
}
