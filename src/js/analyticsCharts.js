// High-Performance Real-Time Telemetry & Data Analytics Charting Engine
// Renders 4 specialized compact charts for Social Compliance, Comfort Index, Min Distance, and Personal Violations

export class TelemetryAnalyticsEngine {
  constructor() {
    this.bufferSize = 150; // ~15 seconds @ 10 Hz sampling
    this.history = {
      timestamps: [],
      compliance: [],
      comfort: [],
      minDistance: [],
      violations: [],
      instantViolations: []
    };

    this.isPaused = false;
    this.timeWindowSeconds = 15;
    this.lastViolationCount = 0;
    this.sampleTimer = 0;
    this.sampleInterval = 90; // ms between data samples (~11 Hz)
    this.lastSampleTime = performance.now();

    // Canvas elements
    this.canvases = {
      compliance: null,
      comfort: null,
      minDistance: null,
      violations: null
    };

    this.contexts = {};
    this.statElements = {};
  }

  init() {
    this.canvases.compliance = document.getElementById('chart-compliance-canvas');
    this.canvases.comfort = document.getElementById('chart-comfort-canvas');
    this.canvases.minDistance = document.getElementById('chart-mindist-canvas');
    this.canvases.violations = document.getElementById('chart-violations-canvas');

    for (let key in this.canvases) {
      if (this.canvases[key]) {
        this.contexts[key] = this.canvases[key].getContext('2d');
        this.resizeCanvas(this.canvases[key]);
      }
    }

    window.addEventListener('resize', () => {
      for (let key in this.canvases) {
        if (this.canvases[key]) {
          this.resizeCanvas(this.canvases[key]);
          this.renderChart(key);
        }
      }
    });

    this.bindControls();
    this.startRenderLoop();
  }

  resizeCanvas(canvas) {
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(50, Math.floor(rect.width || 180)) * dpr;
    canvas.height = Math.max(30, Math.floor(rect.height || 92)) * dpr;
  }

  bindControls() {
    const pauseBtn = document.getElementById('btn-analytics-pause');
    const clearBtn = document.getElementById('btn-analytics-clear');
    const exportCsvBtn = document.getElementById('btn-analytics-csv');
    const exportJsonBtn = document.getElementById('btn-analytics-json');

    pauseBtn?.addEventListener('click', () => {
      this.isPaused = !this.isPaused;
      pauseBtn.classList.toggle('active', this.isPaused);
      pauseBtn.innerHTML = this.isPaused 
        ? `<svg width="12" height="12" viewBox="0 0 24 24" fill="currentColor"><polygon points="5 3 19 12 5 21 5 3"></polygon></svg> Resume`
        : `<svg width="12" height="12" viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="4" width="4" height="16"></rect><rect x="14" y="4" width="4" height="16"></rect></svg> Pause`;
    });

    clearBtn?.addEventListener('click', () => {
      this.clearHistory();
    });

    exportCsvBtn?.addEventListener('click', () => {
      this.exportCSV();
    });

    exportJsonBtn?.addEventListener('click', () => {
      this.exportJSON();
    });
  }

  pushTelemetry(data) {
    if (this.isPaused) return;

    const now = performance.now();
    if (now - this.lastSampleTime < this.sampleInterval) return;
    this.lastSampleTime = now;

    const timestamp = (now / 1000).toFixed(2);
    const compliance = Math.max(0, Math.min(100, data.complianceScore || 0));
    const comfort = Math.max(0, Math.min(100, data.comfortIndex || 0));
    const minDistance = Math.max(0, Math.min(10, data.minDistanceToHuman || 0));
    const totalViolations = data.violationsCount || 0;
    const instantViolation = totalViolations > this.lastViolationCount ? (totalViolations - this.lastViolationCount) : 0;
    this.lastViolationCount = totalViolations;

    this.history.timestamps.push(timestamp);
    this.history.compliance.push(compliance);
    this.history.comfort.push(comfort);
    this.history.minDistance.push(minDistance);
    this.history.violations.push(totalViolations);
    this.history.instantViolations.push(instantViolation);

    if (this.history.timestamps.length > this.bufferSize) {
      for (let k in this.history) {
        this.history[k].shift();
      }
    }

    this.updateStatSummary('compliance', this.history.compliance, '%', 1);
    this.updateStatSummary('comfort', this.history.comfort, '%', 1);
    this.updateStatSummary('mindist', this.history.minDistance, 'm', 2);
    this.updateStatSummary('violations', this.history.violations, ' ev', 0);
  }

  clearHistory() {
    for (let k in this.history) {
      this.history[k] = [];
    }
    this.lastViolationCount = 0;
  }

  calcStats(arr) {
    if (!arr || arr.length === 0) return { cur: 0, mean: 0, min: 0, max: 0, std: 0 };
    const cur = arr[arr.length - 1];
    const sum = arr.reduce((a, b) => a + b, 0);
    const mean = sum / arr.length;
    const min = Math.min(...arr);
    const max = Math.max(...arr);
    const variance = arr.reduce((a, b) => a + Math.pow(b - mean, 2), 0) / arr.length;
    const std = Math.sqrt(variance);
    return { cur, mean, min, max, std };
  }

  updateStatSummary(key, arr, unit = '', decimals = 1) {
    const stats = this.calcStats(arr);
    const meanEl = document.getElementById(`card-stat-${key}-mean`);
    const minMaxEl = document.getElementById(`card-stat-${key}-minmax`);

    if (meanEl) meanEl.textContent = `μ: ${stats.mean.toFixed(decimals)}${unit}`;
    if (minMaxEl) minMaxEl.textContent = `[${stats.min.toFixed(decimals)} – ${stats.max.toFixed(decimals)}]`;
  }

  startRenderLoop() {
    const loop = () => {
      for (let key in this.canvases) {
        if (this.canvases[key]) {
          this.renderChart(key);
        }
      }
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  renderChart(key) {
    const canvas = this.canvases[key];
    const ctx = this.contexts[key];
    if (!canvas || !ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    const padL = 26 * dpr;
    const padR = 6 * dpr;
    const padT = 5 * dpr;
    const padB = 14 * dpr;
    const plotW = w - padL - padR;
    const plotH = h - padT - padB;

    if (plotW <= 0 || plotH <= 0) return;

    if (key === 'compliance') {
      this.drawComplianceChart(ctx, plotW, plotH, padL, padT, dpr);
    } else if (key === 'comfort') {
      this.drawComfortChart(ctx, plotW, plotH, padL, padT, dpr);
    } else if (key === 'minDistance') {
      this.drawMinDistanceChart(ctx, plotW, plotH, padL, padT, dpr);
    } else if (key === 'violations') {
      this.drawViolationsChart(ctx, plotW, plotH, padL, padT, dpr);
    }
  }

  // 1. Social Compliance Area Chart (0 - 100%)
  drawComplianceChart(ctx, pw, ph, pl, pt, dpr) {
    const data = this.history.compliance;
    const isDark = !document.body.classList.contains('theme-light') && 
                   !document.body.classList.contains('theme-solar-light') && 
                   !document.body.classList.contains('theme-sakura-light') && 
                   !document.body.classList.contains('theme-mint-light');

    this.drawGrid(ctx, pw, ph, pl, pt, dpr, [0, 50, 100], (v) => `${v}%`, isDark);

    // 90% Benchmark Target Dashed Line
    const targetY = pt + ph * (1 - 90 / 100);
    ctx.save();
    ctx.strokeStyle = '#eab308';
    ctx.lineWidth = 1.0 * dpr;
    ctx.setLineDash([3 * dpr, 3 * dpr]);
    ctx.beginPath();
    ctx.moveTo(pl, targetY);
    ctx.lineTo(pl + pw, targetY);
    ctx.stroke();
    ctx.restore();

    if (data.length < 2) return;

    // Gradient Area Fill
    const grad = ctx.createLinearGradient(0, pt, 0, pt + ph);
    grad.addColorStop(0, 'rgba(0, 255, 157, 0.4)');
    grad.addColorStop(0.8, 'rgba(0, 255, 157, 0.08)');
    grad.addColorStop(1, 'rgba(0, 255, 157, 0.0)');

    ctx.beginPath();
    const stepX = pw / Math.max(1, this.bufferSize - 1);
    const startIdx = Math.max(0, this.bufferSize - data.length);

    data.forEach((val, i) => {
      const x = pl + (startIdx + i) * stepX;
      const y = pt + ph * (1 - Math.max(0, Math.min(100, val)) / 100);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });

    const lastX = pl + (startIdx + data.length - 1) * stepX;
    const firstX = pl + startIdx * stepX;
    ctx.lineTo(lastX, pt + ph);
    ctx.lineTo(firstX, pt + ph);
    ctx.closePath();
    ctx.fillStyle = grad;
    ctx.fill();

    // Foreground Line
    ctx.beginPath();
    data.forEach((val, i) => {
      const x = pl + (startIdx + i) * stepX;
      const y = pt + ph * (1 - Math.max(0, Math.min(100, val)) / 100);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.strokeStyle = '#00FF9D';
    ctx.lineWidth = 1.8 * dpr;
    ctx.stroke();

    // Pulse dot at latest value
    const lastVal = data[data.length - 1];
    const lastY = pt + ph * (1 - Math.max(0, Math.min(100, lastVal)) / 100);
    ctx.beginPath();
    ctx.arc(lastX, lastY, 3 * dpr, 0, Math.PI * 2);
    ctx.fillStyle = '#ffffff';
    ctx.fill();
    ctx.lineWidth = 1.5 * dpr;
    ctx.strokeStyle = '#00FF9D';
    ctx.stroke();
  }

  // 2. Comfort Index: Bar Chart with Peak-Connected Envelope Line (0 - 100%)
  drawComfortChart(ctx, pw, ph, pl, pt, dpr) {
    const data = this.history.comfort;
    const isDark = !document.body.classList.contains('theme-light') && 
                   !document.body.classList.contains('theme-solar-light') && 
                   !document.body.classList.contains('theme-sakura-light') && 
                   !document.body.classList.contains('theme-mint-light');

    // Comfort Grade Background Bands
    const y90 = pt + ph * 0.1;
    const y70 = pt + ph * 0.3;
    ctx.fillStyle = isDark ? 'rgba(0, 255, 157, 0.04)' : 'rgba(0, 255, 157, 0.1)';
    ctx.fillRect(pl, pt, pw, y90 - pt);

    ctx.fillStyle = isDark ? 'rgba(234, 179, 8, 0.04)' : 'rgba(234, 179, 8, 0.08)';
    ctx.fillRect(pl, y90, pw, y70 - y90);

    ctx.fillStyle = isDark ? 'rgba(255, 0, 85, 0.04)' : 'rgba(255, 0, 85, 0.08)';
    ctx.fillRect(pl, y70, pw, (pt + ph) - y70);

    this.drawGrid(ctx, pw, ph, pl, pt, dpr, [0, 50, 100], (v) => `${v}%`, isDark);

    if (data.length < 2) return;

    const stepX = pw / Math.max(1, this.bufferSize - 1);
    const startIdx = Math.max(0, this.bufferSize - data.length);
    const barWidth = Math.max(2 * dpr, Math.min(6 * dpr, stepX * 0.65));

    // 1. Draw Vertical Bars for each telemetry sample
    data.forEach((val, i) => {
      const x = pl + (startIdx + i) * stepX;
      const clampedVal = Math.max(0, Math.min(100, val));
      const y = pt + ph * (1 - clampedVal / 100);
      const barH = (pt + ph) - y;

      const barGrad = ctx.createLinearGradient(0, y, 0, pt + ph);
      barGrad.addColorStop(0, 'rgba(192, 132, 252, 0.45)');
      barGrad.addColorStop(1, 'rgba(168, 85, 247, 0.04)');

      ctx.fillStyle = barGrad;
      ctx.fillRect(x - barWidth / 2, y, barWidth, barH);
    });

    // 2. Draw Peak-Connected Polyline
    ctx.beginPath();
    data.forEach((val, i) => {
      const x = pl + (startIdx + i) * stepX;
      const clampedVal = Math.max(0, Math.min(100, val));
      const y = pt + ph * (1 - clampedVal / 100);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.strokeStyle = '#c084fc';
    ctx.lineWidth = 1.8 * dpr;
    ctx.stroke();

    // 3. Peak Summit Node Dots (sampled for clean visuals)
    const stepSample = Math.max(1, Math.floor(data.length / 25));
    data.forEach((val, i) => {
      if (i % stepSample === 0 || i === data.length - 1) {
        const x = pl + (startIdx + i) * stepX;
        const clampedVal = Math.max(0, Math.min(100, val));
        const y = pt + ph * (1 - clampedVal / 100);
        ctx.beginPath();
        ctx.arc(x, y, 2.0 * dpr, 0, Math.PI * 2);
        ctx.fillStyle = '#ffffff';
        ctx.fill();
        ctx.strokeStyle = '#a855f7';
        ctx.lineWidth = 1.2 * dpr;
        ctx.stroke();
      }
    });

    // Latest Active Head
    const lastX = pl + (startIdx + data.length - 1) * stepX;
    const lastVal = data[data.length - 1];
    const lastY = pt + ph * (1 - Math.max(0, Math.min(100, lastVal)) / 100);
    ctx.beginPath();
    ctx.arc(lastX, lastY, 3.5 * dpr, 0, Math.PI * 2);
    ctx.fillStyle = '#ffffff';
    ctx.fill();
    ctx.strokeStyle = '#c084fc';
    ctx.lineWidth = 1.8 * dpr;
    ctx.stroke();
  }

  // 3. Min Distance to Human with 3 Proxemics Zones (0m - 3.6m)
  drawMinDistanceChart(ctx, pw, ph, pl, pt, dpr) {
    const data = this.history.minDistance;
    const maxMeters = 3.6;
    const isDark = !document.body.classList.contains('theme-light') && 
                   !document.body.classList.contains('theme-solar-light') && 
                   !document.body.classList.contains('theme-sakura-light') && 
                   !document.body.classList.contains('theme-mint-light');

    // 1. Intimate Zone (< 0.45m)
    const yIntimate = pt + ph * (1 - 0.45 / maxMeters);
    ctx.fillStyle = isDark ? 'rgba(255, 0, 85, 0.16)' : 'rgba(255, 0, 85, 0.22)';
    ctx.fillRect(pl, yIntimate, pw, (pt + ph) - yIntimate);

    // 2. Personal Space (0.45m - 1.20m)
    const yPersonal = pt + ph * (1 - 1.20 / maxMeters);
    ctx.fillStyle = isDark ? 'rgba(245, 158, 11, 0.12)' : 'rgba(245, 158, 11, 0.16)';
    ctx.fillRect(pl, yPersonal, pw, yIntimate - yPersonal);

    // 3. Social Space (> 1.20m)
    ctx.fillStyle = isDark ? 'rgba(0, 229, 255, 0.05)' : 'rgba(0, 229, 255, 0.1)';
    ctx.fillRect(pl, pt, pw, yPersonal - pt);

    this.drawGrid(ctx, pw, ph, pl, pt, dpr, [0, 1.2, 3.6], (v) => `${v.toFixed(1)}m`, isDark);

    if (data.length < 2) return;

    const stepX = pw / Math.max(1, this.bufferSize - 1);
    const startIdx = Math.max(0, this.bufferSize - data.length);

    ctx.beginPath();
    data.forEach((val, i) => {
      const x = pl + (startIdx + i) * stepX;
      const clampedVal = Math.max(0, Math.min(maxMeters, val));
      const y = pt + ph * (1 - clampedVal / maxMeters);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });

    ctx.strokeStyle = '#00E5FF';
    ctx.lineWidth = 1.8 * dpr;
    ctx.stroke();

    // Current point dot color-coded by zone
    const lastX = pl + (startIdx + data.length - 1) * stepX;
    const lastVal = data[data.length - 1];
    const clampedLast = Math.max(0, Math.min(maxMeters, lastVal));
    const lastY = pt + ph * (1 - clampedLast / maxMeters);

    let dotColor = '#00E5FF';
    if (lastVal < 0.45) dotColor = '#FF0055';
    else if (lastVal < 1.20) dotColor = '#F59E0B';

    ctx.beginPath();
    ctx.arc(lastX, lastY, 3.5 * dpr, 0, Math.PI * 2);
    ctx.fillStyle = dotColor;
    ctx.fill();
    ctx.lineWidth = 1.5 * dpr;
    ctx.strokeStyle = '#ffffff';
    ctx.stroke();
  }

  // 4. Personal Violations: Dirac-Delta Impulse Spike Train Graph
  drawViolationsChart(ctx, pw, ph, pl, pt, dpr) {
    const dataCum = this.history.violations;
    const dataInstant = this.history.instantViolations;
    const isDark = !document.body.classList.contains('theme-light') && 
                   !document.body.classList.contains('theme-solar-light') && 
                   !document.body.classList.contains('theme-sakura-light') && 
                   !document.body.classList.contains('theme-mint-light');

    // Draw baseline and impulse range ticks (0, 1)
    this.drawGrid(ctx, pw, ph, pl, pt, dpr, [0, 1], (v) => v === 0 ? '0' : '1 (impulse)', isDark);

    // Baseline illuminated axis at Y = 0
    ctx.beginPath();
    ctx.moveTo(pl, pt + ph);
    ctx.lineTo(pl + pw, pt + ph);
    ctx.strokeStyle = 'rgba(255, 0, 85, 0.35)';
    ctx.lineWidth = 1.5 * dpr;
    ctx.stroke();

    if (dataInstant.length === 0) return;

    const stepX = pw / Math.max(1, this.bufferSize - 1);
    const startIdx = Math.max(0, this.bufferSize - dataInstant.length);

    // Draw Dirac-Delta Impulse Spikes
    let hasActivePulse = false;
    dataInstant.forEach((v, i) => {
      const x = pl + (startIdx + i) * stepX;
      if (v > 0) {
        hasActivePulse = true;
        const spikeHeight = ph * 0.85;
        const topY = pt + ph - spikeHeight;

        // Vertical Glowing Stem
        const stemGrad = ctx.createLinearGradient(0, topY, 0, pt + ph);
        stemGrad.addColorStop(0, '#FF0055');
        stemGrad.addColorStop(0.7, 'rgba(255, 0, 85, 0.5)');
        stemGrad.addColorStop(1, 'rgba(255, 0, 85, 0.05)');

        ctx.beginPath();
        ctx.moveTo(x, pt + ph);
        ctx.lineTo(x, topY);
        ctx.strokeStyle = stemGrad;
        ctx.lineWidth = 2.2 * dpr;
        ctx.stroke();

        // Glowing Needle Head Cap (Diamond / Circle)
        ctx.beginPath();
        ctx.arc(x, topY, 3.2 * dpr, 0, Math.PI * 2);
        ctx.fillStyle = '#ffffff';
        ctx.fill();
        ctx.strokeStyle = '#FF0055';
        ctx.lineWidth = 1.5 * dpr;
        ctx.stroke();

        // Pulse Echo Halo
        ctx.beginPath();
        ctx.arc(x, topY, 6.0 * dpr, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(255, 0, 85, 0.3)';
        ctx.lineWidth = 1.0 * dpr;
        ctx.stroke();
      }
    });

    // Draw baseline idle signal trace
    ctx.beginPath();
    dataInstant.forEach((v, i) => {
      const x = pl + (startIdx + i) * stepX;
      const y = v > 0 ? (pt + ph - ph * 0.85) : (pt + ph);
      if (i === 0) ctx.moveTo(x, y);
      else {
        // Square wave step connection for impulse transitions
        const prevX = pl + (startIdx + i - 1) * stepX;
        const prevV = dataInstant[i - 1];
        const prevY = prevV > 0 ? (pt + ph - ph * 0.85) : (pt + ph);
        if (prevY !== y) {
          ctx.lineTo(x, prevY);
        }
        ctx.lineTo(x, y);
      }
    });
    ctx.strokeStyle = 'rgba(255, 0, 85, 0.6)';
    ctx.lineWidth = 1.2 * dpr;
    ctx.stroke();

    // Pulse Head at latest position
    const lastX = pl + (startIdx + dataInstant.length - 1) * stepX;
    const lastVal = dataInstant[dataInstant.length - 1];
    const lastY = lastVal > 0 ? (pt + ph - ph * 0.85) : (pt + ph);
    ctx.beginPath();
    ctx.arc(lastX, lastY, 3.0 * dpr, 0, Math.PI * 2);
    ctx.fillStyle = lastVal > 0 ? '#FF0055' : 'rgba(255, 0, 85, 0.4)';
    ctx.fill();
  }

  drawGrid(ctx, pw, ph, pl, pt, dpr, yTicks, formatFn, isDark) {
    ctx.save();
    ctx.lineWidth = 0.6 * dpr;
    ctx.strokeStyle = isDark ? 'rgba(255, 255, 255, 0.08)' : 'rgba(0, 0, 0, 0.08)';
    ctx.fillStyle = isDark ? 'rgba(255, 255, 255, 0.4)' : 'rgba(0, 0, 0, 0.5)';
    ctx.font = `${7.5 * dpr}px JetBrains Mono, monospace`;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    const maxVal = yTicks[yTicks.length - 1];
    yTicks.forEach(tick => {
      const ratio = tick / (maxVal || 1);
      const y = pt + ph * (1 - ratio);
      ctx.beginPath();
      ctx.moveTo(pl, y);
      ctx.lineTo(pl + pw, y);
      ctx.stroke();

      ctx.fillText(formatFn(tick), pl - 3 * dpr, y);
    });

    ctx.beginPath();
    ctx.moveTo(pl, pt + ph);
    ctx.lineTo(pl + pw, pt + ph);
    ctx.strokeStyle = isDark ? 'rgba(255, 255, 255, 0.15)' : 'rgba(0, 0, 0, 0.15)';
    ctx.stroke();

    ctx.restore();
  }

  exportCSV() {
    if (!this.history.timestamps.length) {
      alert('No telemetry data to export.');
      return;
    }

    let csv = 'timestamp_s,social_compliance_pct,comfort_index_pct,min_distance_meters,total_violations,instant_violations\n';
    for (let i = 0; i < this.history.timestamps.length; i++) {
      csv += `${this.history.timestamps[i]},${this.history.compliance[i]},${this.history.comfort[i]},${this.history.minDistance[i]},${this.history.violations[i]},${this.history.instantViolations[i]}\n`;
    }

    const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.setAttribute('href', url);
    link.setAttribute('download', `socialnav_telemetry_${Date.now()}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  }

  exportJSON() {
    if (!this.history.timestamps.length) {
      alert('No telemetry data to export.');
      return;
    }

    const exportData = {
      project: "SocialNav Studio Telemetry Analytics",
      exportedAt: new Date().toISOString(),
      sampleCount: this.history.timestamps.length,
      timeWindowSeconds: this.timeWindowSeconds,
      summaryStats: {
        compliance: this.calcStats(this.history.compliance),
        comfort: this.calcStats(this.history.comfort),
        minDistance: this.calcStats(this.history.minDistance),
        violations: this.calcStats(this.history.violations)
      },
      timeSeries: this.history.timestamps.map((t, i) => ({
        timestamp: parseFloat(t),
        socialCompliancePct: this.history.compliance[i],
        comfortIndexPct: this.history.comfort[i],
        minDistanceMeters: this.history.minDistance[i],
        totalViolations: this.history.violations[i],
        instantViolations: this.history.instantViolations[i]
      }))
    };

    const blob = new Blob([JSON.stringify(exportData, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.setAttribute('href', url);
    link.setAttribute('download', `socialnav_telemetry_${Date.now()}.json`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  }
}

export const telemetryAnalytics = new TelemetryAnalyticsEngine();
