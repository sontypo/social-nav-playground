// ROS2 Telemetry & Attitude Streamer

export function initTelemetryStream() {
  const elHeading = document.getElementById('hud-val-heading');
  const elBatt = document.getElementById('hud-val-batt');
  const elBattBar = document.getElementById('hud-bar-batt');
  const elCpu = document.getElementById('hud-val-cpu');
  const elCpuBar = document.getElementById('hud-bar-cpu');
  const elComfort = document.getElementById('hud-val-comfort');
  const elComfortBar = document.getElementById('hud-bar-comfort');
  const elLoopTime = document.getElementById('hud-val-loop');

  let heading = 142;
  let battery = 88;
  let cpu = 32;
  let comfort = 98.4;

  setInterval(() => {
    // Slight jitter to simulate real robot sensor streams
    heading = (heading + (Math.random() * 2 - 1) + 360) % 360;
    cpu = Math.min(65, Math.max(24, Math.floor(cpu + (Math.random() * 6 - 3))));
    comfort = Math.min(99.9, Math.max(92.0, +(comfort + (Math.random() * 0.4 - 0.2)).toFixed(1)));
    const loopMs = (8.2 + Math.random() * 0.6).toFixed(1);

    if (elHeading) elHeading.textContent = `${Math.floor(heading).toString().padStart(3, '0')}°`;
    if (elBatt) elBatt.textContent = `${battery}%`;
    if (elBattBar) elBattBar.style.width = `${battery}%`;
    if (elCpu) elCpu.textContent = `${cpu}%`;
    if (elCpuBar) elCpuBar.style.width = `${cpu}%`;
    if (elComfort) elComfort.textContent = `${comfort}%`;
    if (elComfortBar) elComfortBar.style.width = `${comfort}%`;
    if (elLoopTime) elLoopTime.textContent = `${loopMs}ms`;
  }, 600);
}
