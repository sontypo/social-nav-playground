/**
 * SocialNav Studio - SSH Remote Interactive Terminal Window & Robot Controller
 * Handles remote robot profiles, SSH Gateway WebSocket IPC (ws://localhost:9092),
 * remote launch execution, port forwarding, and interactive ANSI Web Terminal.
 */

export class SSHManager {
  constructor(liveController) {
    this.liveController = liveController;
    this.wsUrl = 'ws://localhost:9092';
    this.socket = null;
    this.isConnected = false;
    this.isLaunching = false;
    this.remoteRunning = false;

    this.profiles = [];
    this.currentProfileId = null;

    this.commandHistory = [];
    this.historyIndex = -1;

    this.initProfiles();
  }

  initProfiles() {
    const defaultProfiles = [
      {
        id: 'amr-zedx-livox',
        name: '🤖 AMR Robot (ZED X Camera + Livox Mid-360 LiDAR)',
        host: '192.168.1.108',
        port: 22,
        username: 'robot',
        auth_mode: 'password',
        key_path: '~/.ssh/id_rsa',
        ros_distro: 'humble',
        ws_setup: '/opt/ros/humble/setup.bash',
        preset: 'rosbridge',
        custom_cmd: 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091',
        auto_tunnel: true,
        local_port: 9091,
        remote_port: 9091
      },
      {
        id: 'jackal-lab-1',
        name: '🤖 Lab Jackal AMR (192.168.1.105)',
        host: '192.168.1.105',
        port: 22,
        username: 'robot',
        auth_mode: 'password',
        key_path: '~/.ssh/id_rsa',
        ros_distro: 'humble',
        ws_setup: '/opt/ros/humble/setup.bash',
        preset: 'rosbridge',
        custom_cmd: 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091',
        auto_tunnel: true,
        local_port: 9091,
        remote_port: 9091
      },
      {
        id: 'turtlebot4-lab',
        name: '🤖 Turtlebot4 Hallway (192.168.1.106)',
        host: '192.168.1.106',
        port: 22,
        username: 'ubuntu',
        auth_mode: 'password',
        key_path: '',
        ros_distro: 'humble',
        ws_setup: '/opt/ros/humble/setup.bash',
        preset: 'rosbridge',
        custom_cmd: '',
        auto_tunnel: true,
        local_port: 9091,
        remote_port: 9091
      },
      {
        id: 'remote-gpu-nuc',
        name: '🖥️ Remote GPU Sim NUC (10.0.0.12)',
        host: '10.0.0.12',
        port: 22,
        username: 'nav_admin',
        auth_mode: 'key',
        key_path: '~/.ssh/id_ed25519',
        ros_distro: 'humble',
        ws_setup: '~/ros2_ws/install/setup.bash',
        preset: 'rosbridge',
        custom_cmd: '',
        auto_tunnel: true,
        local_port: 9091,
        remote_port: 9091
      },
      {
        id: 'localhost-dev',
        name: '💻 Localhost Loopback (127.0.0.1)',
        host: '127.0.0.1',
        port: 22,
        username: '',
        auth_mode: 'password',
        key_path: '',
        ros_distro: 'humble',
        ws_setup: '/opt/ros/humble/setup.bash',
        preset: 'rosbridge',
        custom_cmd: '',
        auto_tunnel: false,
        local_port: 9091,
        remote_port: 9091
      }
    ];

    try {
      const stored = localStorage.getItem('socialnav_ssh_profiles');
      this.profiles = stored ? JSON.parse(stored) : defaultProfiles;
    } catch {
      this.profiles = defaultProfiles;
    }
    this.currentProfileId = this.profiles[0]?.id || 'jackal-lab-1';
  }

  saveProfilesToStorage() {
    try {
      localStorage.setItem('socialnav_ssh_profiles', JSON.stringify(this.profiles));
    } catch (e) {
      console.warn('Failed to persist SSH profiles to localStorage:', e);
    }
  }

  initUI() {
    this.populateProfileDropdown();
    this.loadProfileIntoForm(this.currentProfileId);
    this.bindEvents();
    this.connectGateway();
  }

  connectGateway() {
    if (this.socket && (this.socket.readyState === WebSocket.OPEN || this.socket.readyState === WebSocket.CONNECTING)) {
      return;
    }

    try {
      this.socket = new WebSocket(this.wsUrl);

      this.socket.onopen = () => {
        this.isConnected = true;
        this.updateGatewayBadge(true);
        this.appendTerminalLog('\x1b[32m[GATEWAY-OK]\x1b[0m Connected to local SSH Daemon (ws://localhost:9092)');
      };

      this.socket.onmessage = (event) => {
        try {
          const msg = JSON.parse(event.data);
          this.handleGatewayMessage(msg);
        } catch (e) {
          console.error('Invalid JSON from SSH Gateway:', e);
        }
      };

      this.socket.onclose = () => {
        this.isConnected = false;
        this.updateGatewayBadge(false);
        setTimeout(() => this.connectGateway(), 4000);
      };

      this.socket.onerror = () => {
        this.isConnected = false;
        this.updateGatewayBadge(false);
      };
    } catch {
      this.updateGatewayBadge(false);
    }
  }

  updateGatewayBadge(connected) {
    const badge = document.getElementById('ssh-daemon-status-badge');
    if (!badge) return;
    if (connected) {
      badge.className = 'ssh-daemon-pill online';
      badge.innerHTML = '<span class="pulse-dot"></span> Daemon: ONLINE';
    } else {
      badge.className = 'ssh-daemon-pill offline';
      badge.innerHTML = '<span class="pulse-dot"></span> Daemon: OFFLINE';
    }
  }

  handleGatewayMessage(msg) {
    const op = msg.op;

    if (op === 'terminal_log') {
      this.appendTerminalLog(msg.text);
    } else if (op === 'test_result') {
      const resultBox = document.getElementById('ssh-probe-result');
      if (resultBox) {
        resultBox.style.display = 'block';
        if (msg.success) {
          resultBox.className = 'ssh-probe-box success';
          resultBox.innerHTML = `✅ <strong>Ping OK!</strong> Latency: <code>${msg.latency_ms}ms</code><br><span style="font-size:10px; color:#c0caf5;">${msg.details || ''}</span>`;
        } else {
          resultBox.className = 'ssh-probe-box error';
          resultBox.innerHTML = `❌ <strong>Probe Failed:</strong> ${msg.message}`;
        }
      }
    } else if (op === 'launch_status') {
      const btnLaunch = document.getElementById('btn-ssh-launch-bridge');
      const statusPill = document.getElementById('ssh-session-status-pill');

      if (msg.status === 'RUNNING') {
        this.remoteRunning = true;
        if (btnLaunch) {
          btnLaunch.innerHTML = '🛑 Stop ROS2 Bridge';
          btnLaunch.classList.add('running');
        }
        if (statusPill) {
          statusPill.className = 'ssh-session-pill active';
          statusPill.textContent = `STREAMING (${msg.host || ''})`;
        }
        this.appendTerminalLog(`\x1b[32m[ROS2-ACTIVE]\x1b[0m Remote ROS2 is streaming on port ${msg.local_port || 9091}!`);

        // Automatically trigger liveStream connect to port 9091
        setTimeout(() => {
          const wsInput = document.getElementById('live-ws-endpoint-input');
          const targetWsUrl = `ws://localhost:${msg.local_port || 9091}`;
          if (wsInput) wsInput.value = targetWsUrl;
          if (this.liveController) {
            this.liveController.connectBridge(targetWsUrl);
          }
        }, 1200);

      } else {
        this.remoteRunning = false;
        if (btnLaunch) {
          btnLaunch.innerHTML = '🚀 Launch ROS2 Bridge';
          btnLaunch.classList.remove('running');
        }
        if (statusPill) {
          statusPill.className = 'ssh-session-pill inactive';
          statusPill.textContent = 'IDLE';
        }
      }
    } else if (op === 'topics_discovered') {
      if (msg.success && msg.topics) {
        if (this.liveController && this.liveController.bridge) {
          this.liveController.bridge.discoveredTopics = msg.topics;
          const mapping = this.liveController.bridge.autoResolveTopics(msg.topics);
          if (this.liveController.bridge.onTopicsDiscovered) {
            this.liveController.bridge.onTopicsDiscovered(mapping, msg.topics);
          }
        }
      }
    }
  }

  getActiveCredentials() {
    const selectEl = document.getElementById('ssh-profile-select');
    const selectedId = selectEl?.value || this.currentProfileId;
    const savedP = this.profiles.find(x => x.id === selectedId) || this.profiles[0] || {};

    const getVal = (id, fallback) => {
      const v = document.getElementById(id)?.value?.trim();
      return (v !== undefined && v !== '') ? v : (fallback || '');
    };
    const getPwd = (id, fallback) => {
      const v = document.getElementById(id)?.value;
      return (v !== undefined && v !== '') ? v : (fallback || '');
    };

    return {
      host: getVal('ssh-input-host', savedP.host) || savedP.host || '127.0.0.1',
      port: parseInt(getVal('ssh-input-port', savedP.port)) || savedP.port || 22,
      username: getVal('ssh-input-user', savedP.username) || savedP.username || '',
      auth_mode: getVal('ssh-input-auth-mode', savedP.auth_mode) || savedP.auth_mode || 'password',
      password: getPwd('ssh-input-password', savedP.password) || savedP.password || '',
      key_path: getVal('ssh-input-key-path', savedP.key_path) || savedP.key_path || '',
      ros_distro: getVal('ssh-input-distro', savedP.ros_distro) || savedP.ros_distro || 'humble',
      ws_setup: getVal('ssh-input-setup', savedP.ws_setup) || savedP.ws_setup || '/opt/ros/humble/setup.bash'
    };
  }

  fetchRobotTopics() {
    if (!this.isConnected || !this.socket) {
      this.appendTerminalLog('\x1b[31m[ERR]\x1b[0m SSH Gateway is not connected. Run `./ros2_bridge/launch_ssh_bridge.sh`');
      return;
    }
    const creds = this.getActiveCredentials();
    this.appendTerminalLog(`\x1b[36m[DISCOVERY]\x1b[0m Querying ROS2 topics and message types from ${creds.username}@${creds.host}...`);
    this.socket.send(JSON.stringify({
      op: 'fetch_topics',
      host: creds.host,
      port: creds.port,
      username: creds.username,
      auth_mode: creds.auth_mode,
      password: creds.password,
      key_path: creds.key_path,
      ros_distro: creds.ros_distro,
      ws_setup: creds.ws_setup
    }));
  }

  populateProfileDropdown() {
    const select = document.getElementById('ssh-profile-select');
    if (!select) return;
    select.innerHTML = '';

    this.profiles.forEach(p => {
      const opt = document.createElement('option');
      opt.value = p.id;
      opt.textContent = p.name;
      if (p.id === this.currentProfileId) opt.selected = true;
      select.appendChild(opt);
    });
  }

  loadProfileIntoForm(profileId) {
    const p = this.profiles.find(x => x.id === profileId) || this.profiles[0];
    if (!p) return;

    this.currentProfileId = p.id;

    const setVal = (id, val) => {
      const el = document.getElementById(id);
      if (el) el.value = val !== undefined ? val : '';
    };

    setVal('ssh-input-name', p.name);
    setVal('ssh-input-host', p.host);
    setVal('ssh-input-port', p.port || 22);
    setVal('ssh-input-user', p.username);
    setVal('ssh-input-password', p.password || '');
    setVal('ssh-input-auth-mode', p.auth_mode || 'password');
    setVal('ssh-input-key-path', p.key_path || '');
    setVal('ssh-input-distro', p.ros_distro || 'humble');
    setVal('ssh-input-setup', p.ws_setup || '/opt/ros/humble/setup.bash');
    setVal('ssh-input-preset', p.preset || 'rosbridge');
    setVal('ssh-input-custom-cmd', p.custom_cmd || '');
    setVal('ssh-input-local-port', p.local_port || 9091);
    setVal('ssh-input-remote-port', p.remote_port || 9091);

    const autoTunnelCb = document.getElementById('ssh-cb-auto-tunnel');
    if (autoTunnelCb) autoTunnelCb.checked = p.auto_tunnel !== false;

    this.toggleAuthFields(p.auth_mode || 'password');
    this.updatePromptLabel();
  }

  updatePromptLabel() {
    const p = this.profiles.find(x => x.id === this.currentProfileId);
    const promptLabel = document.getElementById('ssh-term-prompt-label');
    if (promptLabel && p) {
      const user = p.username || 'robot';
      const host = p.host || 'remote';
      promptLabel.textContent = `${user}@${host}:~$`;
    }
  }

  collectFormProfile() {
    const getVal = (id) => document.getElementById(id)?.value?.trim() || '';
    const getPwd = (id) => document.getElementById(id)?.value || '';
    return {
      id: this.currentProfileId || `profile-${Date.now()}`,
      name: getVal('ssh-input-name') || 'New Robot Profile',
      host: getVal('ssh-input-host') || '127.0.0.1',
      port: parseInt(getVal('ssh-input-port')) || 22,
      username: getVal('ssh-input-user'),
      password: getPwd('ssh-input-password'),
      auth_mode: getVal('ssh-input-auth-mode') || 'password',
      key_path: getVal('ssh-input-key-path'),
      ros_distro: getVal('ssh-input-distro') || 'humble',
      ws_setup: getVal('ssh-input-setup') || '/opt/ros/humble/setup.bash',
      preset: getVal('ssh-input-preset') || 'rosbridge',
      custom_cmd: getVal('ssh-input-custom-cmd'),
      auto_tunnel: document.getElementById('ssh-cb-auto-tunnel')?.checked !== false,
      local_port: parseInt(getVal('ssh-input-local-port')) || 9091,
      remote_port: parseInt(getVal('ssh-input-remote-port')) || 9091
    };
  }

  toggleAuthFields(mode) {
    const pwdWrap = document.getElementById('ssh-wrap-password');
    const keyWrap = document.getElementById('ssh-wrap-key');
    if (pwdWrap && keyWrap) {
      if (mode === 'key') {
        pwdWrap.style.display = 'none';
        keyWrap.style.display = 'block';
      } else {
        pwdWrap.style.display = 'block';
        keyWrap.style.display = 'none';
      }
    }
  }

  openModal() {
    const modal = document.getElementById('ssh-manager-modal');
    const termInput = document.getElementById('ssh-terminal-input');
    if (!modal) return;
    modal.classList.add('open');
    this.connectGateway();
    setTimeout(() => termInput?.focus(), 150);
  }

  closeModal() {
    const modal = document.getElementById('ssh-manager-modal');
    if (!modal) return;
    modal.classList.remove('open');
  }

  bindEvents() {
    // Open & Close Modal
    const btnOpen = document.getElementById('btn-open-ssh-modal');
    const btnClose = document.getElementById('btn-close-ssh-modal');
    const modal = document.getElementById('ssh-manager-modal');
    const termInput = document.getElementById('ssh-terminal-input');

    btnOpen?.addEventListener('click', () => this.openModal());
    btnClose?.addEventListener('click', () => this.closeModal());

    modal?.addEventListener('click', (e) => {
      if (e.target === modal) {
        this.closeModal();
      }
    });

    window.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && modal?.classList.contains('open')) {
        this.closeModal();
      }
    });

    // Toggle Settings Drawer
    const btnToggleSettings = document.getElementById('btn-ssh-toggle-settings');
    const settingsDrawer = document.getElementById('ssh-settings-drawer');
    btnToggleSettings?.addEventListener('click', () => {
      if (settingsDrawer) {
        const isHidden = settingsDrawer.style.display === 'none';
        settingsDrawer.style.display = isHidden ? 'block' : 'none';
        btnToggleSettings.classList.toggle('active', isHidden);
      }
    });

    // Toggle Fullscreen
    const btnMaximize = document.getElementById('btn-ssh-toggle-maximize');
    const terminalWin = document.getElementById('ssh-terminal-window');
    btnMaximize?.addEventListener('click', () => {
      if (terminalWin) {
        terminalWin.classList.toggle('fullscreen');
        btnMaximize.textContent = terminalWin.classList.contains('fullscreen') ? '🗗' : '⛶';
      }
    });

    // Profile Dropdown change
    const selectProfile = document.getElementById('ssh-profile-select');
    selectProfile?.addEventListener('change', (e) => {
      this.loadProfileIntoForm(e.target.value);
    });

    // Auth mode dropdown
    const authSelect = document.getElementById('ssh-input-auth-mode');
    authSelect?.addEventListener('change', (e) => {
      this.toggleAuthFields(e.target.value);
    });

    // Preset dropdown change
    const presetSelect = document.getElementById('ssh-input-preset');
    presetSelect?.addEventListener('change', (e) => {
      const customWrap = document.getElementById('ssh-wrap-custom-cmd');
      if (customWrap) {
        customWrap.style.display = e.target.value === 'custom' ? 'block' : 'none';
      }
    });

    // Save Profile button
    const btnSave = document.getElementById('btn-ssh-save-profile');
    btnSave?.addEventListener('click', () => {
      const p = this.collectFormProfile();
      const idx = this.profiles.findIndex(x => x.id === p.id);
      if (idx !== -1) {
        this.profiles[idx] = p;
      } else {
        this.profiles.push(p);
      }
      this.saveProfilesToStorage();
      this.populateProfileDropdown();
      this.updatePromptLabel();
      this.appendTerminalLog(`\x1b[32m[PROFILE]\x1b[0m Saved profile: "${p.name}"`);
    });

    // Delete Profile button
    const btnDelete = document.getElementById('btn-ssh-delete-profile');
    btnDelete?.addEventListener('click', () => {
      if (this.profiles.length <= 1) {
        alert('Cannot delete the last remaining profile.');
        return;
      }
      this.profiles = this.profiles.filter(x => x.id !== this.currentProfileId);
      this.currentProfileId = this.profiles[0]?.id;
      this.saveProfilesToStorage();
      this.populateProfileDropdown();
      this.loadProfileIntoForm(this.currentProfileId);
    });

    // Test Connection Probe Handlers
    const handleTestProbe = () => this.runTestConnection();
    document.getElementById('btn-ssh-test-probe')?.addEventListener('click', handleTestProbe);
    document.getElementById('btn-ssh-drawer-test-probe')?.addEventListener('click', handleTestProbe);
    document.getElementById('btn-ssh-chip-test-probe')?.addEventListener('click', handleTestProbe);

    // Connect & Launch Button
    const btnLaunch = document.getElementById('btn-ssh-launch-bridge');
    btnLaunch?.addEventListener('click', () => {
      if (!this.isConnected) {
        alert('Local SSH Gateway Daemon is OFFLINE.\nPlease run `./ros2_bridge/launch_ssh_bridge.sh` first!');
        return;
      }

      if (this.remoteRunning) {
        this.socket.send(JSON.stringify({ op: 'stop_session' }));
      } else {
        const p = this.collectFormProfile();
        const pwd = document.getElementById('ssh-input-password')?.value || '';
        this.socket.send(JSON.stringify({
          op: 'launch_ros2',
          host: p.host,
          port: p.port,
          username: p.username,
          auth_mode: p.auth_mode,
          password: pwd,
          key_path: p.key_path,
          ros_distro: p.ros_distro,
          ws_setup: p.ws_setup,
          preset: p.preset,
          custom_cmd: p.custom_cmd,
          auto_tunnel: p.auto_tunnel,
          local_port: p.local_port,
          remote_port: p.remote_port
        }));
      }
    });

    // Terminal Command Line Input with Command History, Auto-Complete, Shortcuts & Clear
    const ROS2_AUTOCOMPLETE = [
      'ros2 topic list -t',
      'ros2 topic echo ',
      'ros2 topic hz ',
      'ros2 topic info ',
      'ros2 topic pub ',
      'ros2 node list',
      'ros2 node info ',
      'ros2 service list',
      'ros2 param list',
      'ros2 launch ',
      'ros2 run ',
      'ros2 doctor',
      'ros2 bag record ',
      'colcon build --symlink-install',
      'source install/setup.bash'
    ];

    termInput?.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        const cmd = termInput.value.trim();
        if (!cmd) return;
        termInput.value = '';

        this.commandHistory.push(cmd);
        this.historyIndex = this.commandHistory.length;

        if (cmd === 'clear') {
          const termBox = document.getElementById('ssh-terminal-logs');
          if (termBox) termBox.innerHTML = '';
          return;
        }

        if (!this.isConnected) {
          this.appendTerminalLog('\x1b[31m[ERROR]\x1b[0m Local SSH Daemon is offline. Run `./ros2_bridge/launch_ssh_bridge.sh`');
          return;
        }

        const creds = this.getActiveCredentials();
        this.socket.send(JSON.stringify({
          op: 'exec_command',
          command: cmd,
          host: creds.host,
          port: creds.port,
          username: creds.username,
          auth_mode: creds.auth_mode,
          password: creds.password,
          key_path: creds.key_path,
          ws_setup: creds.ws_setup
        }));
      } else if (e.ctrlKey && (e.key === 'c' || e.key === 'C')) {
        // Send Ctrl+C Interrupt if no text is selected for copying
        if (window.getSelection()?.toString().length === 0) {
          e.preventDefault();
          this.interruptRunningCommand();
          termInput.value = '';
        }
      } else if (e.ctrlKey && (e.key === 'l' || e.key === 'L')) {
        e.preventDefault();
        const termBox = document.getElementById('ssh-terminal-logs');
        if (termBox) termBox.innerHTML = '';
      } else if (e.key === 'Tab') {
        e.preventDefault();
        const cur = termInput.value;
        if (cur) {
          const match = ROS2_AUTOCOMPLETE.find(k => k.startsWith(cur));
          if (match) {
            termInput.value = match;
          }
        }
      } else if (e.key === 'ArrowUp') {
        e.preventDefault();
        if (this.commandHistory.length > 0 && this.historyIndex > 0) {
          this.historyIndex--;
          termInput.value = this.commandHistory[this.historyIndex];
        }
      } else if (e.key === 'ArrowDown') {
        e.preventDefault();
        if (this.historyIndex < this.commandHistory.length - 1) {
          this.historyIndex++;
          termInput.value = this.commandHistory[this.historyIndex];
        } else {
          this.historyIndex = this.commandHistory.length;
          termInput.value = '';
        }
      }
    });

    // Interrupt Buttons (Ctrl+C)
    const handleInterrupt = () => this.interruptRunningCommand();
    document.getElementById('btn-ssh-ctrl-c')?.addEventListener('click', handleInterrupt);
    document.getElementById('btn-ssh-cli-interrupt')?.addEventListener('click', handleInterrupt);

    // Copy Logs Button
    const btnCopy = document.getElementById('btn-ssh-copy-logs');
    btnCopy?.addEventListener('click', () => {
      const termBox = document.getElementById('ssh-terminal-logs');
      if (!termBox) return;
      navigator.clipboard?.writeText(termBox.innerText).then(() => {
        btnCopy.textContent = '✅ Copied!';
        setTimeout(() => { btnCopy.textContent = '📋 Copy'; }, 2000);
      });
    });

    // Quick Action Chips in Terminal
    document.querySelectorAll('.ssh-quick-chip').forEach(chip => {
      chip.addEventListener('click', () => {
        const action = chip.getAttribute('data-action');
        if (action === 'discover') {
          this.fetchRobotTopics();
          return;
        }

        const cmd = chip.getAttribute('data-cmd');
        if (cmd) {
          if (cmd === 'clear') {
            const termBox = document.getElementById('ssh-terminal-logs');
            if (termBox) termBox.innerHTML = '';
            return;
          }
          if (this.isConnected) {
            const creds = this.getActiveCredentials();
            this.socket.send(JSON.stringify({
              op: 'exec_command',
              command: cmd,
              host: creds.host,
              port: creds.port,
              username: creds.username,
              auth_mode: creds.auth_mode,
              password: creds.password,
              key_path: creds.key_path,
              ws_setup: creds.ws_setup
            }));
          }
        }
      });
    });

    // Clear Terminal Log Button
    const btnClearTerm = document.getElementById('btn-ssh-clear-terminal');
    btnClearTerm?.addEventListener('click', () => {
      const termBox = document.getElementById('ssh-terminal-logs');
      if (termBox) termBox.innerHTML = '';
    });
  }

  interruptRunningCommand() {
    if (this.isConnected && this.socket) {
      this.appendTerminalLog('\x1b[33m^C [Interrupt signal sent]\x1b[0m');
      this.socket.send(JSON.stringify({ op: 'interrupt_command' }));
    }
  }

  runTestConnection() {
    if (!this.isConnected) {
      alert('Local SSH Gateway Daemon is OFFLINE.\nPlease run `./ros2_bridge/launch_ssh_bridge.sh` first!');
      return;
    }
    const p = this.collectFormProfile();
    const pwd = document.getElementById('ssh-input-password')?.value || '';

    const resultBox = document.getElementById('ssh-probe-result');
    if (resultBox) {
      resultBox.style.display = 'block';
      resultBox.className = 'ssh-probe-box';
      resultBox.innerHTML = `📡 <strong>Pinging ${p.username}@${p.host}:${p.port}...</strong>`;
    }

    this.appendTerminalLog(`\x1b[36m[SSH-PROBE]\x1b[0m Testing SSH connection to \x1b[33m${p.username}@${p.host}:${p.port}\x1b[0m (Auth: ${p.auth_mode})...`);

    this.socket.send(JSON.stringify({
      op: 'test_connection',
      host: p.host,
      port: p.port,
      username: p.username,
      auth_mode: p.auth_mode,
      password: pwd,
      key_path: p.key_path
    }));
  }

  appendTerminalLog(text) {
    const termBox = document.getElementById('ssh-terminal-logs');
    if (!termBox) return;

    const formatted = this.ansiToHtml(text);
    const lineDiv = document.createElement('div');
    lineDiv.className = 'term-line';
    lineDiv.innerHTML = formatted;
    termBox.appendChild(lineDiv);

    termBox.scrollTop = termBox.scrollHeight;
  }

  ansiToHtml(text) {
    if (!text) return '';
    let result = text
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;');

    const ansiMap = {
      '\x1b[30m': '<span style="color:#6272a4;">',
      '\x1b[31m': '<span style="color:#ff5555;font-weight:700;">',
      '\x1b[32m': '<span style="color:#50fa7b;font-weight:700;">',
      '\x1b[33m': '<span style="color:#f1fa8c;font-weight:700;">',
      '\x1b[34m': '<span style="color:#bd93f9;">',
      '\x1b[35m': '<span style="color:#ff79c6;">',
      '\x1b[36m': '<span style="color:#8be9fd;font-weight:700;">',
      '\x1b[37m': '<span style="color:#f8f8f2;">',
      '\x1b[0m': '</span>',
      '\x1b[1m': '<span style="font-weight:700;">'
    };

    for (const [code, htmlTag] of Object.entries(ansiMap)) {
      result = result.split(code).join(htmlTag);
    }

    return result;
  }
}
