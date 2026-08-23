// AI Robotics Assistant Engine (Multi-Backend: Google Gemini API, Ollama Local LLMs, Gemini Nano & Embedded Expert)

export class SocialNavAIAssistant {
  constructor() {
    this.providers = ['gemini', 'ollama', 'nano', 'local'];
    this.geminiApiKey = (typeof localStorage !== 'undefined' ? localStorage.getItem('socialnav_gemini_api_key') : '') || '';
    this.geminiModel = (typeof localStorage !== 'undefined' ? localStorage.getItem('socialnav_gemini_model') : '') || 'gemini-2.5-flash';
    this.ollamaEndpoint = (typeof localStorage !== 'undefined' ? localStorage.getItem('socialnav_ollama_endpoint') : '') || 'http://localhost:11434';
    this.ollamaModel = (typeof localStorage !== 'undefined' ? localStorage.getItem('socialnav_ollama_model') : '') || 'llama3:latest';
    
    // Default provider selection
    const savedProvider = typeof localStorage !== 'undefined' ? localStorage.getItem('socialnav_ai_provider') : null;
    if (savedProvider && this.providers.includes(savedProvider)) {
      this.activeProvider = savedProvider;
    } else if (this.geminiApiKey) {
      this.activeProvider = 'gemini';
    } else {
      this.activeProvider = 'local';
    }

    this.geminiNanoSession = null;
    this.history = []; // Multi-turn chat memory: [ { role: 'user' | 'assistant', content: string } ]
    this.initNanoIfAvailable();
  }

  async initNanoIfAvailable() {
    try {
      if (typeof window !== 'undefined' && window.ai && window.ai.languageModel) {
        const caps = await window.ai.languageModel.capabilities();
        if (caps && caps.available !== 'no' && !this.geminiApiKey) {
          this.activeProvider = 'nano';
        }
      }
    } catch (e) {
      // Ignore
    }
  }

  setApiKey(key) {
    const trimmed = key ? key.trim() : '';
    this.geminiApiKey = trimmed;
    if (typeof localStorage !== 'undefined') {
      if (trimmed) {
        localStorage.setItem('socialnav_gemini_api_key', trimmed);
        localStorage.setItem('socialnav_ai_provider', 'gemini');
      } else {
        localStorage.removeItem('socialnav_gemini_api_key');
      }
    }
    if (trimmed) {
      this.activeProvider = 'gemini';
      return `<span class="term-green">✅ Gemini API Key saved successfully! Active AI Core switched to [Google Gemini API: ${this.geminiModel}].</span>\nYou can now ask any unscripted questions (e.g. <code>ai explain how SARL uses self-attention for crowd prediction</code>).`;
    } else {
      this.activeProvider = 'local';
      return '<span class="term-amber">Gemini API Key removed. AI Core reverted to [Local Domain Expert Engine].</span>';
    }
  }

  setProvider(provider) {
    const p = (provider || '').toLowerCase().trim();
    if (!this.providers.includes(p)) {
      return `<span class="term-coral">Invalid provider: '${provider}'. Available providers: ${this.providers.map(x => `<code>${x}</code>`).join(', ')}</span>`;
    }
    this.activeProvider = p;
    if (typeof localStorage !== 'undefined') {
      localStorage.setItem('socialnav_ai_provider', p);
    }
    return `<span class="term-green">✅ Switched active AI provider to: <strong>[${p.toUpperCase()}]</strong></span>`;
  }

  setModel(model) {
    const m = (model || '').trim();
    if (!m) return '<span class="term-coral">Please specify a model name (e.g., <code>ai model gemini-2.5-flash</code> or <code>ai model llama3</code>).</span>';
    
    if (this.activeProvider === 'gemini') {
      this.geminiModel = m;
      if (typeof localStorage !== 'undefined') localStorage.setItem('socialnav_gemini_model', m);
      return `<span class="term-green">✅ Gemini Model set to: <strong>${m}</strong></span>`;
    } else if (this.activeProvider === 'ollama') {
      this.ollamaModel = m;
      if (typeof localStorage !== 'undefined') localStorage.setItem('socialnav_ollama_model', m);
      return `<span class="term-green">✅ Ollama Model set to: <strong>${m}</strong></span>`;
    }
    return `<span class="term-amber">Model set for ${this.activeProvider}: ${m}</span>`;
  }

  setOllamaEndpoint(url) {
    const u = (url || '').trim();
    if (!u) return '<span class="term-coral">Please specify Ollama URL (e.g., <code>ai ollama http://localhost:11434</code>).</span>';
    this.ollamaEndpoint = u;
    if (typeof localStorage !== 'undefined') localStorage.setItem('socialnav_ollama_endpoint', u);
    return `<span class="term-green">✅ Ollama Endpoint updated to: <strong>${u}</strong></span>`;
  }

  clearHistory() {
    this.history = [];
    this.geminiNanoSession = null;
    return '<span class="term-green">🧹 AI Conversation memory & context buffer cleared.</span>';
  }

  getStatusInfo() {
    const maskedKey = this.geminiApiKey
      ? `${this.geminiApiKey.substring(0, 7)}...${this.geminiApiKey.substring(this.geminiApiKey.length - 4)}`
      : '<span class="term-muted">Not Set (Optional)</span>';

    return `
<span class="term-cyan">=== [AI ROBOTICS ASSISTANT ENGINE STATUS] ===</span>
  • Active AI Provider : <span class="term-green">${this.activeProvider.toUpperCase()}</span>
  • Gemini API Key    : ${maskedKey}
  • Gemini Model      : <span class="term-cyan">${this.geminiModel}</span>
  • Ollama Endpoint   : ${this.ollamaEndpoint} (Model: <span class="term-cyan">${this.ollamaModel}</span>)
  • Conversation Turns: ${this.history.length / 2} messages in memory
  • Available Cores   :
    1. <span class="term-highlight">gemini</span> : Google Gemini API (Unscripted Generative LLM via free API key)
    2. <span class="term-highlight">ollama</span> : Local Ollama Server (Llama 3, DeepSeek, Mistral, Qwen)
    3. <span class="term-highlight">nano</span>   : Chrome On-Device Prompt API (Gemini Nano)
    4. <span class="term-highlight">local</span>  : Embedded Robotics Domain Expert Engine (Instant / Zero-API)

💡 <em>Quick Setup:</em>
  • Set Free Gemini Key : <code>ai key &lt;YOUR_API_KEY&gt;</code> (Get free key at aistudio.google.com)
  • Switch to Ollama    : <code>ai provider ollama</code> | <code>ai ollama http://localhost:11434</code>
  • Switch to Local Core: <code>ai provider local</code>
  • Clear Chat History  : <code>ai clear</code>
    `;
  }

  buildSystemContext(simInstance) {
    if (!simInstance || typeof simInstance.getStatus !== 'function') {
      return 'Simulation running with standard AMR robot and Social Force Model physics.';
    }

    const st = simInstance.getStatus();
    const peds = simInstance.pedestrians || [];
    const obs = simInstance.obstacles || [];
    const hits = simInstance.laserHits || [];

    return `
- Active Algorithm: ${st.algorithm.toUpperCase()}
- Active Benchmark Scenario: ${st.scenario}
- Robot Max Speed: ${st.robotSpeed} m/s | Current Linear Velocity: ${st.currentVelocity} m/s
- Robot Pose: (x: ${st.robotPose?.x || 0}m, y: ${st.robotPose?.y || 0}m, yaw: ${st.robotPose?.yawDeg || 0}°)
- Social Compliance Score: ${st.complianceScore}% | Comfort Index: ${st.comfortIndex}%
- Minimum Distance to Human: ${st.minDistanceToHuman}m (Violations count: ${st.violationsCount})
- Courtesy Weight: ${st.courtesyWeight}
- Active Pedestrians in Scene: ${peds.length} agents
- Active Static Obstacles: ${obs.length} (${obs.map(o => o.label || o.type).join(', ')})
- Active LiDAR Raycast Hits: ${hits.length} contact points
- Goal Position: (x: ${((simInstance.goal?.x || 0) / simInstance.scale).toFixed(2)}m, y: ${((simInstance.goal?.y || 0) / simInstance.scale).toFixed(2)}m)
    `.trim();
  }

  formatMarkdownToTerminal(text) {
    if (!text) return '';
    return text
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/```([a-z]*)\n([\s\S]*?)```/gi, '<pre class="term-pre"><code>$2</code></pre>')
      .replace(/`([^`]+)`/g, '<code>$1</code>')
      .replace(/\*\*([^*]+)\*\*/g, '<strong class="term-highlight">$1</strong>')
      .replace(/^### (.*$)/gim, '<span class="term-cyan">=== $1 ===</span>')
      .replace(/^## (.*$)/gim, '<span class="term-green">=== $1 ===</span>')
      .replace(/^# (.*$)/gim, '<span class="term-magenta">=== $1 ===</span>')
      .replace(/\n/g, '<br/>');
  }

  async ask(prompt, simInstance = null) {
    const rawPrompt = prompt.trim();
    if (!rawPrompt) {
      return this.getStatusInfo();
    }

    // Management sub-commands
    const parts = rawPrompt.split(/\s+/);
    const subCmd = parts[0].toLowerCase();

    if (subCmd === 'key' || subCmd === 'setkey' || subCmd === 'apikey') {
      const keyVal = parts.slice(1).join(' ').trim();
      if (keyVal === 'clear' || keyVal === 'remove' || keyVal === 'delete' || !keyVal) {
        return this.setApiKey('');
      }
      return this.setApiKey(keyVal);
    }

    if (subCmd === 'provider' || subCmd === 'backend' || subCmd === 'core') {
      const provName = parts[1];
      if (!provName) {
        return `Current AI Provider: <span class="term-green">${this.activeProvider.toUpperCase()}</span>. To change: <code>ai provider &lt;gemini|ollama|nano|local&gt;</code>`;
      }
      return this.setProvider(provName);
    }

    if (subCmd === 'model') {
      return this.setModel(parts.slice(1).join(' '));
    }

    if (subCmd === 'ollama' && parts[1]?.startsWith('http')) {
      return this.setOllamaEndpoint(parts[1]);
    }

    if (subCmd === 'clear' || subCmd === 'reset') {
      return this.clearHistory();
    }

    if (subCmd === 'status' || subCmd === 'info' || subCmd === 'engine') {
      return this.getStatusInfo();
    }

    const simContext = this.buildSystemContext(simInstance);

    // 1. Google Gemini API Provider (Live Cloud Generative AI)
    if (this.activeProvider === 'gemini') {
      if (!this.geminiApiKey) {
        return `
<span class="term-coral">⚠️ Gemini API Key is missing.</span>
To enable live unscripted Generative AI from Google Gemini:
1. Get a 100% free Gemini API Key from: <a href="https://aistudio.google.com/" target="_blank" class="term-cyan">https://aistudio.google.com/</a>
2. In terminal, type: <code>ai key &lt;YOUR_API_KEY&gt;</code>
3. Or switch to local offline engine: <code>ai provider local</code>
        `;
      }
      try {
        return await this.callGeminiAPI(rawPrompt, simContext);
      } catch (err) {
        console.error('Gemini API Error:', err);
        return `<span class="term-coral">❌ Gemini API Request Failed: ${err.message}</span>\n<span class="term-muted">Falling back to Local Robotics Expert Engine...</span>\n\n` +
          this.generateDynamicLocalResponse(rawPrompt, simInstance, simContext);
      }
    }

    // 2. Ollama Local LLM Provider (On-Premises / Offline Generative LLM)
    if (this.activeProvider === 'ollama') {
      try {
        return await this.callOllamaAPI(rawPrompt, simContext);
      } catch (err) {
        console.error('Ollama Request Failed:', err);
        return `<span class="term-coral">❌ Ollama Connection Failed (${this.ollamaEndpoint}): ${err.message}</span>\n<span class="term-muted">Make sure Ollama is running: <code>ollama run ${this.ollamaModel}</code> and CORS is enabled via <code>OLLAMA_ORIGINS="*"</code>.</span>\n\n` +
          this.generateDynamicLocalResponse(rawPrompt, simInstance, simContext);
      }
    }

    // 3. Chrome Built-in Prompt API (Gemini Nano on-device)
    if (this.activeProvider === 'nano') {
      try {
        return await this.callGeminiNano(rawPrompt, simContext);
      } catch (err) {
        console.warn('Gemini Nano failed, falling back to local expert engine:', err);
      }
    }

    // 4. Local Dynamic Domain Reasoning Engine
    return this.generateDynamicLocalResponse(rawPrompt, simInstance, simContext);
  }

  async callGeminiAPI(prompt, simContext) {
    const endpoint = `https://generativelanguage.googleapis.com/v1beta/models/${this.geminiModel}:generateContent?key=${this.geminiApiKey}`;

    const systemInstruction = {
      role: 'user',
      parts: [
        {
          text: `You are the AI Robotics & Social Navigation Expert embedded in the SOCIAL.NAV simulation platform.
You assist robotics engineers with real-time navigation analysis, Hall's Proxemics, Social Force Model (SFM), Socially Attentive Reinforcement Learning (SARL), Social Model Predictive Control (MPC), Optimal Reciprocal Collision Avoidance (ORCA), ROS2 bridge telemetry, 3D Gazebo/Ignition simulation, and benchmark crowd datasets (ETH, UCY, JRDB, SCAND).

REAL-TIME SIMULATION TELEMETRY & WORLD STATE:
${simContext}

Respond concisely, with technical rigor and mathematical clarity. Format code and commands with markdown backticks. Answer in the same language as the user's question (Vietnamese or English).`
        }
      ]
    };

    const contents = [
      ...this.history.map(msg => ({
        role: msg.role === 'assistant' ? 'model' : 'user',
        parts: [{ text: msg.content }]
      })),
      {
        role: 'user',
        parts: [{ text: prompt }]
      }
    ];

    const payload = {
      system_instruction: systemInstruction,
      contents: contents,
      generationConfig: {
        temperature: 0.7,
        maxOutputTokens: 900
      }
    };

    const res = await fetch(endpoint, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });

    if (!res.ok) {
      const errData = await res.json().catch(() => ({}));
      const msg = errData?.error?.message || `HTTP ${res.status} ${res.statusText}`;
      throw new Error(msg);
    }

    const data = await res.json();
    const replyText = data?.candidates?.[0]?.content?.parts?.[0]?.text || "No response generated.";

    this.history.push({ role: 'user', content: prompt });
    this.history.push({ role: 'assistant', content: replyText });
    if (this.history.length > 16) {
      this.history = this.history.slice(-16);
    }

    return `<span class="term-green">🤖 [Google Gemini (${this.geminiModel})]:</span>\n${this.formatMarkdownToTerminal(replyText)}`;
  }

  async callOllamaAPI(prompt, simContext) {
    const endpoint = `${this.ollamaEndpoint}/api/chat`;

    const messages = [
      {
        role: 'system',
        content: `You are the AI Robotics & Social Navigation Expert embedded in the SOCIAL.NAV simulation platform.\nLive Simulation State:\n${simContext}\nAnswer concisely, mathematically accurate, in the same language as the user.`
      },
      ...this.history.map(msg => ({
        role: msg.role,
        content: msg.content
      })),
      {
        role: 'user',
        content: prompt
      }
    ];

    const res = await fetch(endpoint, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        model: this.ollamaModel,
        messages: messages,
        stream: false
      })
    });

    if (!res.ok) {
      throw new Error(`HTTP ${res.status} (Ensure Ollama is running and CORS is enabled via OLLAMA_ORIGINS="*")`);
    }

    const data = await res.json();
    const replyText = data?.message?.content || data?.response || "No response generated.";

    this.history.push({ role: 'user', content: prompt });
    this.history.push({ role: 'assistant', content: replyText });
    if (this.history.length > 16) {
      this.history = this.history.slice(-16);
    }

    return `<span class="term-cyan">🤖 [Ollama Local LLM (${this.ollamaModel})]:</span>\n${this.formatMarkdownToTerminal(replyText)}`;
  }

  async callGeminiNano(prompt, simContext) {
    if (!window.ai || !window.ai.languageModel) {
      throw new Error('Chrome Prompt API is not available on this browser.');
    }

    if (!this.geminiNanoSession) {
      const sysPrompt = `You are the AI Social Navigation & Robotics Expert embedded in the SOCIAL.NAV simulation studio.
Current Simulation Status:
${simContext}
Answer concisely, technically accurate, and professionally in the same language as the question (Vietnamese or English).`;

      this.geminiNanoSession = await window.ai.languageModel.create({
        systemPrompt: sysPrompt
      });
    }

    const answer = await this.geminiNanoSession.prompt(prompt);
    return `<span class="term-green">🤖 [Gemini Nano (On-Device NPU/GPU)]:</span>\n${this.formatMarkdownToTerminal(answer)}`;
  }

  generateDynamicLocalResponse(prompt, simInstance, simContext) {
    const lower = prompt.toLowerCase();
    const st = simInstance && typeof simInstance.getStatus === 'function' ? simInstance.getStatus() : null;

    // 1. Benchmark Datasets Dynamic Analysis
    if (lower.includes('dataset') || lower.includes('dữ liệu') || lower.includes('eth') || lower.includes('ucy') || lower.includes('jrdb') || lower.includes('scand')) {
      const currentScen = st ? st.scenario : 'synthetic';
      const pedCount = st ? st.pedestrianCount : 6;
      return `
<span class="term-cyan">🤖 [Local Domain Expert] • Benchmark Pedestrian Datasets Engine:</span>

Currently active scenario in your simulation: <span class="term-green"><strong>${currentScen}</strong></span> (Tracking <strong>${pedCount}</strong> active pedestrian agents).

The playground embeds 4 real-world trajectory datasets with exact metric spatial conversions and timestamps:
1. <span class="term-green">ETH Zurich (ETH / Hotel)</span>:
   • Source: Pellegrini et al., ICCV 2009.
   • Scene: Real campus & hotel plaza recordings. Low-to-moderate density, linear walking behaviors.
   • Command: <code>sim scenario eth_univ</code> or <code>sim scenario eth_hotel</code>
2. <span class="term-cyan">UCY Crowds (Zara01 / Zara02 / Univ)</span>:
   • Source: Lerner et al., Eurographics 2007.
   • Scene: Crowded shopping avenue in Cyprus. Features non-linear group dynamics, chatting pairs, and sudden turns.
   • Command: <code>sim scenario ucy_zara</code>
3. <span class="term-amber">Stanford JRDB 2021 (Jackal Real World)</span>:
   • Source: Martín-Martín et al., CVPR 2021.
   • Scene: Jackal AMR navigating outdoors at Stanford Quad among pedestrians, skateboards, and cyclists.
   • Command: <code>sim scenario jrdb_quad</code>
4. <span class="term-coral">UT Austin SCAND 2022 (Socially Compliant Navigation)</span>:
   • Source: Karnan et al., RA-L 2022.
   • Scene: Over 8.7 hours of Spot and Jackal navigation across UT Austin speedway plaza.
   • Command: <code>sim scenario scand_plaza</code>

💡 <em>Tip: To enable full generative AI reasoning for custom research questions, connect a free Gemini key with <code>ai key &lt;YOUR_KEY&gt;</code> or Ollama with <code>ai provider ollama</code>!</em>
      `;
    }

    // 2. Real-time Scene Telemetry Evaluation
    if (lower.includes('analyze') || lower.includes('phân tích') || lower.includes('hiện tại') || lower.includes('scene')) {
      if (!st) return '<span class="term-coral">No simulation telemetry available.</span>';
      
      const { algorithm, scenario, robotSpeed, currentVelocity, pedestrianCount, complianceScore, minDistanceToHuman, violationsCount, comfortIndex } = st;
      let safetyAssessment = '<span class="term-green">EXCELLENT SOCIAL COMPLIANCE</span>';
      let recs = [];

      if (minDistanceToHuman < 0.6) {
        safetyAssessment = '<span class="term-coral">WARNING: INTIMATE ZONE BREACH (&lt; 0.6m)</span>';
        recs.push('• Increase courtesy weight: <code>sim courtesy 1.2</code> to trigger earlier deceleration.');
        recs.push('• Switch to Social MPC (<code>sim algo mpc</code>) for 10-step predictive horizon.');
      } else if (minDistanceToHuman < 1.2) {
        safetyAssessment = '<span class="term-amber">MODERATE: CURRENTLY IN PERSONAL ZONE (0.6m - 1.2m)</span>';
        recs.push('• Maintain safe passing margin &gt; 1.2m.');
      }

      if (pedestrianCount >= 8) {
        recs.push(`• High crowd density (${pedestrianCount} agents): SARL (Deep RL) or Social MPC recommended.`);
      }
      if (recs.length === 0) {
        recs.push('• Navigation trajectory is optimal. No behavioral intervention needed.');
      }

      return `
<span class="term-cyan">=== [AI REAL-TIME SCENE TELEMETRY ANALYSIS] ===</span>
  • Safety Assessment  : ${safetyAssessment}
  • Navigation Model   : <span class="term-green">${algorithm.toUpperCase()}</span> (Scenario: <strong>${scenario}</strong>)
  • Social Compliance  : <span class="term-green">${complianceScore}%</span> (Violations: ${violationsCount})
  • Minimum Distance   : <span class="term-cyan">${minDistanceToHuman} m</span> | Comfort Score: <span class="term-green">${comfortIndex}%</span>
  • Linear Velocity    : ${currentVelocity} m/s (Max: ${robotSpeed} m/s) | Crowd Count: ${pedestrianCount}

<span class="term-muted">📋 DYNAMIC RECOMMENDATIONS:</span>
${recs.join('\n')}
      `;
    }

    // 3. Hall's Proxemics Theory
    if (lower.includes('proxemic') || lower.includes('hall') || lower.includes('khoảng cách') || lower.includes('cự ly') || lower.includes('zone')) {
      return `
<span class="term-cyan">🤖 [Local Domain Expert] • Hall's Proxemics in Autonomous Social Navigation:</span>

Anthropologist Edward T. Hall (1966) formulated 4 discrete spatial zones around humans:
1. <span class="term-coral">Intimate Zone (0.00m – 0.45m)</span>: Reserved for close contact. AMR intrusion causes extreme discomfort ($C_{cost} = 100$).
2. <span class="term-amber">Personal Zone (0.45m – 1.20m)</span>: Personal space for acquaintances. Intrusion without signaling causes anxiety ($C_{cost} = 75$).
3. <span class="term-green">Social Zone (1.20m – 3.60m)</span>: Standard zone for social interactions and robot passing maneuvers.
4. <span class="term-cyan">Public Zone (&gt; 3.60m)</span>: Public area where robots can safely travel at maximum kinematic velocity.

📐 <em>Anisotropic Cost Formulation:</em>
<code>Cost(x, y) = A · exp(-0.5 · ((x'/σ_front)² + (y'/σ_side)²))</code> with $\\sigma_{front} > \\sigma_{side}$ to reflect forward directional velocity attention.
      `;
    }

    // 4. Algorithm Breakdown
    if (lower.includes('sfm') || lower.includes('sarl') || lower.includes('drl') || lower.includes('mpc') || lower.includes('orca') || lower.includes('algorithm') || lower.includes('thuật toán')) {
      return `
<span class="term-cyan">🤖 [Local Domain Expert] • Navigation Algorithms Architecture:</span>

• <span class="term-green">Social Force Model (SFM - Helbing)</span>:
  Classical physics-based potential fields: $\\vec{F}_{total} = \\vec{F}_{goal} + \\sum \\vec{F}_{ped} + \\sum \\vec{F}_{obs}$. Extremely fast ($&lt;1\\text{ms}$), reactive, but prone to local minima in dense crowds.

• <span class="term-cyan">Socially Attentive RL (SARL - Chen et al.)</span>:
  Deep Reinforcement Learning with Self-Attention mechanism over crowd joint states. Excels at anticipating collective human movements ($98.4\\%$ compliance score).

• <span class="term-amber">Social MPC (Model Predictive Control)</span>:
  Receding horizon optimization over $N=10$ preview steps ($2.5\\text{s}$). Balances differential-drive smoothness with anisotropic Proxemic constraints.

• <span class="term-coral">ORCA (Reciprocal Velocity Obstacles)</span>:
  Collision avoidance sharing $50\\% - 50\\%$ reciprocal responsibility between moving entities.

💡 <em>Switch algorithm live: <code>sim algo &lt;sfm|sarl|mpc|orca&gt;</code></em>
      `;
    }

    // 5. Why slow down / stop
    if (lower.includes('why') || lower.includes('slow') || lower.includes('tại sao') || lower.includes('chậm') || lower.includes('dừng') || lower.includes('stop') || lower.includes('stuck')) {
      const minD = st ? st.minDistanceToHuman : 2.0;
      const courtesy = st ? st.courtesyWeight : 0.8;
      const speed = st ? st.currentVelocity : 0;
      return `
<span class="term-cyan">🤖 [Local Domain Expert] • Robot Kinematics & Deceleration Diagnostics:</span>

Live Telemetry: Velocity = <strong>${speed} m/s</strong>, Nearest human = <span class="term-cyan">${minD} m</span>, Courtesy weight = <strong>${courtesy}</strong>.

Primary factors governing velocity throttling:
1. <strong>Proxemic Cost Gradient</strong>: As humans enter the Personal Zone ($&lt;1.2\\text{m}$), the repulsive potential force $F_{rep}$ opposes forward motion.
2. <strong>Courtesy Weight (${courtesy})</strong>: Higher courtesy forces the robot to yield right-of-way when crossing human paths.
3. <strong>Goal Approach Deceleration</strong>: Kinematic ramping smoothly brakes the AMR as distance to goal becomes $&lt;1.5\\text{m}$.

💡 <em>Quick adjustments:</em>
• Faster overtaking: <code>sim courtesy 0.2</code>
• Higher top speed: <code>sim speed 2.2</code>
      `;
    }

    // 6. ROS2 and RViz2
    if (lower.includes('ros2') || lower.includes('rviz') || lower.includes('topic') || lower.includes('bridge') || lower.includes('subscriber')) {
      return `
<span class="term-cyan">🤖 [Local Domain Expert] • ROS2 Bridge & RViz2 Setup:</span>

1. <strong>Start ROSBridge WebSocket Server</strong>:
   <code>cd ros2_bridge && ./launch_rosbridge.sh</code> (Port: <code>ws://localhost:9090</code>)
2. <strong>Launch RViz2 Visualizer</strong>:
   <code>./launch_visualizer.sh</code>
3. <strong>Live ROS2 Topics Published</strong>:
   • <code>/scan</code>: <code>sensor_msgs/LaserScan</code> (360-degree LiDAR raycast)
   • <code>/odom</code>: <code>nav_msgs/Odometry</code> (Jackal AMR pose & linear/angular velocity)
   • <code>/social_costmap</code>: <code>nav_msgs/OccupancyGrid</code> (2D Proxemics costmap)
   • <code>/tracked_humans</code>: <code>geometry_msgs/PoseArray</code> (Real-time pedestrian poses)

💡 <em>Verify topics in bash terminal: <code>ros2 topic list</code> or <code>ros2 topic echo /scan</code></em>
      `;
    }

    // 7. General Fallback
    return `
<span class="term-cyan">🤖 [Local Domain Expert]:</span>
Question received: "<em>${prompt}</em>"

Currently using the offline local domain expert engine.
• To enable live unscripted Generative AI with Google Gemini:
  1. Get a free API key at <a href="https://aistudio.google.com/" target="_blank" class="term-cyan">https://aistudio.google.com/</a>
  2. Type: <code>ai key &lt;YOUR_API_KEY&gt;</code>
• To use a local LLM via Ollama:
  1. Start Ollama and type: <code>ai provider ollama</code>
• Real-time scene analysis: <code>ai analyze</code>
• Check AI settings: <code>ai status</code>
    `;
  }
}

export const aiAssistantInstance = new SocialNavAIAssistant();

