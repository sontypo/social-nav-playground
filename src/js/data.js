// Simulation Theory, Proxemics Formulations, ROS2 Topic Metadata & CLI Knowledge Base

export const simTheoryData = {
  algorithms: [
    {
      id: "sfm",
      name: "Social Force Model (SFM)",
      badge: "Physics-Based",
      author: "Helbing & Molnár (1995)",
      description: "Models human crowd and robot dynamics through virtual physical forces: attractive destination forces, interpersonal repulsive forces modulated by anisotropic vision, and barrier repulsion.",
      equation: "\\mathbf{f}_i(t) = \\frac{\\mathbf{v}_i^0 - \\mathbf{v}_i}{\\tau_i} + \\sum_{j \\neq i} \\mathbf{f}_{ij}^{soc} + \\sum_W \\mathbf{f}_{iW}^{obs}",
      details: [
        "Repulsive force: f_ij = A * exp((r_ij - d_ij) / B) * n_ij * w(phi)",
        "Anisotropy factor w(phi) = lambda + (1 - lambda) * (1 + cos(phi)) / 2 gives directional field of view",
        "Includes social right-hand passing bias (courtesy vector)"
      ],
      codeSnippet: `// Helbing Social Force Model Calculation
Eigen::Vector2d f_dest = (target_vel - current_vel) / relaxation_time;
Eigen::Vector2d f_repulsive(0, 0);

for (const auto& human : crowd) {
    Eigen::Vector2d diff = robot_pos - human.pos;
    double dist = diff.norm();
    double overlap = (robot_radius + human.radius) - dist;
    Eigen::Vector2d n_ij = diff.normalized();
    
    // Anisotropic vision weighting
    double cos_phi = -n_ij.dot(robot_vel.normalized());
    double w_phi = lambda + (1.0 - lambda) * (1.0 + cos_phi) / 2.0;
    
    f_repulsive += A * std::exp(overlap / B) * n_ij * w_phi;
}`
    },
    {
      id: "sarl",
      name: "Self-Attention Relational Graph DRL (SARL)",
      badge: "Deep RL (GNN)",
      author: "Chen et al. (ICRA / RSS)",
      description: "Uses a spatial-temporal graph neural network with self-attention to model pairwise human-human and human-robot topological interactions, anticipating future collision states 1.5s - 2.5s ahead.",
      equation: "V(s) = \\mathrm{MLP}\\left( [s_{rob}, \\sum_{i=1}^N \\alpha_i \\cdot \\psi(s_{rob}, s_{hum}^i)] \\right), \\quad \\alpha_i = \\mathrm{Softmax}\\left( \\frac{Q K^T}{\\sqrt{d_k}} \\right)",
      details: [
        "Self-attention dynamically weights critical pedestrians in robot's travel cone",
        "Trained via Deep Reinforcement Learning (PPO / SAC) with social comfort penalties",
        "Eliminates freezing robot problem in high crowd densities"
      ],
      codeSnippet: `# PyTorch SARL Attention Value Network
class RelationalGraphValueNetwork(nn.Module):
    def forward(self, robot_state, human_states):
        # robot_state: [B, 6] (px, py, vx, vy, radius, gx, gy)
        # human_states: [B, N, 5] (px, py, vx, vy, radius)
        robot_emb = self.robot_mlp(robot_state)
        pairwise_emb = self.joint_mlp(torch.cat([robot_state.unsqueeze(1).repeat(1, N, 1), human_states], dim=-1))
        
        # Multi-head Self Attention
        attn_weights = F.softmax(self.attn(pairwise_emb), dim=1)
        crowd_context = torch.sum(attn_weights * pairwise_emb, dim=1)
        
        value = self.value_head(torch.cat([robot_emb, crowd_context], dim=-1))
        return value, attn_weights`
    },
    {
      id: "cadrl",
      name: "MIT CADRL (Collision Avoidance DRL)",
      badge: "Deep RL",
      author: "Chen, Everett, How (MIT)",
      description: "Reciprocal collision avoidance with deep reinforcement learning. Learns value functions over pairwise interaction spaces, naturally discovering right-of-way passing and respectful yielding.",
      equation: "\\pi^*(s) = \\arg\\max_{\\mathbf{a} \\in \\mathcal{A}} \\left[ R(s, \\mathbf{a}) + \\gamma V^*(s') \\right]",
      details: [
        "Value network parameterized over relative coordinates",
        "Enforces reciprocal velocity concessions between autonomous agents",
        "Fast sub-millisecond inference suitable for micro-controllers"
      ],
      codeSnippet: `// CADRL Action Selection
std::vector<Action> admissible_actions = sample_velocity_space(robot_max_speed);
Action best_action;
double max_value = -1e9;

for (const auto& a : admissible_actions) {
    State s_next = kinematic_step(robot_state, a, dt);
    double r = compute_step_reward(s_next, humans, goal);
    double v = value_network->evaluate(s_next, humans);
    if (r + gamma * v > max_value) {
        max_value = r + gamma * v;
        best_action = a;
    }
}`
    },
    {
      id: "social_mpc",
      name: "Social Model Predictive Control (Social-MPC)",
      badge: "Optimal Control",
      author: "Receding Horizon Optimization",
      description: "Formulates social navigation as a finite receding-horizon constrained optimization problem, penalizing deviations from preferred paths, proxemics violations, and control jerk.",
      equation: "\\min_{\\mathbf{u}_{0:N-1}} \\sum_{k=0}^{N-1} \\left( \\|x_k - x_{goal}\\|_{Q}^2 + \\|u_k\\|_{R}^2 + \\mathcal{C}_{prox}(x_k, \\hat{x}_{hum,k}) \\right)",
      details: [
        "12-step predictive horizon incorporates predicted pedestrian trajectory spline",
        "Smooth acceleration and angular velocity profile for ride comfort",
        "Explicit safety constraints through barrier functions"
      ],
      codeSnippet: `// Social MPC Cost Function (CasADi / ACADOS)
Function cost_fn = [=](const Trajectory& x_traj, const Control& u_traj) {
    MX J = 0;
    for (int k = 0; k < N; ++k) {
        J += (x_traj[k].pos - goal).squaredNorm() * w_goal;
        J += u_traj[k].squaredNorm() * w_control;
        for (const auto& h_pred : human_predictions) {
            double dist = (x_traj[k].pos - h_pred[k].pos).norm();
            J += w_social * exp(-0.5 * pow(dist / sigma_p, 2));
        }
    }
    return J;
};`
    },
    {
      id: "orca_social",
      name: "Social-ORCA (Reciprocal Velocity Obstacles)",
      badge: "Geometric",
      author: "van den Berg et al. (UNC Chapel Hill)",
      description: "Computes optimal collision-free velocities using geometric half-planes in 2D velocity space, augmented with asymmetric social boundary constraints.",
      equation: "\\mathrm{ORCA}_{A|B}^\\tau = \\left\\{ \\mathbf{v} \\;\\Big|\\; \\left( \\mathbf{v} - \\left(\\mathbf{v}_A + \\frac{1}{2}\\mathbf{u}\\right) \\right) \\cdot \\mathbf{n} \\ge 0 \\right\\}",
      details: [
        "Guaranteed collision-free under reciprocal assumption",
        "Linear programming optimization solves for velocity in microseconds",
        "Modified with asymmetric comfort elliptical cones"
      ],
      codeSnippet: `// Social-ORCA Half-Plane Linear Programming
std::vector<Line> orca_lines;
for (const auto& other : agents) {
    Line line = compute_orca_halfplane(robot, other, time_horizon, inv_time_step);
    orca_lines.push_back(line);
}
Vector2 optimal_vel = linear_program_2d(orca_lines, robot_pref_vel, robot_max_speed);`
    },
    {
      id: "nonsocial",
      name: "Non-Social A* / Naive DWA",
      badge: "Baseline",
      author: "Traditional Navigation",
      description: "Treats human pedestrians as static circular obstacles without recognizing social zones, heading direction, or communicative intention.",
      equation: "f(n) = g(n) + h(n), \\quad \\text{Costmap} = \\begin{cases} \\infty & \\text{if } d < r_{col} \\\\ 0 & \\text{otherwise} \\end{cases}",
      details: [
        "Causes frequent freezing robot problem and intrusive personal space breaches",
        "Serves as benchmark baseline to quantify social navigation improvements"
      ],
      codeSnippet: `// Non-Social Baseline: Standard Static Circular Inflation
double dist = (robot_pos - human_pos).norm();
if (dist < robot_radius + human_radius + safety_margin) {
    costmap.set_cost(x, y, LETHAL_OBSTACLE);
} else {
    costmap.set_cost(x, y, FREE_SPACE);
}`
    }
  ],

  proxemicsTheory: {
    title: "Edward T. Hall's Proxemics & Asymmetric Gaussian Fields",
    zones: [
      {
        name: "Intimate Space",
        radius: "< 0.45 meters",
        color: "#ff0055",
        description: "Reserved for close personal contact. Entering this zone causes acute discomfort and social violation."
      },
      {
        name: "Personal Space",
        radius: "0.45 m – 1.20 meters",
        color: "#f59e0b",
        description: "Conversational buffer distance. Robot must navigate around this boundary with courteous yielding."
      },
      {
        name: "Social Space",
        radius: "1.20 m – 3.60 meters",
        color: "#00e5ff",
        description: "Interaction zone where robot trajectory intention and passing side are communicated visually."
      },
      {
        name: "Public Space",
        radius: "> 3.60 meters",
        color: "#00ff9d",
        description: "General environment where normal global path planning proceeds without social penalty."
      }
    ],
    gaussianFormula: "\\mathcal{C}(x, y) = A \\cdot \\exp\\left( -\\frac{1}{2} \\left[ \\left(\\frac{x'}{\\sigma_{front}(v)}\\right)^2 + \\left(\\frac{y'}{\\sigma_{side}}\\right)^2 \\right] \\right), \\quad \\sigma_{front}(v) = \\sigma_0 \\cdot (1 + \\beta \\cdot v)"
  },

  benchmarks: [
    { name: "Non-Social A* / DWA", ade: "1.12m", fde: "2.45m", compliance: "61.2%", violations: "38.4%", comfort: "58.0%" },
    { name: "Social Force (SFM)", ade: "0.65m", fde: "1.32m", compliance: "82.4%", violations: "14.2%", comfort: "81.5%" },
    { name: "Social-ORCA (RVO)", ade: "0.54m", fde: "1.10m", compliance: "89.1%", violations: "8.6%", comfort: "88.2%" },
    { name: "CADRL (MIT DRL)", ade: "0.44m", fde: "0.89m", compliance: "94.3%", violations: "4.2%", comfort: "93.0%" },
    { name: "Social MPC (Receding)", ade: "0.41m", fde: "0.78m", compliance: "96.8%", violations: "2.9%", comfort: "96.4%" },
    { name: "Relational Graph DRL (SARL)", ade: "0.38m", fde: "0.72m", compliance: "98.6%", violations: "1.8%", comfort: "98.1%" }
  ],

  datasetInfo: [
    { id: "eth_univ", name: "ETH Univ", venue: "ICCV 2009 (CVL ETH Zurich)", env: "University Main Entrance", type: "Classical 2D Overhead", features: "Crossing diagonal streams, group dynamics, yielding" },
    { id: "eth_hotel", name: "ETH Hotel", venue: "ICCV 2009 (CVL ETH Zurich)", env: "Tram Station Sidewalk", type: "Classical 2D Overhead", features: "High-density crowd, waiting zones, entering/exiting" },
    { id: "ucy_zara", name: "UCY Zara-01 & 02", venue: "Eurographics 2007 (Univ of Cyprus)", env: "Urban Shopping Street", type: "Classical 2D Overhead", features: "Dense bidirectional flow, window shoppers, group pacing" },
    { id: "jrdb_quad", name: "Stanford JRDB Quad", venue: "CVPR 2021 / TPAMI 2023 (Stanford & Monash)", env: "Outdoor Quad Plaza", type: "Social Robot 3D LiDAR & 360° Vision", features: "JackRabbot mobile robot, standing clusters, skateboarders" },
    { id: "jrdb_atrium", name: "Stanford JRDB CS Atrium", venue: "CVPR 2021 (Stanford University)", env: "Indoor Gates CS Building", type: "Social Robot Indoor 3D LiDAR", features: "Indoor corridors, lounge seating clusters, atrium flow" },
    { id: "scand_plaza", name: "UT Austin SCAND", venue: "IEEE RA-L 2022 (UT Austin)", env: "Campus Pedestrian Plaza", type: "Spot & Jackal AMR Navigation", features: "Human demonstration paths, quadruped/wheeled robot avoidance" },
    { id: "thor_mocap", name: "University of Lincoln THÖR", venue: "2019 (Univ of Lincoln)", env: "Shared Space MoCap Lab", type: "3D Optical MoCap & LiDAR", features: "Human-robot reciprocal yielding in 2.2m shared corridor" },
    { id: "atc_mall", name: "ATR ATC Shopping Mall", venue: "ATR IRC Labs Kyoto", env: "Asia Pacific Trade Center", type: "Fixed 3D LiDAR Network", features: "Over 100,000 shoppers, wandering paths, store entries" },
    { id: "sdd_coupa", name: "Stanford Drone Dataset (SDD)", venue: "ECCV 2016 (Stanford SVL)", env: "Coupa Cafe Plaza", type: "4K Drone Aerial", features: "Multi-modal agents: pedestrians, bicycles, skateboards, carts" },
    { id: "ind_urban", name: "RWTH Aachen inD", venue: "LevelX 2020 (RWTH Aachen)", env: "Urban Roundabout & Intersection", type: "Drone Metric Trajectories", features: "Shared space pedestrian-vehicle-cyclist interactions" }
  ],

  terminalHelp: `Available Terminal Commands:
  • <span class="term-highlight">ai &lt;question&gt;</span>                                    : Ask AI Robotics Assistant (Zero-API: Gemini Nano / Local Expert)
  • <span class="term-highlight">ai analyze</span>                                          : Real-time telemetry evaluation & navigation recommendations
  • <span class="term-highlight">ai status</span>                                           : Show active AI Core & privacy metrics
  • <span class="term-highlight">sim algo &lt;sfm|sarl|cadrl|mpc|orca|nonsocial&gt;</span> : Switch active motion planning algorithm
  • <span class="term-highlight">sim scenario &lt;name&gt;</span>                            : Load benchmark scenario (e.g. eth_univ, jrdb_quad, scand_plaza, thor_mocap, atc_mall, ucy_zara, etc.)
  • <span class="term-highlight">sim peds &lt;count&gt;</span>                                  : Set pedestrian crowd density (2 - 20)
  • <span class="term-highlight">sim speed &lt;m/s&gt;</span>                                    : Set robot maximum speed (0.4 - 3.0 m/s)
  • <span class="term-highlight">sim courtesy &lt;0.1 - 2.0&gt;</span>                           : Set yielding / social courtesy weight
  • <span class="term-highlight">sim spawn &lt;x&gt; &lt;y&gt;</span>                                  : Spawn human at coordinate (e.g. sim spawn 300 200)
  • <span class="term-highlight">sim pillar &lt;x&gt; &lt;y&gt;</span>                                 : Add static obstacle pillar
  • <span class="term-highlight">sim pause</span> | <span class="term-highlight">sim resume</span> | <span class="term-highlight">sim reset</span>    : Control simulation execution
  • <span class="term-highlight">sim lidar &lt;on|off|rays N|range M|fov D&gt;</span>            : Toggle & configure LiDAR (e.g. sim lidar rays 180, sim lidar fov 270)
  • <span class="term-highlight">sim heatmap &lt;on|off&gt;</span>                               : Toggle Hall's Proxemics Gaussian heatmap
  • <span class="term-highlight">sim status</span>                                          : Show live simulation telemetry & metrics
  • <span class="term-highlight">ros2 topic list</span>                                     : List all active ROS2 published topics
  • <span class="term-highlight">ros2 topic echo &lt;topic&gt;</span>                             : Stream one frame of ROS2 topic data
  • <span class="term-highlight">theory &lt;algo|proxemics|benchmarks|datasets&gt;</span>         : View mathematical formulations & public datasets
  • <span class="term-highlight">analytics &lt;stats|pause|resume|clear|csv|json&gt;</span>       : Inspect telemetry statistics & export CSV/JSON logs
  • <span class="term-highlight">theme &lt;theme_id&gt;</span>                                  : Switch UI studio theme (tokyo, obsidian, solar_light, etc.)
  • <span class="term-highlight">clear</span>                                               : Clear terminal screen output`
};
