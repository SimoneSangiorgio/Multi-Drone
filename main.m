% main.m
%
% DESCRIPTION:
% This script sets up and runs a simulation of N UAVs transporting a suspended
% payload. The system navigates from a starting point to a goal, avoiding
% static obstacles using a Control Barrier Function (CBF) approach.
%
% The script defines system parameters, initial conditions, mission waypoints,
% and obstacles. After running the simulation using ODE45, it performs
% post-processing to analyze performance and check for collisions.

% --- START OF FILE main.m ---

clear; close all; clc;


%% -------------------- Important Parameters --------------------

p.N    = 3;             % Number of UAVs
p.L     = 2.0;            % Nominal length of payload cables [m]
p.Delta = 1.0;          % Desired distance between adjacent UAVs [m]

% Key mission waypoints
p.payload_start_pos = [1; 0.5; 3];
p.payload_goal_pos  = [5; 5; 3];

% Controller gains for the collective motion
p.Kp_stabilize  = 0.2; % Proportional gain for stabilization phases
p.Kp_trajectory = 0.2; % Proportional gain for trajectory tracking


p.max_speed     = 0.4; % Maximum speed for the formation's center of mass [m/s]
p.stabilization_duration = 2.0; % Time to wait once stabilized at a waypoint [s]

p.cbf_influence_distance = 1.5; % Multiplier on obstacle radius to define the "awareness zone"
p.gamma_cbf_obs   = 0.6;      % Aggressiveness of the obstacle avoidance maneuver

positioning_error_magnitude = 0.1; % Add random noise to initial positions

tspan = 0:0.05:150; % Simulation time


%% -------------------- System Parameters --------------------
% Physical properties of the system
p.m    = 0.5;           % UAV mass [kg]
p.mL   = 1.0;           % Payload mass [kg]
p.g    = [0; 0; -9.81]; % Gravity acceleration vector [m/s^2]

% Control & Cable Parameters
p.k     = 10;           % Stiffness of virtual springs between UAVs (tensegrity)
p.gamma = 100;          % Stiffness of payload cables (Hooke's law) [N/m]

% Disturbance: Constant wind force on UAVs (inactive for this simulation)
p.wind = [0; 0; 0];      % Wind disturbance force [N]


%% -------------------- State and Formation Geometry --------------------
% Define state vector dimensions and the geometric structure of the formation.
nAgents = p.N + 1;      % Total agents (N UAVs + 1 Payload)
npos    = 3 * nAgents;  % Total position states (3 per agent)
nvel    = npos;         % Total velocity states (3 per agent)
ndelta  = 3 * p.N;      % Total integral action states (3 per UAV)

% 1. Generate desired relative position vectors (p.rd) for the UAV formation
% The UAVs form a closed polygon in the horizontal plane.
formation_radius = (p.Delta / 2) / sin(pi / p.N); % Radius of the polygon's circumscribed circle

p.T = p.N; % Number of tensegrity links (a closed polygon has N links)
p.rd = zeros(3 * p.T, 1);
for i = 1:p.N
    % Calculate angular positions for UAV 'i' and the next one in the polygon
    angle_i    = (i - 1) * 2 * pi / p.N;
    angle_next = i * 2 * pi / p.N;

    % Calculate the 3D positions in the formation's local frame
    pos_i    = formation_radius * [cos(angle_i); sin(angle_i); 0];
    pos_next = formation_radius * [cos(angle_next); sin(angle_next); 0];

    % The desired relative vector 'rd' for link 'i' points from UAV 'i' to 'i+1'
    idx = (i - 1) * 3 + 1 : i * 3;
    p.rd(idx) = pos_next - pos_i;
end

% 2. Generate the Incidence Matrix (p.D)
% This matrix maps forces from links/cables to the agents (UAVs/payload).
p.E = p.T + p.N; % Total connections: T tensegrity links + N payload cables
D_tensegrity = zeros(3 * (p.N + 1), 3 * p.T);
for l = 1:p.T % Iterate over tensegrity links
    i = l;                  % Start node of link 'l'
    j = mod(l, p.N) + 1;    % End node of link 'l'
    
    idx_col   = (l - 1) * 3 + 1 : l * 3;
    idx_row_i = (i - 1) * 3 + 1 : i * 3;
    idx_row_j = (j - 1) * 3 + 1 : j * 3;
    
    % The matrix applies forces in opposite directions to the connected UAVs
    D_tensegrity(idx_row_i, idx_col) = eye(3);
    D_tensegrity(idx_row_j, idx_col) = -eye(3);
end

D_cables = zeros(3 * (p.N + 1), 3 * p.N);
for i = 1:p.N % Iterate over payload cables
    idx_col      = (i - 1) * 3 + 1 : i * 3;
    idx_row_uav  = (i - 1) * 3 + 1 : i * 3;
    idx_row_load = p.N * 3 + 1 : (p.N + 1) * 3;
    
    % Connects each UAV 'i' to the single payload
    D_cables(idx_row_uav, idx_col)  = eye(3);
    D_cables(idx_row_load, idx_col) = -eye(3);
end

p.D = [D_tensegrity, D_cables]; % Combine into the final incidence matrix

%% -------------------- Mission Parameters --------------------
p.floor_z = 0; % Ground level is at z=0

% Thresholds for advancing mission phases
p.formation_pos_threshold     = 0.1;  % Max total distance error for formation shape
p.formation_vel_threshold     = 0.05; % Max speed of any UAV for formation to be considered stable
p.stabilization_pos_threshold = 0.1;  % Max distance to waypoint for stabilization
p.stabilization_vel_threshold = 0.1;  % Max speed of payload for stabilization
p.goal_pos_threshold          = 0.1;  % Max distance to goal to consider it "reached"

%% -------------------- Obstacle & Floor Avoidance Parameters --------------------
p.nObs = 3;
p.obstacles = cell(p.nObs, 1);

%p.obstacles{1}.type = 'cylinder';
%p.obstacles{1}.base_center = [7; 7; p.floor_z];
%p.obstacles{1}.radius = 0.5;
%p.obstacles{1}.height = 10;

p.obstacles{1}.type = 'sphere';
p.obstacles{1}.center = [2; 2; 3]; 
p.obstacles{1}.radius = 0.3;

p.obstacles{2}.type = 'cylinder';
p.obstacles{2}.base_center = [4; 2; p.floor_z];
p.obstacles{2}.radius = 0.5;
p.obstacles{2}.height = 6;

p.obstacles{3}.type = 'sphere';
p.obstacles{3}.center = [4.5; 4; 3]; 
p.obstacles{3}.radius = 0.4;

% Control Barrier Function (CBF) gains
p.gamma_cbf_floor = 1.0;      % Gain for the floor avoidance barrier function
p.uav_radius      = 0.1;      % Physical radius of each UAV for collision checks [m]


%% -------------------- Initial Conditions --------------------
% This block robustly sets the initial positions of UAVs on the floor,
% ensuring a stable orientation regardless of the number of UAVs (p.N).

eta0 = zeros(npos, 1);
payload_initial_pos = [0; 0; p.floor_z]; 

% --- Stable Initial Positioning Logic ---

% 1. Calculate the final aerial formation radius.
formation_radius_final = (p.Delta / 2) / sin(pi / p.N);

%    Check for geometric feasibility.
if formation_radius_final > p.L
    error('Impossible configuration: p.Delta (%.2f) is too large for p.L (%.2f) with N=%d.', p.Delta, p.L, p.N);
end

% 2. Project the formation onto the ground. Using the final radius ensures a
%    smooth takeoff and formation-shaping transition.
ground_radius = formation_radius_final;
fprintf('Setting initial ground positions with projected radius: %.2f m\n', ground_radius);
if positioning_error_magnitude > 0
    fprintf('Adding random positioning error up to +/- %.2f m\n', positioning_error_magnitude);
end

% 3. Calculate an angular offset for a visually stable orientation.
%    This key correction rotates the entire formation to have a flat "base"
%    or to be symmetric with respect to the main axes.
if mod(p.N, 2) == 1 
    % For odd N (3, 5, ...)
    angle_offset = -pi; 
else
    % For even N (4, 6, ...)
    angle_offset = -pi -pi/4 + pi/p.N;
end

% 4. Dynamically place N UAVs on the floor using the corrected orientation.
for i = 1:p.N
    % Calculate the angle for UAV 'i' and add the rotation offset.
    angle = (i - 1) * 2 * pi / p.N + angle_offset;
    
    % Calculate the ideal XY position relative to the center.
    ideal_relative_pos_xy = ground_radius * [cos(angle); sin(angle)];
    random_error_xy = positioning_error_magnitude * (2 * rand(2, 1) - 1);
    final_relative_pos_xy = ideal_relative_pos_xy + random_error_xy;
    
    % Assign the absolute UAV position in the state vector eta0.
    uav_indices = (i - 1) * 3 + 1 : i * 3;
    eta0(uav_indices) = payload_initial_pos + [final_relative_pos_xy; 0];
end

% 5. Place the payload at the center of the formation.
payload_indices = p.N * 3 + 1 : (p.N + 1) * 3;
eta0(payload_indices) = payload_initial_pos;

% 6. Ensure all initial Z-coordinates are exactly at the floor level.
eta0(3:3:npos) = p.floor_z;

% Set initial velocities and integral states to zero.
v0 = zeros(nvel, 1);
delta0 = zeros(ndelta, 1);
x0 = [eta0; v0; delta0]; % Assemble the full initial state vector


%% -------------------- Simulation --------------------
fprintf('\nStarting simulation...\n');
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
clear functions; % Clear persistent variables in the ODE function
[t, X] = ode45(@(t,x) uav_odes(t, x, p, npos, nvel, ndelta), tspan, x0, opts);
fprintf('Simulation finished.\n');

%% -------------------- Extract Trajectories --------------------
nT = length(t);
eta_traj   = X(:, 1:npos);
v_traj     = X(:, npos + 1 : npos + nvel);
delta_traj = X(:, npos + nvel + 1 : end);

%% -------------------- Post-processing & Performance Analysis --------------------
% This section re-simulates the mission logic without dynamics to calculate
% the desired velocity (vd) at each time step. This is used for error analysis
% and to verify the mission state transitions.

vd_traj = zeros(3, nT); 
post_mission_phase = 0; % Re-start logic from phase 0
stabilization_timer_start_post = NaN;

time_at_start_departure = NaN;
time_at_goal_arrival    = NaN;

for ti = 1:nT
    eta_k   = eta_traj(ti, :)';
    v_k     = v_traj(ti, :)';
    r_k     = p.D' * eta_k;
    
    payload_pos_k = eta_k(p.N * 3 + 1 : p.N * 3 + 3);
    payload_vel_k = v_k(p.N * 3 + 1 : p.N * 3 + 3);
    
    uav_positions_k  = eta_k(1 : p.N * 3);
    uav_velocities_k = v_k(1 : p.N * 3);
    avg_uav_pos_k = mean(reshape(uav_positions_k, 3, p.N), 2);
    avg_uav_vel_k = mean(reshape(uav_velocities_k, 3, p.N), 2);

    previous_phase = post_mission_phase;
    
    vd_k = [0; 0; 0]; % Default desired velocity
    
    % Replicate the state machine from uav_odes.m
    switch post_mission_phase
        case 0 % Raise UAVs
            target_pos_phase0 = [avg_uav_pos_k(1:2); p.floor_z + p.L / 6];
            pos_error_phase0 = target_pos_phase0 - avg_uav_pos_k;
            vd_k = p.Kp_stabilize * pos_error_phase0;
            if abs(avg_uav_pos_k(3) - target_pos_phase0(3)) < 0.05 && norm(avg_uav_vel_k) < 0.05
                post_mission_phase = 1;
            end
        case 1 % Achieve Formation
            dist_err = 0; 
            for l = 1:p.T
                rl = r_k((l - 1) * 3 + 1 : l * 3);
                dist_err = dist_err + abs(norm(rl) - p.Delta); 
            end
            uav_speeds = vecnorm(reshape(uav_velocities_k, 3, p.N));
            if dist_err < p.formation_pos_threshold && all(uav_speeds < p.formation_vel_threshold)
                post_mission_phase = 2;
            end
        case 2 % Go to START
            pos_error_k = p.payload_start_pos - payload_pos_k;
            vd_k = p.Kp_stabilize * pos_error_k;
            if norm(pos_error_k) < p.stabilization_pos_threshold
                post_mission_phase = 3;
            end
        case 3 % Stabilize at START
            pos_error_k = p.payload_start_pos - payload_pos_k;
            vd_k = p.Kp_stabilize * pos_error_k;
            is_stabilized = (norm(pos_error_k) < p.stabilization_pos_threshold && norm(payload_vel_k) < p.stabilization_vel_threshold);
            if is_stabilized
                if isnan(stabilization_timer_start_post), stabilization_timer_start_post = t(ti); end
                if (t(ti) - stabilization_timer_start_post >= p.stabilization_duration)
                    post_mission_phase = 4; stabilization_timer_start_post = NaN;
                end
            else
                stabilization_timer_start_post = NaN; 
            end
        case 4 % Go to GOAL
            pos_error_k = p.payload_goal_pos - payload_pos_k;
            vd_k = p.Kp_trajectory * pos_error_k;
            if norm(pos_error_k) < p.goal_pos_threshold
                post_mission_phase = 5;
            end
        case 5 % Stabilize at GOAL
            pos_error_k = p.payload_goal_pos - payload_pos_k;
            vd_k = p.Kp_trajectory * pos_error_k;
            is_stabilized_at_goal = (norm(pos_error_k) < p.goal_pos_threshold && norm(payload_vel_k) < p.stabilization_vel_threshold);
            if is_stabilized_at_goal
                if isnan(stabilization_timer_start_post), stabilization_timer_start_post = t(ti); end
                if (t(ti) - stabilization_timer_start_post >= p.stabilization_duration)
                    post_mission_phase = 6; stabilization_timer_start_post = NaN;
                end
            else
                stabilization_timer_start_post = NaN;
            end
        case 6 % Mission Complete
            vd_k = [0; 0; 0];
    end

    % --- LOGIC TO CAPTURE TRANSITION TIMES ---
    if post_mission_phase ~= previous_phase % A transition occurred
        
        % Check for departure from START waypoint
        % This is the transition from phase 3 (Stabilize at START) to 4 (Go to GOAL)
        if previous_phase == 3 && post_mission_phase == 4 && isnan(time_at_start_departure)
            time_at_start_departure = t(ti);
        end
        
        % Check for arrival at GOAL waypoint
        % This is the transition from phase 4 (Go to GOAL) to 5 (Stabilize at GOAL)
        if previous_phase == 4 && post_mission_phase == 5 && isnan(time_at_goal_arrival)
            time_at_goal_arrival = t(ti);
        end
    end
    
    % Saturate desired velocity
    if norm(vd_k) > p.max_speed, vd_k = vd_k / norm(vd_k) * p.max_speed; end
    vd_traj(:, ti) = vd_k;
end

%% -------------------- Enhanced Collision Check --------------------
fprintf('\n--- Running Enhanced Collision Check ---\n');
collision_found = false;
for ti = 1:nT
    if collision_found, break; end % Exit loop once a collision is found
    
    current_eta = eta_traj(ti, :);
    payload_pos = current_eta(p.N * 3 + 1 : (p.N + 1) * 3)';
    
    % --- 1. Agent-Obstacle Collision Check (UAVs + Payload) ---
    for agent_idx = 1:(p.N + 1)
        if agent_idx <= p.N
            agent_name = sprintf('UAV %d', agent_idx);
            pos_idx = (agent_idx - 1) * 3 + 1 : agent_idx * 3;
            agent_radius = p.uav_radius;
        else
            agent_name = 'Payload';
            pos_idx = p.N * 3 + 1 : (p.N + 1) * 3;
            agent_radius = 0; % Assuming payload is a point for this check
        end
        agent_pos = current_eta(pos_idx)';
        
        for obs_idx = 1:p.nObs
            obs = p.obstacles{obs_idx};
            distance = inf; 
            if strcmp(obs.type, 'sphere')
                % Distance between agent surface and obstacle surface
                distance = norm(agent_pos - obs.center) - obs.radius - agent_radius;
            elseif strcmp(obs.type, 'cylinder')
                % Check if agent is within the cylinder's height range
                is_vertically_aligned = (agent_pos(3) > obs.base_center(3)) && ...
                                        (agent_pos(3) < (obs.base_center(3) + obs.height));
                if is_vertically_aligned
                    dist_xy = norm(agent_pos(1:2) - obs.base_center(1:2));
                    distance = dist_xy - obs.radius - agent_radius;
                end
            end
            
            if distance <= 0
                fprintf('AGENT-OBSTACLE COLLISION DETECTED at t=%.2f s!\n', t(ti));
                fprintf('  - Agent: %s\n', agent_name);
                fprintf('  - Obstacle: %d (type: %s)\n', obs_idx, obs.type);
                fprintf('  - Overlap distance: %.3f m\n', -distance);
                collision_found = true;
                break; 
            end
        end
        if collision_found, break; end 
    end
    if collision_found, continue; end % Go to next timestep

    % --- 2. Cable-Obstacle Collision Check ---
    for cable_idx = 1:p.N
        uav_pos = current_eta((cable_idx - 1) * 3 + 1 : cable_idx * 3)';
        cable_vec = payload_pos - uav_pos;
        cable_len_sq = dot(cable_vec, cable_vec);

        if cable_len_sq < 1e-6, continue; end % Skip if cable is zero-length

        for obs_idx = 1:p.nObs
            obs = p.obstacles{obs_idx};
            
            % Find the point on the cable segment closest to the obstacle's reference point
            if strcmp(obs.type, 'sphere'), obs_ref_point = obs.center;
            elseif strcmp(obs.type, 'cylinder'), obs_ref_point = obs.base_center;
            else, continue; 
            end

            vec_uav_to_obs_ref = obs_ref_point - uav_pos;
            t_proj = dot(vec_uav_to_obs_ref, cable_vec) / cable_len_sq;
            t_proj = max(0, min(1, t_proj)); % Clamp projection to the segment [0, 1]
            closest_point_on_cable = uav_pos + t_proj * cable_vec;
            
            distance = inf;
            if strcmp(obs.type, 'sphere')
                distance = norm(closest_point_on_cable - obs.center) - obs.radius;
            elseif strcmp(obs.type, 'cylinder')
                is_vertically_aligned = (closest_point_on_cable(3) > obs.base_center(3)) && ...
                                        (closest_point_on_cable(3) < (obs.base_center(3) + obs.height));
                if is_vertically_aligned
                    dist_xy = norm(closest_point_on_cable(1:2) - obs.base_center(1:2));
                    distance = dist_xy - obs.radius;
                end
            end

            if distance <= 0
                fprintf('CABLE-OBSTACLE COLLISION DETECTED at t=%.2f s!\n', t(ti));
                fprintf('  - Cable: From UAV %d to Payload\n', cable_idx);
                fprintf('  - Obstacle: %d (type: %s)\n', obs_idx, obs.type);
                fprintf('  - Overlap distance: %.3f m\n', -distance);
                collision_found = true;
                break;
            end
        end
        if collision_found, break; end
    end
    if collision_found, continue; end

    % --- 3. Inter-UAV Collision Check ---
    for i = 1:p.N
        uav_i_pos = current_eta((i - 1) * 3 + 1 : i * 3)';
        for j = i + 1:p.N % Compare only with subsequent UAVs to avoid duplicates
            uav_j_pos = current_eta((j - 1) * 3 + 1 : j * 3)';
            distance_between_uavs = norm(uav_i_pos - uav_j_pos);
            
            if distance_between_uavs <= (2 * p.uav_radius)
                fprintf('INTER-UAV COLLISION DETECTED at t=%.2f s!\n', t(ti));
                fprintf('  - UAV %d and UAV %d\n', i, j);
                fprintf('  - Distance between centers: %.3f m (minimum required: %.3f m)\n', ...
                        distance_between_uavs, 2 * p.uav_radius);
                collision_found = true;
                break;
            end
        end
        if collision_found, break; end
    end
end

if ~collision_found
    disp('No collisions detected throughout the simulation.');
end
disp('------------------------------------------');

%% -------------------- Calculate and Display Travel Time --------------------
if ~isnan(time_at_start_departure) && ~isnan(time_at_goal_arrival)
    travel_time = time_at_goal_arrival - time_at_start_departure;
    fprintf('\n--- Mission Timing Analysis ---\n');
    fprintf('Departure time from START waypoint: %.2f s\n', time_at_start_departure);
    fprintf('Arrival time at GOAL waypoint:    %.2f s\n', time_at_goal_arrival);
    fprintf('Total travel time from START to GOAL: %.2f s\n', travel_time);
    fprintf('---------------------------------\n');
else
    fprintf('\n--- Mission Timing Analysis ---\n');
    fprintf('Could not determine travel time. The mission may not have reached the GOAL.\n');
    fprintf('---------------------------------\n');
end
%% -------------------- Plot and Animation --------------------
% The parameter structure 'p' is required by plotting/animation functions
% to correctly render obstacles.

disp(' ');
disp('Plotting and animation functions are commented out.');
disp('Uncomment the desired function call to generate visualizations.');

% plot_results(t, eta_traj, v_traj, vd_traj, p);
 animation_3D_playload_traj(t, eta_traj, p);
% animation_3D_uavs_traj(t, eta_traj, p);