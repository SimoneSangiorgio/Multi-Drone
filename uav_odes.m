% --- START OF FILE uav_odes.m ---

% uav_odes.m
%
% DESCRIPTION:
% This function defines the Ordinary Differential Equations (ODEs) for the
% UAV-payload system. It is called by an ODE solver (e.g., ode45) at each
% time step.
%
% The function implements:
% 1. A state machine to manage the mission phases (takeoff, navigation, etc.).
% 2. A Control Barrier Function (CBF) for real-time obstacle avoidance.
% 3. The dynamics of the UAVs and the payload, including internal forces
%    from tensegrity links and cables, and external forces like gravity.
% 4. An integral action controller to compensate for model uncertainties.
%
% INPUTS:
%   t      - Current time
%   x      - Current state vector [eta; v; delta]
%   p      - Struct with system parameters
%   npos   - Number of position states
%   nvel   - Number of velocity states
%   ndelta - Number of integral action states
%
% OUTPUT:
%   dx     - State derivative vector [eta_dot; v_dot; delta_dot]

function dx = uav_odes(t, x, p, npos, nvel, ndelta)
    % Persistent variables to maintain state across function calls
    persistent mission_phase;
    persistent stabilization_timer_start;

    % Initialize on the first call
    if isempty(mission_phase)
        mission_phase = 0; 
        stabilization_timer_start = NaN; 
        fprintf('t=%.2f: Mission started. Phase 0 - Raising UAVs.\n', t);
    end

    % --- Unpack state vector for clarity ---
    eta = x(1:npos); 
    v = x(npos + 1 : npos + nvel); 
    delta = x(npos + nvel + 1 : end);
    
    uav_pos = reshape(eta(1:p.N*3), 3, p.N); 
    uav_vel = reshape(v(1:p.N*3), 3, p.N);
    payload_pos = eta(p.N * 3 + 1 : p.N * 3 + 3); 
    payload_vel = v(p.N * 3 + 1 : p.N * 3 + 3);

    %% --- 1. Mission Phase Logic & Desired Velocity (vd) Generation ---
    % This state machine determines the desired collective velocity (vd)
    % based on the current phase of the mission.
    
    vd = [0; 0; 0]; % Default desired velocity is zero (hover)
    
    avg_uav_pos = mean(uav_pos, 2); 
    avg_uav_vel = mean(uav_vel, 2); 

    switch mission_phase
        case 0 % PHASE 0: Lift UAVs to a pre-formation altitude
            target_pos = [avg_uav_pos(1:2); p.floor_z + p.L / 6];
            pos_error = target_pos - avg_uav_pos;
            vd = p.Kp_stabilize * pos_error;
            if abs(avg_uav_pos(3) - target_pos(3)) < 0.05
                mission_phase = 1;
                fprintf('t=%.2f: Phase 1 - Stabilizing after ascent (waiting 2s).\n', t);
            end

        case 1 % PHASE 1: Stabilize at hover altitude for 2 seconds
            target_pos = [avg_uav_pos(1:2); p.floor_z + p.L / 6];
            pos_error = target_pos - avg_uav_pos;
            vd = p.Kp_stabilize * pos_error;
            is_stabilized_at_hover = (norm(pos_error) < 0.05 && norm(avg_uav_vel) < 0.05);
            if is_stabilized_at_hover
                if isnan(stabilization_timer_start)
                    stabilization_timer_start = t;
                end
                if (t - stabilization_timer_start >= 2.0)
                    mission_phase = 2; stabilization_timer_start = NaN;
                    fprintf('t=%.2f: Phase 2 - Achieving formation.\n', t);
                end
            else
                stabilization_timer_start = NaN;
            end
            
        case 2 % PHASE 2: Achieve desired UAV-UAV formation
            r = p.D' * eta;
            dist_err = 0; 
            for l = 1:p.T
                rl = r((l - 1) * 3 + 1 : l * 3);
                dist_err = dist_err + abs(norm(rl) - p.Delta);
            end
            if dist_err < p.formation_pos_threshold && all(vecnorm(uav_vel) < p.formation_vel_threshold)
                mission_phase = 3; 
                fprintf('t=%.2f: Phase 3 - Lifting payload vertically.\n', t);
            end

        case 3 % PHASE 3: Lift the payload vertically off the ground
            target_pos = [payload_pos(1:2); p.floor_z + 1];
            pos_error = target_pos - payload_pos;
            vd = p.Kp_stabilize * pos_error;
            if abs(payload_pos(3) - target_pos(3)) < 0.05
                mission_phase = 4;
                fprintf('t=%.2f: Phase 4 - Stabilizing payload after lift (waiting 2s).\n', t);
            end
            
        case 4 % PHASE 4: Stabilize briefly after lifting the payload
            target_pos = [payload_pos(1:2); p.floor_z + 1];
            pos_error = target_pos - payload_pos;
            vd = p.Kp_stabilize * pos_error;
            if norm(payload_vel) < p.stabilization_vel_threshold
                mission_phase = 5;
                fprintf('t=%.2f: Phase 5 - Moving to START.\n', t);
            end

        case 5 % PHASE 5: Move to the START waypoint
            pos_error = p.payload_start_pos - payload_pos; 
            vd = p.Kp_stabilize * pos_error; 
            if norm(pos_error) < p.stabilization_pos_threshold
                mission_phase = 6;
                fprintf('t=%.2f: Phase 6 - Stabilizing at START (waiting 2s).\n', t);
            end

        case 6 % PHASE 6: Stabilize at the START waypoint
            pos_error = p.payload_start_pos - payload_pos; 
            vd = p.Kp_stabilize * pos_error; 
            is_stabilized = (norm(pos_error) < p.stabilization_pos_threshold && norm(payload_vel) < p.stabilization_vel_threshold); 
            if is_stabilized
                if isnan(stabilization_timer_start)
                    stabilization_timer_start = t;
                    fprintf('t=%.2f: Stabilization timer at START started.\n',t);
                end
                if (t - stabilization_timer_start >= p.stabilization_duration)
                    mission_phase = 7; stabilization_timer_start = NaN; 
                    fprintf('t=%.2f: Phase 7 - Moving to GOAL.\n', t);
                end
            else
                stabilization_timer_start = NaN; 
            end

        case 7 % PHASE 7: Move to the GOAL waypoint
            pos_error = p.payload_goal_pos - payload_pos; 
            vd = p.Kp_trajectory * pos_error; 
            if norm(pos_error) < p.goal_pos_threshold
                mission_phase = 8; 
                fprintf('t=%.2f: Phase 8 - Stabilizing at GOAL (waiting 2s).\n', t);
            end

        case 8 % PHASE 8: Stabilize at the GOAL waypoint
            pos_error = p.payload_goal_pos - payload_pos; 
            vd = p.Kp_trajectory * pos_error; 
            is_stabilized_at_goal = (norm(pos_error) < p.goal_pos_threshold && norm(payload_vel) < p.stabilization_vel_threshold); 
            if is_stabilized_at_goal
                if isnan(stabilization_timer_start)
                    stabilization_timer_start = t;
                    fprintf('t=%.2f: Stabilization timer at GOAL started.\n',t);
                end
                if (t - stabilization_timer_start >= p.stabilization_duration)
                    mission_phase = 9; stabilization_timer_start = NaN; 
                    fprintf('t=%.2f: Mission Complete.\n', t);
                end
            else
                stabilization_timer_start = NaN; 
            end
            
        case 9 % PHASE 9: Mission complete
            vd = [0; 0; 0];
    end
    
    if norm(vd) > p.max_speed
        vd = vd / norm(vd) * p.max_speed;
    end
    
    %% --- 2. Obstacle Avoidance (Control Barrier Function) ---
    % This section implements the obstacle avoidance logic. It activates when
    % the system comes within a predefined absolute distance (p.cbf_influence_distance)
    % from any obstacle's surface.
    
    [critical_h, critical_grad_h, is_inside, ~] = find_most_critical_obstacle(uav_pos, payload_pos, p);
    
    avoidance_active = false;
    vd_safe = vd; % Start with the nominal velocity
    
    % Check if the closest point to any obstacle is within the influence distance.
    % 'critical_h' is the distance from the system's surface to the obstacle's surface.
    if critical_h < p.cbf_influence_distance
        avoidance_active = true;
        
        % --- A. Circumnavigation Logic ---
        % Blend the nominal velocity with a tangential velocity to slide around the obstacle.
        up_vector = [0; 0; 1];
        t_dir = cross(critical_grad_h, up_vector); % Tangential direction
        if norm(t_dir) < 1e-3, t_dir = cross(critical_grad_h, [0; 1; 0]); end
        t_dir = t_dir / norm(t_dir);
        if dot(vd, t_dir) < 0, t_dir = -t_dir; end % Move in a sensible direction
    
        % The weight for circumnavigation increases as we get closer to the obstacle.
        % normalized_dist is 1 at the edge of the influence zone, 0 at the surface.
        normalized_dist = critical_h / p.cbf_influence_distance;
        w_circ = 0.5 * (1 + cos(pi * normalized_dist)); % Cosine blending
        w_circ = max(0, min(1, w_circ));
        
        v_circ = norm(vd) * t_dir; % Tangential velocity with same speed as nominal
        vd_nominal = (1 - w_circ) * vd + w_circ * v_circ;
    
        % --- B. Final CBF Safety Correction ---
        % This acts as a hard safety constraint to ensure the barrier is not crossed.
        phi = dot(critical_grad_h, vd_nominal) + p.gamma_cbf_obs * critical_h;
        
        v_corr = [0; 0; 0];
        if is_inside % If already inside, the priority is to escape
            v_corr = (p.max_speed * critical_grad_h) - vd_nominal;
        elseif phi < 0 % If about to violate the barrier, apply a correction
            v_corr = -phi * critical_grad_h;
        end
        
        vd_safe = vd_nominal + v_corr;
    end
    
    %% --- 3. System Dynamics and Forces ---
    r = p.D' * eta;
    h_force = zeros(3 * p.E, 1);
    
    k_effective = p.k; 
    if avoidance_active, k_effective = p.k * 0.1; end
    
    for l = 1:p.E
        idx = (l - 1) * 3 + 1 : l * 3;
        rl = r(idx);
        if l <= p.T
            h_force(idx) = k_effective * (rl - p.rd(idx));
        else
            nr = norm(rl);
            if nr > eps
                sigma = p.gamma * max(0, nr - p.L);
                h_force(idx) = sigma * (rl / nr);
            end
        end
    end
    u_all_from_formation = -p.D * h_force; 
    
    u_cbf_floor = zeros(3 * p.N, 1);
    all_agents_pos = [uav_pos, payload_pos];
    all_agents_vel = [uav_vel, payload_vel];
    for agent_idx = 1:(p.N + 1)
        pos = all_agents_pos(:, agent_idx);
        vel = all_agents_vel(:, agent_idx);
        h_floor = pos(3) - p.floor_z;
        grad_h_floor = [0; 0; 1];
        phi_floor = dot(grad_h_floor, vel) + p.gamma_cbf_floor * h_floor;
        if phi_floor < 0
            f_repulsive_floor = -phi_floor * grad_h_floor;
            if agent_idx <= p.N
                u_cbf_floor((agent_idx - 1) * 3 + 1 : agent_idx * 3) = u_cbf_floor((agent_idx - 1) * 3 + 1 : agent_idx * 3) + f_repulsive_floor;
            else
                u_cbf_floor = u_cbf_floor + repmat(f_repulsive_floor / p.N, p.N, 1);
            end
        end
    end
    
    u_all = u_all_from_formation;
    u_all(1:3*p.N) = u_all(1:3*p.N) + u_cbf_floor + repmat(p.wind, p.N, 1);
    
    %% --- 4. State Derivatives ---
    eta_dot = v;
    v_dot = zeros(nvel, 1);
    delta_dot = zeros(ndelta, 1);
    
    for i = 1:p.N
        idx = (i - 1) * 3 + 1 : i * 3;
        v_dot(idx) = (vd_safe - v(idx) - delta(idx) + u_all(idx)) / p.m;
        delta_dot(idx) = v(idx) - vd_safe;
    end
    
    idxL = p.N * 3 + 1 : (p.N + 1) * 3;
    v_dot(idxL) = p.g + u_all(idxL) / p.mL;
    
    for agent_idx = 1:(p.N + 1)
        pos_idx = (agent_idx - 1) * 3 + 1 : agent_idx * 3;
        if eta(pos_idx(3)) <= p.floor_z
            v_dot(pos_idx(3)) = max(0, v_dot(pos_idx(3)));
            if v(pos_idx(3)) < 0
                v(pos_idx(3)) = 0;
                eta_dot(pos_idx(3)) = 0;
            end
        end
    end

    dx = [eta_dot; v_dot; delta_dot];
end

%--------------------------------------------------------------------------
%                       --- HELPER FUNCTIONS ---
%--------------------------------------------------------------------------

%% --- MODIFIED Helper Function: Find Most Critical Obstacle ---
function [critical_h, critical_grad_h, is_inside, critical_obs_idx] = find_most_critical_obstacle(uav_pos, payload_pos, p)
    critical_h = inf;
    critical_grad_h = [0; 0; 0];
    is_inside = false;
    critical_obs_idx = -1;

    critical_points = {payload_pos}; 
    for i = 1:p.N, critical_points{end+1} = uav_pos(:, i); end
    for i = 1:p.N, critical_points{end+1} = 0.5 * uav_pos(:, i) + 0.5 * payload_pos; end

    agent_positions = [uav_pos, payload_pos];
    all_segments = {};
    if p.N > 1
        for i = 1:p.N
            next_uav_idx = mod(i, p.N) + 1;
            all_segments{end+1} = {i, next_uav_idx};
        end
    end
    for i = 1:p.N, all_segments{end+1} = {i, p.N + 1}; end

    for obs_idx = 1:p.nObs
        obs = p.obstacles{obs_idx};
        
        % Check discrete points
        for i = 1:length(critical_points)
            point = critical_points{i};
            [h, grad_h, is_inside_pt] = get_barrier_function_data(point, obs);
            if h < critical_h
                critical_h = h; critical_grad_h = grad_h; is_inside = is_inside_pt; critical_obs_idx = obs_idx;
            end
        end

        % +++ MODIFIED SECTION: ANALYTICAL DISTANCE CHECK FOR SEGMENTS +++
        for s = 1:length(all_segments)
            p1 = agent_positions(:, all_segments{s}{1});
            p2 = agent_positions(:, all_segments{s}{2});

            if norm(p1 - p2) < 1e-6, continue; end

            if strcmp(obs.type, 'sphere')
                [h, grad_h, is_inside_seg] = distance_segment_to_sphere(p1, p2, obs.center, obs.radius);
            
            elseif strcmp(obs.type, 'cylinder')
                [h, grad_h, is_inside_seg] = distance_segment_to_cylinder(p1, p2, obs.base_center, obs.radius, obs.height);
                
            else % Fallback to original approximation for unsupported types
                obs_ref_point = obs.center; % Generic reference
                vec_seg = p2 - p1;
                len_sq = dot(vec_seg, vec_seg);
                t_proj = max(0, min(1, dot(obs_ref_point - p1, vec_seg) / len_sq));
                closest_point = p1 + t_proj * vec_seg;
                [h, grad_h, is_inside_seg] = get_barrier_function_data(closest_point, obs);
            end

            if h < critical_h
                critical_h = h; critical_grad_h = grad_h; is_inside = is_inside_seg; critical_obs_idx = obs_idx;
            end
        end
    end
end

%% --- Helper Function: Calculate Barrier Data for a POINT ---
function [h, grad_h, is_inside] = get_barrier_function_data(point_pos, obs)
    h = inf; grad_h = [0; 0; 0]; is_inside = false;
    
    if strcmp(obs.type, 'sphere')
        vec_to_center = point_pos - obs.center;
        dist_to_center = norm(vec_to_center);
        h = dist_to_center - obs.radius;
        if dist_to_center > 1e-6, grad_h = vec_to_center / dist_to_center; end
        if h < 0, is_inside = true; end
        
    elseif strcmp(obs.type, 'cylinder')
        is_vertically_aligned = (point_pos(3) > obs.base_center(3)) && ...
                                (point_pos(3) < (obs.base_center(3) + obs.height));
        if is_vertically_aligned
            vec_to_axis_xy = point_pos(1:2) - obs.base_center(1:2);
            dist_to_axis = norm(vec_to_axis_xy);
            h = dist_to_axis - obs.radius;
            if dist_to_axis > 1e-6, grad_h = [vec_to_axis_xy / dist_to_axis; 0]; end
            if h < 0, is_inside = true; end
        end
    end
end

%% --- NEW Helper: Analytical Segment-to-Sphere Distance ---
function [h, grad_h, is_inside] = distance_segment_to_sphere(p1, p2, sphere_center, sphere_radius)
    vec_seg = p2 - p1;
    len_sq = dot(vec_seg, vec_seg);

    if len_sq < 1e-12
        closest_point_on_segment = p1;
    else
        t = dot(sphere_center - p1, vec_seg) / len_sq;
        t_clamped = max(0, min(1, t));
        closest_point_on_segment = p1 + t_clamped * vec_seg;
    end
    
    dist_vector = closest_point_on_segment - sphere_center;
    dist_from_center = norm(dist_vector);
    
    h = dist_from_center - sphere_radius;
    is_inside = (h <= 0);
    
    if dist_from_center > 1e-9
        grad_h = dist_vector / dist_from_center;
    else
        grad_h = [0; 0; 1];
    end
end

%% --- NEW Helper: Analytical Segment-to-Cylinder Distance ---
function [h, grad_h, is_inside] = distance_segment_to_cylinder(p1, p2, cyl_base_center, cyl_radius, cyl_height)
    % Find closest point on segment to cylinder axis
    p1_xy = p1(1:2); p2_xy = p2(1:2); axis_xy = cyl_base_center(1:2);
    vec_seg_xy = p2_xy - p1_xy;
    len_sq_xy = dot(vec_seg_xy, vec_seg_xy);
    if len_sq_xy < 1e-12, t_xy = 0; else, t_xy = dot(axis_xy - p1_xy, vec_seg_xy) / len_sq_xy; end
    t_clamped_xy = max(0, min(1, t_xy));
    pt_on_seg_to_axis = p1 + t_clamped_xy * (p2 - p1);

    % Find closest point on cylinder axis
    z_min = cyl_base_center(3); z_max = z_min + cyl_height;
    clamped_z = max(z_min, min(z_max, pt_on_seg_to_axis(3)));
    pt_on_axis = [axis_xy; clamped_z];

    % Case 1: Closest point is on the cylinder wall
    if pt_on_seg_to_axis(3) >= z_min && pt_on_seg_to_axis(3) <= z_max
        dist_vec = pt_on_seg_to_axis - pt_on_axis;
        dist = norm(dist_vec);
        h = dist - cyl_radius;
        if dist > 1e-9, grad_h = dist_vec / dist; else, grad_h = [0;0;1]; end

    % Case 2: Closest point is on the top or bottom cap (a disk)
    else
        pt_on_cap_center = pt_on_axis;
        dist_to_cap_center_vec = pt_on_seg_to_axis - pt_on_cap_center;
        dist_radial = norm(dist_to_cap_center_vec(1:2));

        if dist_radial <= cyl_radius % Closest point is on the flat surface of the cap
            dist_vec = pt_on_seg_to_axis - pt_on_cap_center;
        else % Closest point is on the circular edge (rim) of the cap
            rim_point = pt_on_cap_center + [cyl_radius * dist_to_cap_center_vec(1:2) / dist_radial; 0];
            dist_vec = pt_on_seg_to_axis - rim_point;
        end
        dist = norm(dist_vec);
        h = dist;
        if pt_on_seg_to_axis(3) < z_min, h = -h; end % Negative if inside the plane
        
        is_inside_radially = (norm(pt_on_seg_to_axis(1:2) - axis_xy) < cyl_radius);
        if (pt_on_seg_to_axis(3) < z_min && is_inside_radially) || (pt_on_seg_to_axis(3) > z_max && is_inside_radially)
             % This logic is complex; a simple vertical gradient is a robust choice
             grad_h = [0;0; sign(pt_on_seg_to_axis(3) - pt_on_axis(3))];
        else
             if dist > 1e-9, grad_h = dist_vec / dist; else, grad_h = [0;0;1]; end
        end
    end
    is_inside = (h <= 0);
end