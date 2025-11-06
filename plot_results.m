% --- START OF FILE plot_results.m ---
function plot_results(t, eta_traj, v_traj, vd_traj, p)
% plot_results Generates a comprehensive set of plots to analyze the 
% simulation results of the UAV-payload system.
%
% This function is updated to be compatible with the provided main.m script.
% It calculates required metrics internally from the trajectory data.

%% --- Figure Setup ---
figure('Name', 'Comprehensive Simulation Analysis', 'Units', 'normalized', 'Position', [0.05 0.05 0.9 0.9]);
sgtitle('UAV Swarm with Slung Load: Mission Analysis', 'FontSize', 16, 'FontWeight', 'bold');

nT = length(t);
colors = lines(p.N);
uav_legend_labels = arrayfun(@(i) sprintf('UAV %d', i), 1:p.N, 'UniformOutput', false);

%% --- 1. XY Trajectories and Obstacles ---
subplot(3, 2, 1);
hold on; grid on; axis equal;
title('XY Trajectories & Obstacles');
xlabel('X [m]');
ylabel('Y [m]');

% Draw obstacles for context
for i = 1:p.nObs
    obs = p.obstacles{i};
    if strcmp(obs.type, 'sphere')
        plot_circle(obs.center(1), obs.center(2), obs.radius);
    elseif strcmp(obs.type, 'cylinder')
        plot_circle(obs.base_center(1), obs.base_center(2), obs.radius);
    end
end

% Plot trajectories
for i = 1:p.N
    plot(eta_traj(:, (i-1)*3+1), eta_traj(:, (i-1)*3+2), '-', 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', uav_legend_labels{i});
end
plot(eta_traj(:, p.N*3+1), eta_traj(:, p.N*3+2), 'k--', 'LineWidth', 2, 'DisplayName', 'Payload');
plot(p.payload_start_pos(1), p.payload_start_pos(2), 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot(p.payload_goal_pos(1), p.payload_goal_pos(2), 'bp', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Goal');
legend('Location', 'best');

%% --- 2. Altitude (Z) vs. Time ---
subplot(3, 2, 2);
hold on; grid on;
title('Altitude Evolution');
xlabel('Time [s]');
ylabel('Altitude Z [m]');

for i = 1:p.N
    plot(t, eta_traj(:, (i-1)*3+3), '-', 'Color', colors(i,:), 'LineWidth', 1.2, 'DisplayName', uav_legend_labels{i});
end
plot(t, eta_traj(:, p.N*3+3), 'k--', 'LineWidth', 2, 'DisplayName', 'Payload');
yline(p.floor_z, 'k-', 'LineWidth', 1.5, 'Label', 'Floor');
yline(p.payload_goal_pos(3), 'b--', 'Label', 'Goal Altitude');
legend('Location', 'best');

%% --- 3. Inter-UAV Distances (Formation Maintenance) ---
subplot(3, 2, 3);
hold on; grid on;
title('Inter-UAV Link Distances');
xlabel('Time [s]');
ylabel('Distance [m]');

link_dist = zeros(nT, p.T);
for l = 1:p.T
    i = l;
    j = mod(l, p.N) + 1; % Next UAV in the polygon
    
    uav_i_traj = eta_traj(:, (i-1)*3+1 : i*3);
    uav_j_traj = eta_traj(:, (j-1)*3+1 : j*3);
    
    link_dist(:, l) = vecnorm(uav_i_traj - uav_j_traj, 2, 2);
end

link_legend_labels = arrayfun(@(l) sprintf('Link %d-%d', l, mod(l, p.N)+1), 1:p.T, 'UniformOutput', false);
plot(t, link_dist, 'LineWidth', 1.2);
yline(p.Delta, '--k', '$\Delta_{desired}$', 'LineWidth', 1.5, 'Interpreter', 'latex');
legend(link_legend_labels, 'Location', 'best');
if p.Delta > 0, ylim([0, p.Delta * 1.5]); end

%% --- 4. Suspension Cable Lengths ---
subplot(3, 2, 4);
hold on; grid on;
title('Suspension Cable Lengths');
xlabel('Time [s]');
ylabel('Length [m]');

payload_traj = eta_traj(:, p.N*3+1:(p.N+1)*3);
cable_lengths = zeros(nT, p.N);
for i = 1:p.N
    uav_traj = eta_traj(:, (i-1)*3+1:i*3);
    delta_pos = uav_traj - payload_traj;
    cable_lengths(:, i) = vecnorm(delta_pos, 2, 2);
end
plot(t, cable_lengths, 'LineWidth', 1.2);
yline(p.L, '--k', 'L (Nominal)', 'LineWidth', 1.5);
legend(uav_legend_labels, 'Location', 'best');

%% --- 5. UAV Velocity Tracking Error ---
subplot(3, 2, 5);
hold on; grid on;
title('UAV Velocity Tracking Error');
xlabel('Time [s]');
ylabel('||v_i - v_d|| [m/s]');

vel_err = zeros(nT, p.N);
vd_traj_T = vd_traj'; % Transpose to nT x 3 for easier vector math
for i = 1:p.N
    uav_vel_traj = v_traj(:, (i-1)*3+1 : i*3);
    vel_err(:, i) = vecnorm(uav_vel_traj - vd_traj_T, 2, 2);
end

plot(t, vel_err, 'LineWidth', 1.2);
legend(uav_legend_labels, 'Location', 'northeast');

%% --- 6. Minimum Distance to Obstacles ---
subplot(3, 2, 6);
hold on; grid on;
title('Minimum Safety Distance to Obstacles');
xlabel('Time [s]');
ylabel('Minimum Distance [m]');

min_dist_to_obs = zeros(nT, p.nObs);
for ti = 1:nT
    eta_k = eta_traj(ti, :)';
    uav_pos = reshape(eta_k(1:p.N*3), 3, p.N);
    payload_pos = eta_k(p.N*3+1 : (p.N+1)*3);
    
    for obs_idx = 1:p.nObs
        obs = p.obstacles{obs_idx};
        min_dist_to_obs(ti, obs_idx) = find_min_dist_to_obstacle(uav_pos, payload_pos, p, obs);
    end
end
plot(t, min_dist_to_obs, 'LineWidth', 1.5);
yline(0, 'r--', 'Collision!', 'LineWidth', 1.5);
obs_legend_labels = arrayfun(@(i) sprintf('Obstacle %d (%s)', i, p.obstacles{i}.type), 1:p.nObs, 'UniformOutput', false);
legend(obs_legend_labels, 'Location', 'best');

% Get current y-axis limits to preserve the upper limit
current_ylim = ylim; 
% Set the lower limit to -0.1 while keeping the upper limit automatic
ylim([-0.1, current_ylim(2)]); % Ensure yline(0) is clearly visible

end

%% --- Local Helper Functions ---

function h = plot_circle(x, y, r)
    % Plots a filled circle for 2D obstacle visualization.
    th = linspace(0, 2*pi, 100);
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = fill(xunit, yunit, [0.5 0.5 0.5], 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end

function min_dist = find_min_dist_to_obstacle(uav_pos, payload_pos, p, obs)
    % Finds the minimum distance from any part of the system (UAVs, payload,
    % cables) to a single specified obstacle.
    
    min_dist = inf;
    
    % Check all agents (UAVs + payload)
    all_agents = [uav_pos, payload_pos];
    for i = 1:size(all_agents, 2)
        point = all_agents(:, i);
        agent_radius = (i <= p.N) * p.uav_radius;
        dist = get_dist_point_to_obs(point, obs, agent_radius);
        if dist < min_dist
            min_dist = dist;
        end
    end

    % Check all cables (UAV to payload)
    for i = 1:p.N
        p1 = uav_pos(:, i);
        p2 = payload_pos;
        dist = get_dist_segment_to_obs(p1, p2, obs);
        if dist < min_dist
            min_dist = dist;
        end
    end
end

function dist = get_dist_point_to_obs(point, obs, agent_radius)
    % Calculates the distance from a single point to an obstacle surface.
    dist = inf;
    if strcmp(obs.type, 'sphere')
        dist = norm(point - obs.center) - obs.radius - agent_radius;
    elseif strcmp(obs.type, 'cylinder')
        is_vertically_aligned = (point(3) > obs.base_center(3)) && ...
                                (point(3) < (obs.base_center(3) + obs.height));
        if is_vertically_aligned
            dist_xy = norm(point(1:2) - obs.base_center(1:2));
            dist = dist_xy - obs.radius - agent_radius;
        end
    end
end

function dist = get_dist_segment_to_obs(p1, p2, obs)
    % Calculates the minimum distance from a line segment to an obstacle surface.
    vec_seg = p2 - p1;
    len_sq = dot(vec_seg, vec_seg);
    if len_sq < 1e-9, dist = get_dist_point_to_obs(p1, obs, 0); return; end

    % Define the reference point on the obstacle axis/center
    if strcmp(obs.type, 'sphere')
        obs_ref_point = obs.center;
    else % cylinder
        obs_ref_point = [obs.base_center(1:2); mean([p1(3), p2(3)])];
    end
    
    % Find the closest point on the segment to the obstacle's reference point
    t_proj = dot(obs_ref_point - p1, vec_seg) / len_sq;
    t_proj = max(0, min(1, t_proj)); % Clamp to segment
    closest_point_on_segment = p1 + t_proj * vec_seg;
    
    % Calculate distance from this closest point to the obstacle surface
    dist = get_dist_point_to_obs(closest_point_on_segment, obs, 0);
end