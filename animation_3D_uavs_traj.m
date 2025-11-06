% --- START OF FILE animation_3D_uavs_traj.m ---
function animation_3D_uavs_traj(t, eta_traj, p)
% animation_3D_uavs_traj Animates the 3D mission, highlighting the 
% individual trajectories (tails) of each UAV.
%
% This function creates a 3D visualization of the entire mission, drawing
% the UAVs, payload, cables, obstacles, and waypoints. It specifically
% emphasizes the path taken by each UAV by rendering a trailing line.

%% --- 1. Figure and Axis Setup ---
figure('Name', '3D Animation: UAV Trajectories', 'Position', [100 100 1000 800]);
ax = gca;
hold on; grid on; axis equal;
view(30, 20); % Set a good initial viewing angle

% Configure plot aesthetics
ax.Color = 'w';
ax.GridColor = 'k';
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

% Set axis limits dynamically to encompass all mission elements
set_axis_limits(eta_traj, p);

%% --- 2. Draw Static Scene Elements ---
draw_static_scene(p);

%% --- 3. Initialize Dynamic Graphics Objects ---
colors = lines(p.N); % Unique color for each UAV

% Pre-allocate handles for all moving objects
h_uavs = gobjects(p.N, 1);
h_cables = gobjects(p.N, 1);
h_uav_tails = gobjects(p.N, 1);

for i = 1:p.N
    % UAV markers
    h_uavs(i) = plot3(NaN, NaN, NaN, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k', ...
        'DisplayName', sprintf('UAV %d', i));
    
    % Cable lines
    h_cables(i) = plot3(NaN, NaN, NaN, 'k-', 'HandleVisibility', 'off');
    
    % UAV trajectory tails
    h_uav_tails(i) = plot3(NaN, NaN, NaN, '--', 'Color', colors(i,:), ...
        'LineWidth', 1.5, 'HandleVisibility', 'off');
end

% Payload marker and title
h_payload = plot3(NaN, NaN, NaN, 'ks', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'r', 'DisplayName', 'Payload');
h_title = title('');
legend('Location', 'best');

%%%%%%%%
disp('Inizio creazione video. L''operazione potrebbe richiedere del tempo...');
video_filename = 'mission_animation_u.mp4';
video_writer = VideoWriter(video_filename, 'MPEG-4');
video_writer.FrameRate = 25;  % Frame per secondo
video_writer.Quality = 95;   % Qualità (0-100)
open(video_writer);
%%%%%%%

%% --- 4. Animation Loop ---
animation_step = max(1, floor(length(t) / 500)); % Target ~500 frames
tail_length = 3000; % Number of past data points to show in the tail

for ti = 1:animation_step:length(t)
    % Get current payload position
    payload_indices = (p.N*3 + 1):(p.N*3 + 3);
    payload_pos = eta_traj(ti, payload_indices);
    
    % Update payload marker
    set(h_payload, 'XData', payload_pos(1), 'YData', payload_pos(2), 'ZData', payload_pos(3));

    % Determine the start index for drawing tails
    tail_start_idx = max(1, ti - tail_length);
    
    % Update each UAV, its cable, and its tail
    for i = 1:p.N
        uav_indices = (i-1)*3 + 1 : i*3;
        uav_pos = eta_traj(ti, uav_indices);
        
        % Update UAV marker
        set(h_uavs(i), 'XData', uav_pos(1), 'YData', uav_pos(2), 'ZData', uav_pos(3));
        
        % Update cable line
        set(h_cables(i), 'XData', [uav_pos(1), payload_pos(1)], ...
                          'YData', [uav_pos(2), payload_pos(2)], ...
                          'ZData', [uav_pos(3), payload_pos(3)]);
        
        % Update UAV tail
        set(h_uav_tails(i), 'XData', eta_traj(tail_start_idx:ti, uav_indices(1)), ...
                            'YData', eta_traj(tail_start_idx:ti, uav_indices(2)), ...
                            'ZData', eta_traj(tail_start_idx:ti, uav_indices(3)));
    end

    % Update title with current simulation time
    set(h_title, 'String', sprintf('Time = %.2f s', t(ti)));
    
    drawnow;

    %%%%%
    frame = getframe(gcf); % gcf = get current figure
    writeVideo(video_writer, frame);
    %%%%%

    % pause(0.01); % Uncomment for slower playback
end

%%%%%%
close(video_writer);
fprintf('Video salvato con successo come "%s"\n', video_filename);
%%%%%%

disp('Animation complete.');

end

%% --- Helper Functions ---

function set_axis_limits(eta_traj, p)
    % Calculates and sets robust axis limits based on all objects in the scene.
    
    % Gather all relevant points
    all_traj_points = reshape(eta_traj, [], 3);
    mission_points = [p.payload_start_pos'; p.payload_goal_pos'];
    obstacle_points = [];
    for i = 1:p.nObs
        obs = p.obstacles{i};
        if strcmp(obs.type, 'sphere')
            center = obs.center';
            r = obs.radius;
            obstacle_points = [obstacle_points; center + [r,0,0]; center - [r,0,0]];
        elseif strcmp(obs.type, 'cylinder')
            base = obs.base_center';
            r = obs.radius;
            h = obs.height;
            obstacle_points = [obstacle_points; base + [r,0,0]; base - [r,0,0]; base + [0,0,h]];
        end
    end
    
    % Combine all points and find boundaries
    all_points_for_limits = [all_traj_points; mission_points; obstacle_points];
    min_lim = min(all_points_for_limits, [], 1) - 2.0; % Add margin
    max_lim = max(all_points_for_limits, [], 1) + 2.0; % Add margin
    
    % Ensure floor is visible
    min_lim(3) = min(min_lim(3), p.floor_z - 0.5);
    
    xlim([min_lim(1), max_lim(1)]);
    ylim([min_lim(2), max_lim(2)]);
    zlim([min_lim(3), max_lim(3)]);
end

function draw_static_scene(p)
    % Draws non-moving elements like the floor, waypoints, and obstacles.
    
    % Draw the floor
    ax_lim = [xlim; ylim];
    floor_x = [ax_lim(1,1), ax_lim(1,2), ax_lim(1,2), ax_lim(1,1)];
    floor_y = [ax_lim(2,1), ax_lim(2,1), ax_lim(2,2), ax_lim(2,2)];
    patch(floor_x, floor_y, ones(1,4) * p.floor_z, 'FaceColor', [0.8 0.9 0.8], ...
          'FaceAlpha', 0.5, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    % Draw START and GOAL waypoints
    plot3(p.payload_start_pos(1), p.payload_start_pos(2), p.payload_start_pos(3), ...
        'g^', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'START');
    plot3(p.payload_goal_pos(1), p.payload_goal_pos(2), p.payload_goal_pos(3), ...
        'bp', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'GOAL');

    % Draw obstacles
    [sx, sy, sz] = sphere(20);
    for i = 1:p.nObs
        obs = p.obstacles{i};
        if strcmp(obs.type, 'sphere')
            surf(sx*obs.radius + obs.center(1), sy*obs.radius + obs.center(2), sz*obs.radius + obs.center(3), ...
                 'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
        elseif strcmp(obs.type, 'cylinder')
            [Xc, Yc, Zc] = cylinder(obs.radius, 30);
            Zc = Zc * obs.height;
            surf(Xc + obs.base_center(1), Yc + obs.base_center(2), Zc + obs.base_center(3), ...
                 'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
        end
    end
end