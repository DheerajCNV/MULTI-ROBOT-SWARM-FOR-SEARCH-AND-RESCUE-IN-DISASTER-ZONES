clear; close all; clc;

% Simulation Parameters
num_robots = 20; % Number of robots
area_size = 200; % Size of area (meters)
time_steps = 250; % Number of simulation steps
dt = 0.1; % Time step
max_speed = 25; % Robot maximum speed
coverage_resolution = 5; % Grid resolution for coverage map
initial_sensor_radius = 16; % Sensor radius for detection range
energy_depletion_rate = 0.1; % Energy depletion rate
initial_energy = 100; % Initial energy of all robots

% Disaster Parameters
num_disaster_zones = 5; % Number of initial disaster zones
safe_distance = 25; % Ensure disaster zones stay at least this far from deployment area
disaster_positions = rand(num_disaster_zones, 2) * area_size;
while any(vecnorm(disaster_positions - [0, 0], 2, 2) < safe_distance)
    disaster_positions = rand(num_disaster_zones, 2) * area_size;
end
disaster_radius = 5 * ones(num_disaster_zones, 1);
max_disaster_radius = 9; % Set max growth of each disaster zone
disaster_growth_rates = rand(num_disaster_zones, 1) * 0.5; % Disaster growth rates

% Initialize robot positions and data
robot_positions = nan(num_robots, 2); % Robots will be deployed sequentially
robot_directions = nan(num_robots, 2);
robot_energy = initial_energy * ones(num_robots, 1);
robot_trajectories = cell(num_robots, 1);
for i = 1:num_robots
    robot_trajectories{i} = nan(time_steps, 2);
end

coverage_map = robotics.BinaryOccupancyGrid(area_size, area_size, 1 / coverage_resolution);
distance_covered = zeros(num_robots, 1);
total_coverage_area = zeros(time_steps, 1); % To track coverage area over time
unsafe_robot_count = zeros(time_steps, 1); % To track unsafe robots over time

% Precompute 3D terrain visualization
[x_terrain, y_terrain] = meshgrid(linspace(0, area_size, 50));
z_terrain = sin(x_terrain / area_size * pi) * 10 + cos(y_terrain / area_size * pi) * 10;

% Initialize a disaster influence heatmap matrix
disaster_influence_map = zeros(ceil(area_size / coverage_resolution));

deploy_interval = 5; % Robots deployed every 5 simulation steps
deployed_robots = 0;

% Simulation loop with robot deployment and movement
for t_idx = 1:time_steps
    clf;
    hold on;
    axis([0 area_size 0 area_size]);
    grid on;
    title(['Simulation Step ', num2str(t_idx)]);
    xlabel('X');
    ylabel('Y');

    % Robot Deployment
    if mod(t_idx, deploy_interval) == 0 && deployed_robots < num_robots
        deployed_robots = deployed_robots + 1;
        robot_positions(deployed_robots, :) = rand(1, 2) * 10; % Random deployment near the corner
        random_direction = rand(1, 2) - 0.5; 
        random_direction = random_direction / norm(random_direction);
        robot_directions(deployed_robots, :) = random_direction;

        fprintf('Deployed Robot %d at position [%f, %f]\n', deployed_robots, robot_positions(deployed_robots, 1), robot_positions(deployed_robots, 2));
    end

    % Compute and plot Voronoi Diagrams if valid
    valid_positions = ~isnan(robot_positions(:, 1));
    if sum(valid_positions) >= 3
        [unique_positions, ia] = unique(robot_positions(valid_positions, :), 'rows');
        if size(unique_positions, 1) >= 3
            [vx, vy] = voronoi(unique_positions(:, 1), unique_positions(:, 2));
            plot(vx, vy, 'k--', 'LineWidth', 1.5);
        end
    end

    % Update robots' movement
    for i = 1:num_robots
        if isnan(robot_positions(i, 1))
            continue;
        end

        % Avoid disaster zones by computing repulsion forces
        unsafe = false;
        for dz = 1:num_disaster_zones
            distance_to_disaster = vecnorm(robot_positions(i, :) - disaster_positions(dz, :), 2, 2);
            if distance_to_disaster <= disaster_radius(dz)
                unsafe = true;
                repulsion_force = (robot_positions(i, :) - disaster_positions(dz, :)) / distance_to_disaster^2;
                robot_directions(i, :) = robot_directions(i, :) + repulsion_force;
            end
        end

        % Boundary repulsion logic
        if robot_positions(i, 1) <= 0 || robot_positions(i, 1) >= area_size
            robot_directions(i, 1) = -robot_directions(i, 1);
        end
        if robot_positions(i, 2) <= 0 || robot_positions(i, 2) >= area_size
            robot_directions(i, 2) = -robot_directions(i, 2);
        end

        % Normalize movement
        if norm(robot_directions(i, :)) == 0
            robot_directions(i, :) = rand(1, 2) - 0.5;
        end
        robot_directions(i, :) = robot_directions(i, :) / norm(robot_directions(i, :));
        proposed_position = robot_positions(i, :) + max_speed * robot_directions(i, :) * dt;

        % Prevent going out of bounds
        proposed_position(1) = min(max(proposed_position(1), 0), area_size);
        proposed_position(2) = min(max(proposed_position(2), 0), area_size);
        robot_positions(i, :) = proposed_position;

        % Update trajectory
        robot_trajectories{i}(t_idx, :) = robot_positions(i, :);

        % Update coverage map
        setOccupancy(coverage_map, robot_positions(i, :), 1);

        % Plot robot positions
        if unsafe
            scatter(robot_positions(i, 1), robot_positions(i, 2), 50, 'r', 'filled'); % Unsafe robot in red
        else
            scatter(robot_positions(i, 1), robot_positions(i, 2), 50, 'g', 'filled'); % Safe robot in green
        end
    end

    % Visualize disaster zones
    for dz = 1:num_disaster_zones
        disaster_radius(dz) = min(disaster_radius(dz) + disaster_growth_rates(dz), max_disaster_radius);
        theta = linspace(0, 2 * pi, 50);
        x_circle = disaster_positions(dz, 1) + disaster_radius(dz) * cos(theta);
        y_circle = disaster_positions(dz, 2) + disaster_radius(dz) * sin(theta);
        plot(x_circle, y_circle, 'r--', 'LineWidth', 1.5);
    end

    % Disaster Influence Map (Exponential Decay)
    [X_grid, Y_grid] = meshgrid(linspace(0, area_size, size(disaster_influence_map, 1)));
    for dz = 1:num_disaster_zones
        distances = sqrt((X_grid - disaster_positions(dz, 1)).^2 + (Y_grid - disaster_positions(dz, 2)).^2);
        disaster_influence_map = disaster_influence_map + exp(-distances / (disaster_radius(dz) * 2));
    end

    % Calculate total coverage area and unsafe robots
    occupancy_matrix = occupancyMatrix(coverage_map);
    total_coverage_area(t_idx) = sum(occupancy_matrix(:)) * (coverage_resolution^2); % Convert grid count to area
    unsafe_robot_count(t_idx) = sum(arrayfun(@(i) ...
        any(vecnorm(robot_positions(i, :) - disaster_positions, 2, 2) <= disaster_radius), 1:num_robots));

    drawnow;
end

% Final Disaster Influence Heatmap
figure;
imagesc(disaster_influence_map);
colorbar;
title('Disaster Influence Zones');
xlabel('X-Grid Index');
ylabel('Y-Grid Index');
axis xy;

% Final Coverage Map
figure;
show(coverage_map);
title('Final Coverage Map');

% Robot Trajectories
figure;
hold on;
for i = 1:num_robots
    trajectory = robot_trajectories{i};
    plot(trajectory(:, 1), trajectory(:, 2), '-');
end
title('Robot Trajectories');
xlabel('X');
ylabel('Y');
grid on;
axis([0 area_size 0 area_size]);

% Coverage over time
figure;
plot(1:time_steps, total_coverage_area / (area_size * area_size) * 100);
title('Coverage Stability');
xlabel('Simulation Steps');
ylabel('Percentage Coverage');
grid on;

% Unsafe robot count over time
figure;
plot(1:time_steps, unsafe_robot_count);
title('Unsafe Robots Over Time');
xlabel('Simulation Steps');
ylabel('Number of Unsafe Robots');
grid on;

% 3D Visualization of Disaster Influence
figure;
hold on;

% Plot terrain
surf(x_terrain, y_terrain, z_terrain, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
colormap('parula');

% Plot disaster zones as hemispheres
for dz = 1:num_disaster_zones
    [X_sphere, Y_sphere, Z_sphere] = sphere;
    Z_sphere(Z_sphere < 0) = 0; % Make it a hemisphere
    X_sphere = X_sphere * disaster_radius(dz) + disaster_positions(dz, 1);
    Y_sphere = Y_sphere * disaster_radius(dz) + disaster_positions(dz, 2);
    Z_sphere = Z_sphere * disaster_radius(dz);
    surf(X_sphere, Y_sphere, Z_sphere, 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'FaceColor', 'r');
end

% Plot robots as points
scatter3(robot_positions(:, 1), robot_positions(:, 2), zeros(num_robots, 1), 50, 'g', 'filled');
xlabel('X');
ylabel('Y');
zlabel('Height');
title('3D Visualization of Disaster Influence Zones');
view(3);
grid on;