function [c, ceq] = obstacle_constraints(V, max_x, max_y, start_x, start_y, target_x, target_y, num_links, obstacle_list, obstacle_radii, grid_resolution)
    % Obstacle avoidance constraints: Ensure path stays a safe distance from obstacles.
    % Also, constrain the start and end points.

    c = [];  % Inequality constraints (c <= 0)
    ceq = []; % Equality constraints (ceq == 0)

     % Extract vertex coordinates
    V_list = [start_x; start_y; V; target_x; target_y];
    x_coords = V_list(1:2:end);
    y_coords = V_list(2:2:end);

    num_obstacles = length(obstacle_radii);
    for j = 1:num_obstacles
        obstacle_x = obstacle_list(j, 1) * grid_resolution;
        obstacle_y = obstacle_list(j, 2) * grid_resolution;
        obstacle_radius = obstacle_radii(j) * grid_resolution;

        for i = 1:num_links
            % Distance between segment and obstacle center
            segment_distance = point_to_segment_distance(x_coords(i), y_coords(i), x_coords(i+1), y_coords(i+1), obstacle_x, obstacle_y);
            % Constraint: Distance must be greater than safety distance
            safety_distance = obstacle_radius * 1.2;  % Adjust the 1.2 factor as needed
            c = [c; segment_distance - safety_distance]; %make it a positive number
        end
    end
    % Additional constraints to fix start and end points.
    ceq = [ceq; x_coords(1) - start_x; y_coords(1) - start_y; x_coords(end) - target_x; y_coords(end) - target_y];
end