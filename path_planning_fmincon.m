function [x_list, y_list, path_found] = path_planning_fmincon(start_x, start_y, target_x, target_y, obstacle_list, obstacle_radii, max_x, max_y, grid_resolution)
    % PATH_PLANNING_FMINCON Path planning using constrained optimization (fmincon).
    %
    % [x_list, y_list, path_found] = path_planning_fmincon(start_x, start_y, target_x, target_y, obstacle_list, obstacle_radii, max_x, max_y, grid_resolution)
    %
    % Inputs:
    %   start_x, start_y:  Starting coordinates.
    %   target_x, target_y: Target coordinates.
    %   obstacle_list:   A list of obstacle coordinates. Each obstacle is a 1x2
    %                    array [x_y].
    %   obstacle_radii:  A list of obstacle radii, corresponding to obstacle_list.
    %   max_x, max_y:      Maximum X and Y dimensions of the original grid.
    %   grid_resolution:  The factor by which to increase the grid resolution.
    %
    % Outputs:
    %   x_list, y_list:  Lists of X and Y coordinates of the optimal path.
    %                    Returns empty arrays if no path is found.
    %   path_found:      Boolean, true if a path was found, false otherwise.

    % 1. Initialization

    % Calculate high-resolution grid dimensions
    high_max_x = max_x * grid_resolution;
    high_max_y = max_y * grid_resolution;

    % Convert start and target to high-resolution coordinates
    high_start_x = round(start_x * grid_resolution);
    high_start_y = round(start_y * grid_resolution);
    high_target_x = round(target_x * grid_resolution);
    high_target_y = round(target_y * grid_resolution);

    % Number of links (path segments) -  Important choice
    num_links = 100;  % You can adjust this

    % 2. Initial Guess
    x_guess = linspace(high_start_x, high_target_x, num_links + 1);
    y_guess = linspace(high_start_y, high_target_y, num_links + 1);
    coords_guess = zeros(2 * (num_links - 1), 1);
    for n = 2:num_links
        coords_guess(2 * n - 3, 1) = x_guess(n);
        coords_guess(2 * n - 2, 1) = y_guess(n);
    end

    % 3. Cost Function
    cost_function = @(V) path_cost_function(V, high_start_x, high_start_y, high_target_x, high_target_y, num_links, obstacle_list, obstacle_radii, grid_resolution);

    % 4. Constraint Function
    constraint_function = @(V) obstacle_constraints(V, high_max_x, high_max_y, high_start_x, high_start_y, high_target_x, high_target_y, num_links, obstacle_list, obstacle_radii, grid_resolution);

    % 5. Optimization
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');  % You can adjust these options
    [optimized_coords, ~, exitflag, output] = fmincon(cost_function, coords_guess, [], [], [], [], [], [], constraint_function, options);

    % 6. Path Construction
     if exitflag > 0 %fmincon found a solution
        V_list = [high_start_x; high_start_y; optimized_coords; high_target_x; high_target_y];
        x_list = V_list(1:2:end) / grid_resolution; %convert back to original
        y_list = V_list(2:2:end) / grid_resolution;
        path_found = true;
     else
        x_list = [];
        y_list = [];
        path_found = false;
     end
end

