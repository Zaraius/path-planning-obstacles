function cost = path_cost_function(V, start_x, start_y, target_x, target_y, num_links, obstacle_list, obstacle_radii, grid_resolution)
    % Path cost function: Penalizes path length and proximity to obstacles.

    % Extract vertex coordinates
    V_list = [start_x; start_y; V; target_x; target_y];
    x_coords = V_list(1:2:end);
    y_coords = V_list(2:2:end);

    % Path length cost
    path_length = 0;
    for i = 1:num_links
        segment_length = sqrt((x_coords(i+1) - x_coords(i))^2 + (y_coords(i+1) - y_coords(i))^2);
        path_length = path_length + segment_length;
    end
    cost = path_length;

     % Obstacle proximity cost
    obstacle_proximity_cost = 0;
    num_obstacles = length(obstacle_radii);
    for j = 1:num_obstacles
        obstacle_x = obstacle_list(j, 1) * grid_resolution;
        obstacle_y = obstacle_list(j, 2) * grid_resolution;
        obstacle_radius = obstacle_radii(j) * grid_resolution;
        for i = 1:num_links
             % Distance between segment and obstacle center.
            segment_distance = point_to_segment_distance(x_coords(i), y_coords(i), x_coords(i+1), y_coords(i+1), obstacle_x, obstacle_y);
            if segment_distance < obstacle_radius*3 % Increased radius.
                obstacle_proximity_cost = obstacle_proximity_cost + 1000 * exp(-(segment_distance / obstacle_radius));
            end
        end
    end
    cost = cost + obstacle_proximity_cost;
end