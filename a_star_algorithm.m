function [x_list, y_list] = a_star_algorithm(start_x, start_y, target_x, target_y, obstacle_list, max_x, max_y, allow_diagonal, interpolation_distance, grid_resolution, robot_radius)
    % A_STAR_ALGORITHM A* pathfinding algorithm.
    %
    % [x_list, y_list] = a_star_algorithm(start_x, start_y, target_x, target_y, obstacle_list, max_x, max_y, allow_diagonal, interpolation_distance, grid_resolution, robot_radius)
    %
    % Inputs:
    %   start_x, start_y:  Starting coordinates.
    %   target_x, target_y: Target coordinates.
    %   obstacle_list:   A list of obstacle coordinates. Each obstacle is a 1x2
    %                    array [x, y].  Can be empty [].
    %   max_x, max_y:      Maximum X and Y dimensions of the original grid.
    %   allow_diagonal:   Boolean, if true, diagonal movements are allowed.
    %   interpolation_distance:   Distance between interpolated points. If 0, no interpolation.
    %   grid_resolution:  The factor by which to increase the grid resolution.
    %   robot_radius:    The radius of the robot (or circle)
    %
    % Outputs:
    %   x_list, y_list:  Lists of X and Y coordinates of the optimal path
    %                    (excluding start and target, which are handled by the calling function).
    %                    Returns empty arrays if no path is found.

    % Calculate the dimensions of the high-resolution grid
    high_max_x = max_x * grid_resolution;
    high_max_y = max_y * grid_resolution;

    % Define the high-resolution map array.
    MAP = 2 * ones(high_max_x, high_max_y);  % 2: Space

    % Convert start and target to high-resolution coordinates
    high_start_x = round(start_x * grid_resolution);
    high_start_y = round(start_y * grid_resolution);
    high_target_x = round(target_x * grid_resolution);
    high_target_y = round(target_y * grid_resolution);

    % Initialize map with target and start
    MAP(high_target_x, high_target_y) = 0;
    MAP(high_start_x, high_start_y) = 1;

    % Calculate the inflation radius in high-resolution grid cells
    inflation_radius_high = ceil(robot_radius * grid_resolution); %important ceil

    % Place obstacles on the high-resolution map, with inflation
    for i = 1:size(obstacle_list, 1)
        obstacle_x = round(obstacle_list(i, 1) * grid_resolution);
        obstacle_y = round(obstacle_list(i, 2) * grid_resolution);

        for x_offset = -inflation_radius_high:inflation_radius_high
            for y_offset = -inflation_radius_high:inflation_radius_high
                % Check boundaries
                if (obstacle_x + x_offset >= 1 && obstacle_x + x_offset <= high_max_x && obstacle_y + y_offset >= 1 && obstacle_y + y_offset <= high_max_y)
                    MAP(obstacle_x + x_offset, obstacle_y + y_offset) = -1;
                end
            end
        end
    end

    % Lists used for the algorithm
    OPEN = [];
    CLOSED = [];

    % Put all obstacles on the Closed list
    k = 1;
    for i = 1:high_max_x
        for j = 1:high_max_y
            if (MAP(i, j) == -1)
                CLOSED(k, 1) = i;
                CLOSED(k, 2) = j;
                k = k + 1;
            end
        end
    end
    CLOSED_COUNT = size(CLOSED, 1);

    % Set the starting node as the first node
    xNode = high_start_x;
    yNode = high_start_y;
    OPEN_COUNT = 1;
    path_cost = 0;
    goal_distance = distance(xNode, yNode, high_target_x, high_target_y, allow_diagonal);
    OPEN(OPEN_COUNT, :) = insert_open(xNode, yNode, xNode, yNode, path_cost, goal_distance, goal_distance);
    OPEN(OPEN_COUNT, 1) = 0;
    CLOSED_COUNT = CLOSED_COUNT + 1;
    CLOSED(CLOSED_COUNT, 1) = xNode;
    CLOSED(CLOSED_COUNT, 2) = yNode;
    NoPath = 1;

    % START ALGORITHM
    while ((xNode ~= high_target_x || yNode ~= high_target_y) && NoPath == 1)
        exp_array = expand_array(xNode, yNode, path_cost, high_target_x, high_target_y, CLOSED, high_max_x, high_max_y, allow_diagonal);
        exp_count = size(exp_array, 1);

        for i = 1:exp_count
            flag = 0;
            for j = 1:OPEN_COUNT
                if (exp_array(i, 1) == OPEN(j, 2) && exp_array(i, 2) == OPEN(j, 3))
                    OPEN(j, 8) = min(OPEN(j, 8), exp_array(i, 5));
                    if OPEN(j, 8) == exp_array(i, 5)
                        OPEN(j, 4) = xNode;
                        OPEN(j, 5) = yNode;
                        OPEN(j, 6) = exp_array(i, 3);
                        OPEN(j, 7) = exp_array(i, 4);
                    end
                    flag = 1;
                end;
            end;
            if flag == 0
                OPEN_COUNT = OPEN_COUNT + 1;
                OPEN(OPEN_COUNT, :) = insert_open(exp_array(i, 1), exp_array(i, 2), xNode, yNode, exp_array(i, 3), exp_array(i, 4), exp_array(i, 5));
            end;
        end;
        index_min_node = min_fn(OPEN, OPEN_COUNT, high_target_x, high_target_y);
        if (index_min_node ~= -1)
            xNode = OPEN(index_min_node, 2);
            yNode = OPEN(index_min_node, 3);
            path_cost = OPEN(index_min_node, 6);
            CLOSED_COUNT = CLOSED_COUNT + 1;
            CLOSED(CLOSED_COUNT, 1) = xNode;
            CLOSED(CLOSED_COUNT, 2) = yNode;
            OPEN(index_min_node, 1) = 0;
        else
            NoPath = 0;
        end;
    end;

    i = size(CLOSED, 1);
    Optimal_path = [];
    xval = CLOSED(i, 1);
    yval = CLOSED(i, 2);
    
    if ( (xval == high_target_x) && (yval == high_target_y))
        path_found = true;
        
        i = 1;
        Optimal_path(i, 1) = xval;
        Optimal_path(i, 2) = yval;
        i = i + 1;
        
        inode = 0;
        parent_x = OPEN(node_index(OPEN, xval, yval), 4);
        parent_y = OPEN(node_index(OPEN, xval, yval), 5);
        
        while (parent_x ~= high_start_x || parent_y ~= high_start_y)
            Optimal_path(i, 1) = parent_x;
            Optimal_path(i, 2) = parent_y;
            inode = node_index(OPEN, parent_x, parent_y);
            parent_x = OPEN(inode, 4);
            parent_y = OPEN(inode, 5);
            i = i + 1;
        end;
        
        % Convert the path back to the original grid coordinates
        x_list = Optimal_path(:, 1) / grid_resolution;
        y_list = Optimal_path(:, 2) / grid_resolution;
        
        % Reverse the lists
        x_list = flip(x_list);
        y_list = flip(y_list);
        
        if interpolation_distance > 0
            [x_list, y_list] = interpolate_path(x_list, y_list, interpolation_distance);
        end
        
    else
        path_found = false;
        x_list = [];
        y_list = [];
    end
    
    if ~path_found
       x_list = [];
       y_list = [];
    end
end

function h_val = distance(xNode, yNode, xTarget, yTarget, allow_diagonal)
    % Distance heuristic function (Manhattan distance or Diagonal)
    if allow_diagonal
        dx = abs(xTarget - xNode);
        dy = abs(yTarget - yNode);
        h_val = min(dx, dy) * 1.414 + abs(dx - dy);
    else
        h_val = abs(xTarget - xNode) + abs(yTarget - yNode);
    end
end

function exp_array = expand_array(xNode, yNode, path_cost, xTarget, yTarget, CLOSED, MAX_X, MAX_Y, allow_diagonal)
    % Function to generate the expanded array of possible next nodes
    % Returns Array of the possible nodes to visit
    % EXPANDED ARRAY FORMAT
    % --------------------------------
    % |X val |Y val ||h(n) |g(n)|f(n)|
    % --------------------------------
    
    exp_array = [];
    exp_count = 0;
    
    % Define possible movements (up, down, left, right, diagonals)
    if allow_diagonal
        moves = [0, 1; 0, -1; 1, 0; -1, 0; 1, 1; 1, -1; -1, 1; -1, -1];
    else
        moves = [0, 1; 0, -1; 1, 0; -1, 0];
    end
    
    for i = 1:size(moves, 1)
        new_x = xNode + moves(i, 1);
        new_y = yNode + moves(i, 2);
        
        % Check if the new coordinates are within the map bounds
        if (new_x >= 1 && new_x <= MAX_X && new_y >= 1 && new_y <= MAX_Y)
            
            % Check if the new coordinates are not in the closed list
            is_in_closed = any(CLOSED(:, 1) == new_x & CLOSED(:, 2) == new_y);
            
            if ~is_in_closed
                exp_count = exp_count + 1;
                exp_array(exp_count, 1) = new_x;
                exp_array(exp_count, 2) = new_y;
                exp_array(exp_count, 3) = path_cost + (i <= 4);
                exp_array(exp_count, 4) = distance(new_x, new_y, xTarget, yTarget, allow_diagonal);
                exp_array(exp_count, 5) = exp_array(exp_count, 3) + exp_array(exp_count, 4);
            end
        end
    end
end

function index_min = min_fn(OPEN, OPEN_COUNT, target_x, target_y)
    % Function to find the node with the minimum f(n) in the OPEN list
    % Returns the index of the node with the smallest f(n)
    % Returns -1 if the OPEN list is empty
    
    if OPEN_COUNT == 0
        index_min = -1;
        return;
    end
    
    min_f = Inf;
    index_min = -1;
    
    for i = 1:OPEN_COUNT
        if OPEN(i, 1) ~= 0
            if OPEN(i, 8) < min_f
                min_f = OPEN(i, 8);
                index_min = i;
            end
        end
    end
    
    if index_min == -1
        index_min = -1;
    end
end

function OPEN = insert_open(xVal, yVal, parent_xVal, parent_yVal, g_val, h_val, f_val)
    % Function to insert a node into the OPEN list
    % OPEN LIST FORMAT
    % --------------------------------------------------------------------------
    % IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    % --------------------------------------------------------------------------
    OPEN = [1, xVal, yVal, parent_xVal, parent_yVal, g_val, h_val, f_val];
end

function index = node_index(OPEN, xVal, yVal)
    % Function to return the array index of the node in OPEN
    % Returns -1 if the node is not found
    
    index = -1;
    for i = 1:size(OPEN, 1)
        if OPEN(i, 2) == xVal && OPEN(i, 3) == yVal
            index = i;
            return;
        end
    end
end

function [x_list_interpolated, y_list_interpolated] = interpolate_path(x_list, y_list, distance)
    % Interpolates points along a path to increase its resolution.
    %
    % Args:
    %     x_list, y_list:  Original path coordinates.
    %     distance:  Distance between interpolated points.
    %
    % Returns:
    %     x_list_interpolated, y_list_interpolated:  Interpolated path coordinates.

    x_list_interpolated = [];
    y_list_interpolated = [];
    
    if length(x_list) <= 1
        x_list_interpolated = x_list;
        y_list_interpolated = y_list;
        return;
    end

    for i = 1:(length(x_list) - 1)
        x1 = x_list(i);
        y1 = y_list(i);
        x2 = x_list(i+1);
        y2 = y_list(i+1);
        
        segment_length = sqrt((x2 - x1)^2 + (y2 - y1)^2);
        num_points = floor(segment_length / distance);
        
        if num_points > 0
            dx = (x2 - x1) / (num_points + 1);
            dy = (y2 - y1) / (num_points + 1);
            
            x_list_interpolated = [x_list_interpolated, x1];
            y_list_interpolated = [y_list_interpolated, y1];

            for j = 1:num_points
                x_interp = x1 + j * dx;
                y_interp = y1 + j * dy;
                x_list_interpolated = [x_list_interpolated, x_interp];
                y_list_interpolated = [y_list_interpolated, y_interp];
            end
        else
             x_list_interpolated = [x_list_interpolated, x1];
             y_list_interpolated = [y_list_interpolated, y1];
        end
    end
    
    x_list_interpolated = [x_list_interpolated, x2];
    y_list_interpolated = [y_list_interpolated, y2];
end
