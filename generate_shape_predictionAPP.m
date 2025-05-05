function [x_list, y_list] = generate_shape_predictionAPP(param_struct)
    %Creating our initial guess coords--straight line from start to finish.
    x0 = param_struct.r0(1);
    y0 = param_struct.r0(2);
    xn = param_struct.rn(1);
    yn = param_struct.rn(2);

    
    x_guess = linspace(x0,xn,param_struct.num_links+1);
    y_guess = linspace(y0,yn,param_struct.num_links+1);

    %Testing the line optimization part of the cost function by
    %making a squiggly line
    if(param_struct.wiggle == 1)
        for k = 1:length(y_guess)
            y_guess(k) = y_guess(k) + param_struct.wiggle_mag*(rand(1) - 0.5);
        end
    end

    coords_guess = zeros(2*(param_struct.num_links-1),1);

    %Assembling our initial guess coords into a version which can be
    %processed by the cost function--all position vectors are in one single
    %column vector. 
    
    for n = 1:(param_struct.num_links-1)
        coords_guess(2*n-1,1) = x_guess(n+1);
        coords_guess(2*n,1) = y_guess(n+1);
    end
    %Defining the cost func
    f_cost = @(V) find_lengths(V, param_struct);

    [optimized_coords, otherVs] = run_gradient_descent(f_cost, coords_guess,param_struct);
    
    %plot the progression (theoretically)
    count = 0;
    % figure;
    % hold on;
    for i = 1:int32(param_struct.max_iter/3):(length(otherVs(1, :)))
        count = count + 1;
        V = otherVs(:, i);

        V_list = [param_struct.r0; V; param_struct.rn];
        x_list = V_list(1:2:(end-1), 1);
        y_list = V_list(2:2:end, 1);

        % plot(x_list, y_list)
        % a(count, 1) = i;
        % title("Gradient Descent Iterations on Path Coordinates")
        % xlabel("X-coordinates (cm)")
        % ylabel("Y-coordinates (cm)")
        % legend("Iteration " + a, Location="southeast")
        % legend show;

    end

    %Calculate and plot the current final predicted set of coordinates
    V = optimized_coords;
        
    V_list = [param_struct.r0; V; param_struct.rn];
    x_list = V_list(1:2:(end-1), 1);
    y_list = V_list(2:2:end, 1);
    % plot(x_list, y_list, LineWidth = 3)

    %%%%RUN GRADIENT DESCENT AGAIN WITH THE NEW COORDS AS START GUESSES %%%
    %Goal: Rerun gradient descent with parameters more tuned to a
    % %lower-intensity gradient to make the final step to completion. 
    [optimized_coords, otherVs] = run_gradient_descent(f_cost, V_list,param_struct);
    
    %plot the progression (theoretically)
    count = 0;
    % figure;
    % hold on;
    for i = 1:int32(param_struct.max_iter/3):(length(otherVs(1, :)))
        count = count + 1;
        V = otherVs(:, i);

        V_list = [param_struct.r0; V; param_struct.rn];
        x_list = V_list(1:2:(end-1), 1);
        y_list = V_list(2:2:end, 1);

        % plot(x_list, y_list)
        % a(count, 1) = i;
        % title("Secondary Gradient Descent Iterations on Path Coordinates")
        % xlabel("X-coordinates (cm)")
        % ylabel("Y-coordinates (cm)")
        % legend("Iteration " + a, Location="southeast")
        % legend show;

    end

    %Calculate and plot the final predicted set of coordinates
    V = optimized_coords;
        
    V_list = [param_struct.r0; V; param_struct.rn];
    x_list = V_list(1:2:(end-1), 1);
    y_list = V_list(2:2:end, 1);
    % plot(x_list, y_list, LineWidth = 3)

    %plot obstacles
    theta = 0:0.1:2*pi;
    
    for i = 1:length(param_struct.obs_r)
        xobs = cos(theta) .* param_struct.obs_r(i) + param_struct.obs_x(i) + 0.5;
        yobs = sin(theta) .* param_struct.obs_r(i) + param_struct.obs_y(i) + 0.5;
        % plot(xobs, yobs, "r-");
    end
end