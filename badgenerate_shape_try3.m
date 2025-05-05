function [x_list, y_list] = generate_shape_prediction(param_struct)
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
    % count = 0;
    % figure;
    % hold on;
    % for i = 1:int32(param_struct.max_iter/3):(length(otherVs(1, :)))
    %     count = count + 1;
    %     V = otherVs(:, i);
    % 
    %     V_list = [param_struct.r0; V; param_struct.rn];
    %     x_list = V_list(1:2:(end-1), 1);
    %     y_list = V_list(2:2:end, 1);
    % 
    %     plot(x_list, y_list)
    %     a(count, 1) = i;
    %     title("Gradient Descent Iterations on Path Coordinates")
    %     xlabel("X-coordinates (cm)")
    %     ylabel("Y-coordinates (cm)")
    %     legend("Iteration " + a, Location="southeast")
    %     legend show;
    % 
    % end
    % 
    % %Calculate and plot the current final predicted set of coordinates
    % V = optimized_coords;
    % 
    % V_list = [param_struct.r0; V; param_struct.rn];
    % x_list = V_list(1:2:(end-1), 1);
    % y_list = V_list(2:2:end, 1);
    % plot(x_list, y_list, LineWidth = 3)
    % 
    % %%%%RUN GRADIENT DESCENT AGAIN WITH THE NEW COORDS AS START GUESSES %%%
    % %Goal: Rerun gradient descent with parameters more tuned to a
    % % %lower-intensity gradient to make the final step to completion. 
    % [optimized_coords, otherVs] = run_gradient_descent(f_cost, V_list,param_struct);

    %plot the progression (theoretically)
    count = 0;
    figure;
    hold on;
    for i = 1:int32(param_struct.max_iter/5):(length(otherVs(1, :)))
        count = count + 1;
        V = otherVs(:, i);

        V_list = [param_struct.r0; V; param_struct.rn];
        x_list = V_list(1:2:(end-1), 1);
        y_list = V_list(2:2:end, 1);

        plot(x_list, y_list)
        a(count, 1) = i;
        title("Secondary Gradient Descent Iterations on Path Coordinates")
        xlabel("X-coordinates (cm)")
        ylabel("Y-coordinates (cm)")
        legend("Iteration " + a, Location="southeast")
        legend show;

    end

    %Calculate and plot the final predicted set of coordinates
    V = optimized_coords;
        
    V_list = [param_struct.r0; V; param_struct.rn];
    x_list = V_list(1:2:(end-1), 1);
    y_list = V_list(2:2:end, 1);
    plot(x_list, y_list, LineWidth = 3)

    %plot obstacles
    theta = 0:0.1:2*pi;
    
    for i = 1:length(param_struct.obs_r)
        xobs = cos(theta) .* param_struct.obs_r(i) + param_struct.obs_x(i);
        yobs = sin(theta) .* param_struct.obs_r(i) + param_struct.obs_y(i);
        plot(xobs, yobs, "r-");
    end
end
clear;
clf;

%%%% path planning parameters %%%%
param_struct = struct;
param_struct.r0 = [0; -3]; %start coords
param_struct.rn = [12; 3]; %end coords
param_struct.num_links = 500;
param_struct.obs_x = [6 ]; %[2 7];
param_struct.obs_y = [1]; %[3 1];
param_struct.obs_r = [5]; %[1 2]; 

%%%% Gradient descent parameters %%%%
param_struct.beta = 0.5; %Original: 0.005 %WHAT DOES THIS DO?
param_struct.gamma = 0.95; %Original: 0.9 %WHAT DOES THIS DO? 
param_struct.max_iter = 50;
param_struct.min_gradient = 1e-7;

param_struct.cost_coeff = 70; %25
param_struct.length_multiplier = 100; %10

param_struct.wiggle = 0; %says if we "wiggle" the start guess line. 0 = no, 1 = yes
param_struct.wiggle_mag = 1; %says the maximum amplitude of the wiggle



%%%%%%%%%%%%%%%%%%%%%% Run Gradient Descent %%%%%%%%%%%%%%%%%%%%%%

%Gradient descent algorithm

%INPUTS:
%fun: the function we want to optimize
%V0: the initial guess for gradient descent
%params: a struct defining the optimization parameters
%params.beta: threshold for choosing alpha (step size scaling)
    %via backtracking line-search
%params.gamma: growth/decay multiplier for backtracking line-search
%params.max_iter: maximum number of iterations for gradient descent
%params.min_gradient: termination condition for gradient descent

%OUTPUTS:
%Vopt: The guess for the local minimum of V0

function [Vopt, allVs] = run_gradient_descent(fun,V0,params)
    total_iter = 0;
    %unpack params
    beta = params.beta;
    gamma = params.gamma;
    max_iter = params.max_iter;
    min_gradient = params.min_gradient;
    
    %set initial values for alpha, V, and n
    alpha = 1; 
    %V = zeros(length(V0), 1);
    V = V0; 
    n = 0;
    allVs = zeros(length(V), 10);
    allVs(:, 1) = V;

    %evaluate gradient and function for first time
    G = approximate_gradient(fun,V);
    F = fun(V);

    %iterate until either gradient is sufficiently small
    %or we hit the max iteration limit
    while n<max_iter && norm(G)>min_gradient

        %compute the square of the norm of the gradient
        NG2 = norm(G)^2;
    
        %run line search algorithm to find alpha
        while fun(V-alpha*G)<F-beta*alpha*NG2
            alpha = alpha/gamma;
        end
    
        while fun(V-alpha*G)>F-beta*alpha*NG2
            alpha = alpha*gamma;
        end
        
        %once alpha has been found, update guess
        V = V-alpha*G;
        
        %evaluate gradient and function at new value of V
        G = approximate_gradient(fun,V);
        F = fun(V);

        %increment our iteration counter and store the allVs
        n = n+1;
        allVs(:, n+1) = V;
        total_iter = n;
    end
    total_iter
    %return final value of V as our numerical solution
    Vopt = V;
end



% Need to create cost function
% 
% Key idea: Cost is proportional to total length of bridge. 
% 
% Cost = sum (all vertex-vertex lengths)
% 
% To calculate vertex-vertex lengths, you just find the distance between the vertex. 
%     HOWEVER, if the vertex-vertex travel passes through a forbidden area, THEN the length is absolutely massive. 


%%%%%%%%%%%%%%%%%% Vector-vector distance function %%%%%%%%%%%%%%%%%
%Inputs: 
%   Vector 1 x
%   Vector 1 y
%   Vector 2 x
%   Vector 2 y
%Outputs:
%   Vector-vector distance
function dist = find_dist(x1, y1, x2, y2)
    dist = sqrt((x2-x1)^2 + (y2-y1)^2);
end



%%%%%%%%%%%%%%%%%% Length-finding %%%%%%%%%%%%%%%%%
%Inputs: (NOTE: With all obstacles circular)
%   vx: Vector with X-coordinates of vertecies
%   vy: Vector with Y-coordinates of vertecies
%   obs_x: Vector with X-coords of obstacles
%   obs_y: Vector with Y-coords of obstacles
%   obs_r: Vector with radiuses of obstacles

%Output: All Vertex-vertex lengths (with massive addition to length if
%crosses through impossible places)

function full_length = find_lengths(vector, param_struct)
    
    V_list = [param_struct.r0; vector; param_struct.rn];
    vx = V_list(1:2:(end-1), 1);
    vy = V_list(2:2:end, 1);

    lengths = zeros(length(vx)-1, 1);

    %find the vertex-vertex length for all inputs
    for i = 1 : length(lengths)
        %Find vertex-vertex length
        
        lengths(i) = (1*find_dist(vx(i), vy(i), vx(i+1), vy(i+1)) );
        
        if(lengths(i) > 0.05)
            lengths(i) = lengths(i) + 2^(lengths(i)/0.05); %^param_struct.length_multiplier;
        end
        
      
        %Calculate to see if the line and the obstacles ever intersect. If
        %they do, add penalty. 
        
        for j = 1:length(param_struct.obs_r)
            

            dist_from_obs_center = find_dist(vx(i+1), vy(i+1), param_struct.obs_x(j), param_struct.obs_y(j));
            lengths(i) =  lengths(i) + param_struct.cost_coeff*exp(max((param_struct.obs_r(j)) - dist_from_obs_center, 0));

            %Vector-vector distance can NOT exceed a certain value.
            % %Finding if the line and circle intersect by making line
            % %parametric equation, then substituting it in to the circle
            % %equation, then making circle equation into quadratic with
            % %input variable of t-step along the line, then taking
            % %discriminant of that equation. 
            % 
            % %Basically looking to see if there's any step along the line
            % %which will intersect with the circle.
            % 
            % %POSSIBLE PROBLEM: Assumes that line extends indefinitely, when
            % %it does NOT. 
            % 
            % %Fix: Check to see if the t-values of the solutions are  between 0
            % %and 1. If it is, that means that it is between the first point
            % %annd the second
            % 
            % %calculate a, b, and c
            % A = (vx(i+1) - vx(i))^2 + (vy(i+1) - vy(i))^2;
            % B = 2*((vx(i+1) - vx(i)) * (vx(i) - param_struct.obs_x(j)) + (vy(i+1) - vy(i)) * (vy(i) - param_struct.obs_y(j)));
            % C = (vx(i) - param_struct.obs_x(j))^2 + (vy(i) - param_struct.obs_y(j))^2 - (param_struct.obs_r(j))^2;
            % 
            % discriminant = B^2 - 4*A*C;
            % 
            % if (discriminant >= 0) %if it seems the infinite line intersects
            % 
            %     %Check that the line SEGMENT actually intersects
            %     t1 = (-B + sqrt(discriminant)) / (2*A);
            %     t2 = (-B - sqrt(discriminant)) / (2*A);
            % 
            %     %If both T1 and T2 are between 0 and 1 (inclusive) THEN the
            %     %intersection takes place on the line segment.
            %     if((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1))
            %         lengths(i) = lengths(i) + 1000;
            %     end
            % end
        end
        
    end
    full_length = sum(lengths);

    % for i = 1:length(lengths)
    %     if(lengths(i) >full_length/50)
    %         lengths(i) = lengths(i) * 2^(lengths(i)/(full_length/50)); %^param_struct.length_multiplier;
    %     end
    % end
    full_length
end







[x, y] =  generate_shape_prediction(param_struct);

%%%%%%%%%%%%%%%%%%%%% PLOT THE FINAL LINE AND THE OBSTACLES
theta = 0:0.1:2*pi;
% 
% figure; 
% hold on;
% plot(x, y)
% 
% for i = 1:length(param_struct.obs_r)
%     xobs = cos(theta) .* param_struct.obs_r(i) + param_struct.obs_x(i);
%     yobs = sin(theta) .* param_struct.obs_r(i) + param_struct.obs_y(i);
%     plot(xobs, yobs, "r-");
% end


%FLOWCHART FOR CODE
%   1. Put inputs in to param_struct
%   2. Run generate_shape_prediction with param_struct as input
%   3. As part of gsp, generate single vector which includes both x and y,
%   giving guess coordinates for the final shape. 
%   4. As part of gsp, set  cost function with param_struct input
%   5. As part of gsp, run gradient descent using the cost function, guess
%   coords, and gradient descent  parameters given as part of gsp
%   6. To run gradient descent, the gradient descent function finds the
%   gradient of the function at  its initial value, then moves opposite the
%   gradient, then retests to find the gradient of the function at the new
%   point, retests, moves again, and repeats until it reaches a gradient
%   magnitude which is accepable. 


%%%%%%%%%%%%
%Ways to test code: 
%   1. Eliminate all obstacles, make it so that guess coordinates are not a
%   straight line, check that the function will make it converge on a
%   straight line. 
%   2. 