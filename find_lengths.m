function full_length = find_lengths(vector, param_struct)
    
    V_list = [param_struct.r0; vector; param_struct.rn];
    vx = V_list(1:2:(end-1), 1);
    vy = V_list(2:2:end, 1);

    lengths = zeros(length(vx)-1, 1);

    %find the vertex-vertex length for all inputs
    for i = 1 : length(lengths)
        %Find vertex-vertex length
        
        lengths(i) = (1*find_dist(vx(i), vy(i), vx(i+1), vy(i+1)) );

        if(lengths(i) >0.05)
            lengths(i) = lengths(i) * 2^(lengths(i)/0.05); %^param_struct.length_multiplier;
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
    full_length = sum(lengths)
end