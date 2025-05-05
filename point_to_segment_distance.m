function dist = point_to_segment_distance(x1, y1, x2, y2, px, py)
    % Calculates the shortest distance from a point to a line segment.
    % (Implementation from previous response)
     segment_length_sq = (x2 - x1)^2 + (y2 - y1)^2;
    if segment_length_sq == 0
        dist = sqrt((px - x1)^2 + (py - y1)^2);
        return;
    end
    t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / segment_length_sq;
    if t < 0
        dist = sqrt((px - x1)^2 + (py - y1)^2);
    elseif t > 1
        dist = sqrt((px - x2)^2 + (py - y2)^2);
    else
        closest_x = x1 + t * (x2 - x1);
        closest_y = y1 + t * (y2 - y1);
        dist = sqrt((px - closest_x)^2 + (py - closest_y)^2);
    end
end