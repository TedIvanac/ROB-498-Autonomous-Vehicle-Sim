function waypoints = waypoints(path, sample_factor, condition)

persistent waypointListX
persistent waypointListY
persistent iteration_count
persistent done

x = inf;
y = inf;

if condition == 0
    done = [];
end

if isempty(done)

    if condition == 0
        waypointListX = path(1:sample_factor:end,1);
        waypointListY = path(1:sample_factor:end,2);

        if isempty(iteration_count)
            iteration_count = 1;
            x = waypointListX(1);
            y = waypointListY(1);         
            waypointListX(1) = [];
            waypointListY(1) = [];
        else
            waypointListX(1:iteration_count) = [];
            waypointListY(1:iteration_count) = [];
            x = waypointListX(1);
            y = waypointListY(2);
        end 

        elseif ~isempty(waypointListX) && condition == 1
            iteration_count = iteration_count + 1;
            x = waypointListX(1);
            y = waypointListY(1);
        
            waypointListX(1) = [];
            waypointListY(1) = [];
    end
    
    elseif condition == 2
        done = 1;
end


waypoints = [x,y];
end

