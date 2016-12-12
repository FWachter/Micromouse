%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
% DATE CREATED: 2016-12-12
% PURPOSE: Micromouse simulator class

% Please refer to the Wiki for instructions on how to use this class
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB

%% Get Simulator
micromouse = simulator;
load('maps/20x20/orthogonal/map3.mat')
micromouse.getMap(map);

locationTraversed = -2;
localMap = micromouse.robot.map;

%% Run Simulation
while (true)
    if ~(micromouse.isFigureAlive())
        break;
    end
    
    localMap(micromouse.robot.location(1), micromouse.robot.location(2)) = locationTraversed;
    
    if (micromouse.robot.openDirections(micromouse.robot.direction) == 1)
        if ((micromouse.robot.direction == 1) && (localMap(micromouse.robot.location(1), micromouse.robot.location(2)+1) == locationTraversed)) || ... 
           ((micromouse.robot.direction == 2) && (localMap(micromouse.robot.location(1)+1, micromouse.robot.location(2)) == locationTraversed)) || ...
           ((micromouse.robot.direction == 3) && (localMap(micromouse.robot.location(1), micromouse.robot.location(2)-1) == locationTraversed)) || ...
           ((micromouse.robot.direction == 4) && (localMap(micromouse.robot.location(1)-1, micromouse.robot.location(2)) == locationTraversed))
            if (sum(micromouse.robot.openDirections) > 2)
                openDirections = micromouse.robot.openDirections;
                openDirections(micromouse.robot.direction) = 0;
                
                directions = find(openDirections == 1);
                for direction = 1:length(directions)
                    if ((directions(direction) == 1) && (localMap(micromouse.robot.location(1), micromouse.robot.location(2)+1) ~= locationTraversed)) || ... 
                       ((directions(direction) == 2) && (localMap(micromouse.robot.location(1)+1, micromouse.robot.location(2)) ~= locationTraversed)) || ...
                       ((directions(direction) == 3) && (localMap(micromouse.robot.location(1), micromouse.robot.location(2)-1) ~= locationTraversed)) || ...
                       ((directions(direction) == 4) && (localMap(micromouse.robot.location(1)-1, micromouse.robot.location(2)) ~= locationTraversed))
                        break;
                    end
                end
                micromouse.moveRobot(directions(direction));
            else
                micromouse.moveRobot(micromouse.robot.direction);
            end
        else
            micromouse.moveRobot(micromouse.robot.direction);
        end
    elseif (sum(micromouse.robot.openDirections) == 1)
        direction = find(micromouse.robot.openDirections == 1);
        micromouse.moveRobot(direction);
    else
        openDirections = micromouse.robot.openDirections;
        
        previousLocation = mod(micromouse.robot.direction+2,4);
        if (previousLocation == 0)
            previousLocation = 4;
        end
        openDirections(previousLocation) = 0;
        
        % create node here
        
        if (sum(openDirections) == 1)
            direction = find(openDirections == 1);
            micromouse.moveRobot(direction);
        else
            directions = find(openDirections == 1);
            for direction = 1:length(directions)
                if ((directions(direction) == 1) && (localMap(micromouse.robot.location(1), micromouse.robot.location(2)+1) ~= locationTraversed)) || ... 
                   ((directions(direction) == 2) && (localMap(micromouse.robot.location(1)+1, micromouse.robot.location(2)) ~= locationTraversed)) || ...
                   ((directions(direction) == 3) && (localMap(micromouse.robot.location(1), micromouse.robot.location(2)-1) ~= locationTraversed)) || ...
                   ((directions(direction) == 4) && (localMap(micromouse.robot.location(1)-1, micromouse.robot.location(2)) ~= locationTraversed))
                    break;
                end
            end
            micromouse.moveRobot(directions(direction));
        end
    end

end


