%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
% DATE CREATED: 2016-12-12
% PURPOSE: Micromouse simulator class

% Please refer to the Wiki for instructions on how to use this class
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB

% This was an initial test algorithm for mapping to test the simulator

%% Get Simulator
micromouse = simulator;
load('maps/33x33/orthogonal/map1.mat')
micromouse.getMap(map);

locationTraversed = -1;
localMap = zeros(size(micromouse.robot.map));

pause();

%% Run Simulation
while (true)
    
    if ~(micromouse.isFigureAlive())
        break;
    end
    
    localMap(micromouse.robot.location(1), micromouse.robot.location(2)) = locationTraversed;
    
    % mark dead ends -> requires path planning to determine optimal
    % location to solution or path planning to explore areas that have not
    % already been explored
    
    if (micromouse.robot.openDirections(micromouse.robot.direction) == 1)
    % If the robot is able to move in the same direction as it was previously traveling
        if ((micromouse.robot.direction == 1) && (localMap(micromouse.robot.location(1), micromouse.robot.location(2)+1) == locationTraversed)) || ... 
           ((micromouse.robot.direction == 2) && (localMap(micromouse.robot.location(1)+1, micromouse.robot.location(2)) == locationTraversed)) || ...
           ((micromouse.robot.direction == 3) && (localMap(micromouse.robot.location(1), micromouse.robot.location(2)-1) == locationTraversed)) || ...
           ((micromouse.robot.direction == 4) && (localMap(micromouse.robot.location(1)-1, micromouse.robot.location(2)) == locationTraversed))
        % If the direction the robot is traveling in has already been traversed
            if (sum(micromouse.robot.openDirections) > 2)
            % If there are other options available
                openDirections = micromouse.robot.openDirections;
                openDirections(micromouse.robot.direction) = 0;
                
                directions = find(openDirections == 1);
                directionChosen = 0;
                for direction = 1:length(directions)
                    if ((directions(direction) == 1) && (localMap(micromouse.robot.location(1), micromouse.robot.location(2)+1) ~= locationTraversed)) || ... 
                       ((directions(direction) == 2) && (localMap(micromouse.robot.location(1)+1, micromouse.robot.location(2)) ~= locationTraversed)) || ...
                       ((directions(direction) == 3) && (localMap(micromouse.robot.location(1), micromouse.robot.location(2)-1) ~= locationTraversed)) || ...
                       ((directions(direction) == 4) && (localMap(micromouse.robot.location(1)-1, micromouse.robot.location(2)) ~= locationTraversed))
                        directionChosen = 1;
                        break;
                    end
                end
                if (directionChosen == 1)
                    micromouse.moveRobot(directions(direction));
                else
                    direction = randi([1,length(directions)],1);
                    micromouse.moveRobot(directions(direction));
                end
            else
                micromouse.moveRobot(micromouse.robot.direction);
            end
        else
        % Travel in the same direciton as previously
            micromouse.moveRobot(micromouse.robot.direction);
        end
    elseif (sum(micromouse.robot.openDirections) == 1)
    % If the robot has only one option for open spaces to move to
        direction = find(micromouse.robot.openDirections == 1);
        micromouse.moveRobot(direction);
    else
    % If the robot has multiple options for open spaces to move to
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
            directionChosen = 0;
            for direction = 1:length(directions)
                if ((directions(direction) == 1) && (localMap(micromouse.robot.location(1), micromouse.robot.location(2)+1) ~= locationTraversed)) || ... 
                   ((directions(direction) == 2) && (localMap(micromouse.robot.location(1)+1, micromouse.robot.location(2)) ~= locationTraversed)) || ...
                   ((directions(direction) == 3) && (localMap(micromouse.robot.location(1), micromouse.robot.location(2)-1) ~= locationTraversed)) || ...
                   ((directions(direction) == 4) && (localMap(micromouse.robot.location(1)-1, micromouse.robot.location(2)) ~= locationTraversed))
                    directionChosen = 1;
                    break;
                end
            end
            if (directionChosen == 1)
                micromouse.moveRobot(directions(direction));
            else
                direction = randi([1,length(directions)],1);
                micromouse.moveRobot(directions(direction));
            end
        end
    end

end


