%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
% DATE CREATED: 2016-12-12
% PURPOSE: Micromouse simulator class

% Please refer to the Wiki for instructions on how to use this class
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB

%% Get Simulator
micromouse = simulator;
load('maps/20x20/orthogonal/map1.mat')
micromouse.getMap(map);

previousPosition = [0, 0];
openDirections = zeros(2, 4);
localMap = zeros(size(micromouse.robot.map));
localMap(micromouse.robot.location(1), micromouse.robot.location(2)) = localMap(micromouse.robot.location(1), micromouse.robot.location(2)) + 1;

pause();

%% Run Simulation
while (true)
    
    if ~(micromouse.isFigureAlive())
        break;
    end
    
    if (sum(previousPosition == micromouse.robot.location) ~= 2)
        previousPosition = micromouse.robot.location; % reset previous position
        
        % Get Parameters from Simulation
        openDirections(1, :) = micromouse.robot.openDirections;
        robotDirection = micromouse.robot.direction;
        robotPosition  = micromouse.robot.location;
        
        % Update the Local Map and Get Map Data for Relevent Positions
        localMap(robotPosition(1), robotPosition(2)) = localMap(robotPosition(1), robotPosition(2)) + 1;
        openDirections(2, :) = [localMap(robotPosition(1), robotPosition(2)+1), localMap(robotPosition(1)+1, robotPosition(2)), ...
                                localMap(robotPosition(1), robotPosition(2)-1), localMap(robotPosition(1)-1, robotPosition(2))];
        
        if ((openDirections(1, robotDirection) == 1) && (openDirections(2, robotDirection) == 0)) % if the robot can move forward
            micromouse.moveRobot(robotDirection);
        elseif (sum(openDirections(1, :)) == 1) % if there is only one option for the robot to move to
            micromouse.moveRobot(find(openDirections(1,:) == 1));
        else

            % CREATE NODE
            
            % Get Comparison Matrix to Determine Best Location to Move to
            comparisonMatrix = [];
            for index = 1:4
                if (openDirections(1, index) == 1)
                    comparisonMatrix = [comparisonMatrix; index, openDirections(2, index)];
                end
            end
            minValue = min(comparisonMatrix(:, 2));
            minValueIndex = find(comparisonMatrix(:, 2) == minValue);

            if (length(minValueIndex) == 1) % if there is one optimum movement
                micromouse.moveRobot(comparisonMatrix(minValueIndex, 1));
            else
                
                % OPTIMAL SOLUTION IS TO INITIATE PATH PLANNER TO AREA WITH
                % MINIMAL AMOUNT OF KNOWN INFORMATION. MOVE THE A PREVIOUS
                % NODE WITH A DIRECTION THAT HAS NOT BEEN TRAVERSED
                
                if (sum(comparisonMatrix(:, 1) == robotDirection) == 1) % if the robot moving forward is one of the optimum movements
                    micromouse.moveRobot(robotDirection);
                else
                    
                    % AVOID MOVING BACKWARDS OR INITIATE A PATH PLANNER
                    
                    backwardsDirection = mod(robotDirection+2,4);
                    if (backwardsDirection == 0); backwardsDirection = 4; end
                    
                    if (sum(comparisonMatrix(:, 1) == backwardsDirection) == 1) % if moving backwards is one of the optimum choises
                        % Make the Backwards Movement Have a Higher Cost
                        backwardsIndex = find(comparisonMatrix(:, 1) == backwardsDirection);
                        comparionMatrix(backwardsIndex, 2) = minValue+1;
                        
                        minValueIndex = find(comparisonMatrix(:, 2) == minValue);
                        if (length(minValueIndex) == 1) % if there is one optimum movement
                            micromouse.moveRobot(comparisonMatrix(minValueIndex, 1));
                        else
                            randIndex = randi([1, length(minValueIndex)], 1);
                            micromouse.moveRobot(comparisonMatrix(minValueIndex(randIndex), 1));
                        end
                    else
                        randIndex = randi([1, length(minValueIndex)], 1);
                        micromouse.moveRobot(comparisonMatrix(minValueIndex(randIndex), 1));
                    end
                end
            end
        end
    end
    
end


