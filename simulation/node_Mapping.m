%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
% DATE CREATED: 2016-12-12
% PURPOSE: Micromouse simulator class

% Please refer to the Wiki for instructions on how to use this class
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB

% FUTURE WORK: Dubplicate nodes are still being made for map3
%              Optimize pure backtrack movements (probably need subset search)

%% Prepare Simulation

dbstop if error;

% Get Neccessary Classes
sim   = simulator;
nodes = node;
robot = micromouse;

% Load Map Into Simulation
displaySimulation = 1;
displayResults    = 1;
load('maps/33x33/orthogonal/map1.mat');
sim.getMap(map, displaySimulation);

% Initialize Variables
skipMove           = 0;
backtrack          = 0;
removeLoop         = 0;
stackIndex         = 0;
backToStart        = 0;
dontRemoveNodes    = 0;
goalPositionFound  = 0;
previousPosition   = zeros(1, 2);
previousLocations  = zeros(5, 2);
robotStartLocation = sim.robot.location;

pause();

nodes.addNode(robotStartLocation, zeros(1, 4));
stackIndex = stackIndex + 1;

% Make First Robot Movement
sim.moveRobot(sim.robot.direction);
robot.addCurrentLocation();

%% Run Simulation
while (true)
    
    if ((displaySimulation) && ~(sim.isFigureAlive())) % check if simulation is still alive
        break;
    end
    
    if ((sum(previousPosition == sim.robot.location) ~= 2) || (skipMove)) % if the robot has moved in the simulation
        if (skipMove)
            skipMove = 0;
        else
            previousPosition = sim.robot.location; % reset previous position
        
            % Get Parameters From Simulation
            robot.location       = sim.robot.location;
            robot.direction      = sim.robot.direction;
            robot.openDirections = sim.robot.openDirections;
            robot.addCurrentLocation();
        end
        
        if (backtrack) % if the robot is backtracking through the node structure
            if ((sum(nodes.getPreviousNodeLocation() == robot.location) == 2) && (removeLoop == 0)) % if the robot is at the previous node
                previousOpenDirections = nodes.getPreviousNodeOpenDirections();
                totalOpenDirections = sum(previousOpenDirections);
                
                if (totalOpenDirections == 0) % if there are no other options to move to from the previous node move to the node before that
                    if (dontRemoveNodes)
                        nodes.popStack();
                    else
                        if (sum(robot.openDirections) > 2)
                            dontRemoveNodes = 1;
                            nodes.popStack();
                        else
                            nodes.removePreviousNode();
                        end
                    end
                    stackIndex = stackIndex - 1;
                    
                    if (sum(robotStartLocation == robot.location) == 2) % if the robot is back at the start location
                        fprintf('Robot back at start location, simultion finished\n');
                        backToStart = 1;
                        break;
                    end
                    
                    robot.direction = robot.getTravelDirection(robot.location, nodes.getPreviousNodeLocation());
                else
                    backtrack = 0;
                    
                    if (totalOpenDirections == 1) % if there is only one option for the robot to move, use that option
                        robot.direction = find(previousOpenDirections == 1);
                    else
                        robot.direction = robot.chooseRandomDirection(previousOpenDirections);
                    end
                    nodes.removeNodeDirection(robot.direction);
                end
            elseif (removeLoop)
                locations  = [];
                innerLoops = [];
                
                nodesInLoop = (stackIndex+1) - stackRef;
                for index = 1:nodesInLoop
                    openDirections = nodes.getPreviousNodeOpenDirections();
                    [location, mapIndex, stackRef] = nodes.popStack();
                    locations = [locations; location];
                    
                    if ((size(innerLoops, 1) > 0) && (sum(locations(end, :) == innerLoops(end, 1:2)) == 2))
                        locations = locations(1:innerLoops(end, 3), :);
                        innerLoops = innerLoops(1:(end-1), :);
                    end

                    if (stackRef ~= stackIndex)
                        innerLoops = [innerLoops; location, index];
                    else
                        totalOpenDirections = sum(openDirections);
                        if (totalOpenDirections ~= 0)
                            for location = 1:size(locations, 1);
                                robot.direction = robot.getTravelDirection(robot.location, locations(location, :));
                                while (sum(robot.location == locations(location, :)) ~= 2)
                                    sim.moveRobot(robot.direction);
                                    robot.location = robot.addCurrentLocation(sim.robot.location);
                                end
                            end
                            
                            if (totalOpenDirections == 1) % if there is only one movement direction left, choose that one
                                robot.direction = find(openDirections == 1);
                            else
                                robot.direction = robot.chooseRandomDirection(openDirections); 
                                % do smarter decision instead of random
                            end
                            nodes.addStack(locations(location, :), mapIndex);
                            nodes.removeNodeDirection(robot.direction);
                            
                            backtrack  = 0;
                            break;
                        end
                    end
                    stackIndex = stackIndex - 1;
                end
                
                if (backtrack)
                    robot.direction = robot.getTravelDirection(robot.location, nodes.getPreviousNodeLocation());
                end
                removeLoop = 0;
            end
        elseif (sum(robot.openDirections(1, :)) == 1) % if the robot hit a dead end
            backtrack = 1;
            dontRemoveNodes = 0; 
            robot.direction = robot.getOppositeDirection(robot.direction);
        elseif ((robot.openDirections(robot.direction) ~= 1) || (sum(robot.openDirections) > 2))
            [nodeExists, index, stackRef] = nodes.checkNodeExists(robot.location);
            if (nodeExists)
                if (stackRef > stackIndex) 
                    error('Stack reference is greater than current stack index');
                end
                % if statement above is for personal reason
                    
                nodes.addStack(robot.location, index);
                stackIndex = stackIndex + 1;
                
                nodes.removeNodeDirection(robot.getOppositeDirection(robot.direction));
                robot.openDirections = nodes.getNodeOpenDirections(index);
                robot.openDirections(robot.getOppositeDirection(robot.direction)) = 0;
                
                if (robot.openDirections(robot.direction) == 1)
                    nodes.removeNodeDirection(robot.direction);
                else
                    skipMove        = 1;
                    backtrack       = 1;
                    removeLoop      = 1;
                    dontRemoveNodes = 1;
                    nodes.popStack();
                    stackIndex = stackIndex - 1;
                end
            else
                robot.openDirections(robot.getOppositeDirection(robot.direction)) = 0;
                if (robot.openDirections(robot.direction) ~= 1) % if the robot is not able to move forward
                    if (sum(robot.openDirections) == 1) % if there is only one movement direction left, choose that one
                        robot.direction = find(robot.openDirections == 1);
                    else
                        robot.direction = robot.chooseRandomDirection(robot.openDirections); 
                        % do smarter decision instead of random
                    end
                end
                
                robot.openDirections(robot.direction) = 0;
                nodes.addNode(robot.location, robot.openDirections);
                stackIndex = stackIndex + 1;
            end
        end
        
        if ~(skipMove)
            sim.moveRobot(robot.direction);
        end
        
        if ((~goalPositionFound) && (robot.isAtGoalLocation)) % if the robot is at the goal position
            goalPositionFound = 1;
            sim.displayGoal(robot.location);
            nodes.goal.location = robot.location;
            
            for index = 1:4 % remove nodes that occupy goal area
                nodes.popStack();
                stackIndex = stackIndex - 1;
            end
            
            backtrack = 1;
            dontRemoveNodes = 1;
            robot.direction = robot.getTravelDirection(robot.location, nodes.getPreviousNodeLocation());
            
            sim.moveRobot(robot.direction);
        end
    end
    
end

if (backToStart)
    fprintf('Traveled %d movements to search maze\n', sim.robot.movements);
    
    if (displayResults)
        astar = AStar_Structure_Fast;
        astar.runMap(sim.robot.map, map.criteria);
        astar.removeSolution();
        astar.displayOptimizedSolution;
    end
end