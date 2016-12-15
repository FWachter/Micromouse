%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
% DATE CREATED: 2016-12-12
% PURPOSE: Micromouse simulator class

% Please refer to the Wiki for instructions on how to use this class
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB

% ISSUE: Dubplicate nodes are still being made for map3

%% Prepare Simulation

% Get Neccessary Classes
sim   = simulator;
nodes = node;
robot = micromouse;
astar = AStar_Structure_Fast;

% Load Map Into Simulation
load('maps/33x33/orthogonal/map1.mat');
sim.getMap(map);

% Initialize Variables
backtrack          = 0;
backToStart        = 0;
dontRemoveNodes    = 0;
goalPositionFound  = 0;
previousPosition   = zeros(1, 2);
previousLocations  = zeros(5, 2);
robotStartLocation = sim.robot.location;

pause();

% Make First Robot Movement
sim.moveRobot(sim.robot.direction);
nodes.addNode(robotStartLocation, zeros(1, 4));

%% Run Simulation
while (true)
    
    if ~(sim.isFigureAlive()) % check if simulation is still alive
        break;
    end
    
    if (sum(previousPosition == sim.robot.location) ~= 2) % if the robot has moved in the simulation
        previousPosition = sim.robot.location; % reset previous position
        
        % Get Parameters From Simulation
        robot.location       = robot.addCurrentLocation(sim.robot.location);
        robot.direction      = sim.robot.direction;
        robot.openDirections = sim.robot.openDirections;
        openDirections = robot.openDirections;
        
        % Check For Goal Position
        if (robot.isAtGoalLocation)
            goalPositionFound = 1;
            sim.displayGoal(robot.location);
            nodes.goal.location = robot.location;
            
            for node = 1:4 % remove nodes that occupy goal area
                nodes.removePreviousNode();
            end
            clear node;
            
            backtrack = 1;
            dontRemoveNodes = 1;
            robot.direction = robot.getTravelDirection(robot.location, nodes.getPreviousNodeLocation());
        end
        
        if (backtrack) % if the robot is backtracking through the node structure
            if (sum(nodes.getPreviousNodeLocation() == robot.location) == 2) % if the robot is at the previous node
                previousOpenDirections = nodes.getPreviousNodeOpenDirections();
                totalOpenDirections = sum(previousOpenDirections);
                
                if (totalOpenDirections == 0) % if there are no other options to move to from the previous node move to the node before that
                    if (dontRemoveNodes)
                        nodes.popStack();
                    else
                        nodes.removePreviousNode();
                    end
                    
                    if (sum(robotStartLocation == robot.location) == 2) % if the robot is back at the start location
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
                clear previousOpenDirections totalOpenDirections;
            else
            end
        elseif (sum(robot.openDirections(1, :)) == 1) % if the robot hit a dead end
            backtrack = 1;
            dontRemoveNodes = 0;
            robot.direction = robot.getOppositeDirection(robot.direction);
        elseif ((robot.openDirections(robot.direction) ~= 1) || (sum(robot.openDirections) > 2))
            [nodeExists, index] = nodes.checkNodeExists(robot.location);
            if (nodeExists)
                nodes.addStack(robot.location, index);
                nodes.removeNodeDirection(robot.getOppositeDirection(robot.direction));
                robot.openDirections = nodes.getNodeOpenDirections(index);
            end
            
            robot.openDirections(robot.getOppositeDirection(robot.direction)) = 0; % make sure the robot does not choose to move backwards
            totalOpenDirections = sum(robot.openDirections);
            
            if (totalOpenDirections ~= 0) % if there are available movements
                if (robot.openDirections(robot.direction) ~= 1) % if the robot is not able to move forward
                    if (sum(robot.openDirections) == 1) % if there is only one movement direction left, choose that one
                        robot.direction = find(robot.openDirections == 1);
                    else
                        robot.direction = robot.chooseRandomDirection(robot.openDirections);
                    end
                end
                
                if ~(nodeExists) % if the node does not already exist
                    robot.openDirections(robot.direction) = 0;
                    nodes.addNode(robot.location, robot.openDirections);
                else
                    nodes.removeNodeDirection(robot.direction);
                end
            else
                backtrack = 1;
                dontRemoveNodes = 1;
                nodes.popStack();
                robot.direction = robot.getOppositeDirection(robot.direction);
            end
        end
        
        sim.moveRobot(robot.direction);
    end
    
end

if (backToStart)
    astar.runMap(sim.robot.map, map.criteria);
    astar.removeSolution();
    astar.displayOptimizedSolution;
end

