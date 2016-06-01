%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter
% DATE CREATED: 2016-06-01
% PURPOSE: AStar algorithm class
% REFERENCE: http://ch.mathworks.com/matlabcentral/fileexchange/26248-a---a-star--search-for-path-planning-tutorial
% CONTACT INFO: wachterfreddy@gmail.com

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: www.github.com/FWchter/Micromouse/Wiki


classdef AStar_Structure_Fast < handle
    
    properties(SetAccess = protected)
        map
        cost
        robot
        state
        display
    end
   
    
%% CONSTRUCTOR METHOD
    methods
        
        function astar = AStar_Structure_Fast(varargin)
        % EXAMPLE FUNCTION CALL: astar = AStar_Structure_Fast()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Construct the structure
        % TYPE: Constructor Method
            
            % Map Properties
            astar.map.coordinates      = [];
            astar.map.MAX_X            = 0;
            astar.map.MAX_Y            = 0;
            
            astar.map.legend.obstacle  = -1;
            astar.map.legend.target    = 0;
            astar.map.legend.start     = 1;
            astar.map.legend.freeSpace = 2;
            
            astar.map.targetLocation   = [0,0];
            astar.map.startLocation    = [0,0];
            astar.map.findCriteria     = 0;
            astar.map.isPath           = 1;
            
            % Figure Properties
            astar.display.obstacleCount         = 0;
            astar.display.handles.obstacles     = [];
            astar.display.figureHandle          = -1;
            astar.display.handles.target        = -1;
            astar.display.handles.robot         = -1;
            astar.display.handles.solutionNodes = -1;
            astar.display.handles.solutionPath  = -1;
            
            % Robot Properties
            astar.robot.location = [0,0];
            astar.robot.path     = [];
            astar.robot.count    = 0;
            astar.robot.cost     = 0;
            astar.robot.goal     = 0;
            
            % Alogrithm State Properties
            astar.state.ready = -1;
            astar.state.time  = [];
            astar.state.count = 0;
            
            % Intiialize Cost Function
            astar.cost = [];
            
        end
        
    end
    
    
%% PUBLIC METHODS

    methods
        
        function solution = runMap(astar,map,criteria)
        % EXAMPLE FUNCTION CALL: solution = astar.runMap(map,criteria)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Take user varables and run master script for AStar
        % INPUTS:      map - 2D array containing obstacles, target, and start location
        %         criteria - 1) Move orthogonally, 2) Move orthogonally and diagonally
        % OUTPUTS: solution - shortest path for the robot to move to the solution
        % TYPE: Public Method
            
            % Get initial map properties
            astar.map.coordinates  = map;
            astar.map.MAX_X        = size(map,1);
            astar.map.MAX_Y        = size(map,2);
            astar.map.findCriteria = criteria;
            
            % Reset Properties
            astar.robot.path   = [];
            astar.robot.count  = 0;
            astar.map.isPath   = 1;
            
            % Run master function
            solution = astar.master();
            
        end
        
    end
        
%% PRIVATE METHODS

    methods(Access = private)
        
        function solution = master(astar)
        % EXAMPLE FUNCTION CALL: Can't run function externally
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Run the setup functions and AStar algorithm
        % TYPE: Private Method
            
            % Prepare Algorithm Parameters
            astar.getMapProperties();
            astar.initializeVariables();
            astar.state.ready = 1;
            
            astar.startAlgorithm();
            
            astar.displayMap();
            astar.displaySolution();
            
            solution = astar.robot.path;
            
        end
        
        function getMapProperties(astar)
        % EXAMPLE FUNCTION CALL: Can't run function externally
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Get object, target, and start location from the user provided map
        % TYPE: Private Method
            
            % Get start and target locations
            [xStartLocation,yStartLocation] = find(astar.map.coordinates == astar.map.legend.start);
            [xTargetLocation,yTargetLocation] = find(astar.map.coordinates == astar.map.legend.target);
            
            % Store start, target, and initial robot location into astar object
            astar.map.startLocation = [xStartLocation,yStartLocation];
            astar.map.targetLocation = [xTargetLocation,yTargetLocation];
            astar.robot.location = astar.map.startLocation;

        end

        function displayMap(astar)
        % EXAMPLE FUNCTION CALL: Can't run function externally
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Display the map with obstacles, start location, and target location
        % TYPE: Private Method
            
            % Initialize the figure
            astar.display.figureHandle = figure('Name','A* Algorithm','NumberTitle','off'); % initialize figure
            axis([1,astar.map.MAX_X+1,1,astar.map.MAX_Y+1]); % initialize axis spacing
            axis square; grid on; hold on; % set axis properties
            
            % Display obstacles, robot, and target location
            for x = 1:astar.map.MAX_X % for all map x locations
                for y = 1:astar.map.MAX_Y % for all map y locations
                    if (astar.map.coordinates(x,y) == astar.map.legend.obstacle) % if the current location is an obstacle
                        astar.display.obstacleCount = astar.display.obstacleCount + 1; % increment obstacle counter
                        astar.display.handles.obstacles(astar.display.obstacleCount) = plot(x+.5,y+.5,'ro'); % show the obstacle
                    elseif (astar.map.coordinates(x,y) == astar.map.legend.target) % if the current location is the target
                        astar.display.handles.target = plot(x+.5,y+.5,'gd'); % show the target
                    elseif (astar.map.coordinates(x,y) == astar.map.legend.start) % if the current locaiton is the robot
                        astar.display.handles.robot = plot(x+.5,y+.5,'bo'); % show the robot
                    end
                end
            end
            drawnow; % force MATLAB to draw the figure before continuing
            
        end
        
        function displaySolution(astar)
        % EXAMPLE FUNCTION CALL: Can't run function externally
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Display the solution from AStar algorithm
        % TYPE: Private Method
            
            astar.display.handles.solutionNodes = plot(astar.robot.path(:,1)+.5,astar.robot.path(:,2)+.5,'bo'); % display the solution as nodes
            astar.display.handles.solutionPath = plot(astar.robot.path(:,1)+.5,astar.robot.path(:,2)+.5,'b--'); % connect the solution nodes with a dotted line
            drawnow; % force MATLAB to draw the figure before continuing
            
        end
        
        function displayAlgorithmState(astar)
        % EXAMPLE FUNCTION CALL: Can't run function externally
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Display the current status of the algorithm
        % TYPE: Private Method
            
            if (astar.state.ready == 1) % algorithm is ready to compute
                fprintf('Starting A* Algorithm... '); % notify user that the algorithm starting
                astar.state.count = astar.state.count + 1; tic; % start timer
            elseif (astar.state.ready == 0) % algorithm has finished
                astar.state.time(astar.state.count) = toc; % store algorithm running time
                fprintf('Algorithm took %0.4f seconds to generate solution\n',astar.state.time(astar.state.count)); % notify user that the algorithm is finished
            elseif (astar.state.ready == -1) % algorithm was not setup
                error('Failure to initialize algorithm'); % notify user that the algorithm was not set up properly
            elseif (astar.state.ready == -2) % algorithm could not find a solution
                fprintf('No solution exists for given map\n'); % notify user that no solution could be found for the given map
            end

        end
        
        function initializeVariables(astar)
        % EXAMPLE FUNCTION CALL: Can't run function externally
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Initialize variables before running AStar algorithm
        % TYPE: Private Method
            
            astar.cost = @(location,target)(sqrt((target(1)-location(1))^2 + (target(2)-location(2))^2)); % set cost function
            
            % Set robot cost and goal values
            astar.robot.cost = 0;
            astar.robot.goal = astar.cost(astar.robot.location,astar.map.targetLocation);
            
            % Get user defined criteria
            switch astar.map.findCriteria
                case 1 % check only orthoginal nodes
                    astar.map.criteria = @(x,y)(((x == 0) && (y ~= 0)) || ((y == 0) && (x ~= 0)));
                case 2 % check orthogonal and diagonal nodes
                    astar.map.criteria = @(x,y)((x ~= y) || (x ~= 0));
            end
            
        end
        
        function startAlgorithm(astar)
        % EXAMPLE FUNCTION CALL: Can't run function externally
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Run the AStar algorithm
        % TYPE: Private Method
            
            % Initialize Open Node List and Counter
            openList  = [];
            openCount = 0;
            
            % Initialize Closed Node List and Counter
            closedList  = [];
            closedCount = 0;
            
            % Initialize Expand Node List and Counter
            expandList  = [];
            expandCount = 0;
            
            % Initialize Optimal Node List and Counter
            optimalList  = [];
            optimalCount = 0;  
                        
            % Set all of the objects as closed nodes
            for x = 1:MAX_X
                for y = 1:MAX_Y
                    if (astar.map.coordinates(x,y) == astar.map.legend.obstacle)
                        closedCount = closedCount + 1;
                        closedList(closedCount,1:2) = [x,y];
                    end
                end
            end            
                        
            %---------- Convert Structured Components to Non-Structures ----------
            robotLocation  = astar.robot.location;
            robotCost      = astar.robot.cost;
            robotPath      = astar.robot.path;
            robotGoal      = astar.robot.goal;
            robotCount     = astar.robot.count;
            mapIsPath      = astar.map.isPath;
            targetLocation = astar.map.targetLocation;
            startLocation  = astar.map.startLocation;
            criteria       = astar.map.criteria;
            MAX_X          = astar.map.MAX_X;
            MAX_Y          = astar.map.MAX_Y;
            cost           = astar.cost;
            
            astar.displayAlgorithmState(); % start timer
            
            % Add robot start location to open node list and set as parent node
            openCount = openCount + 1;
            openList(openCount,:) = [0,robotLocation,robotLocation,robotCost,robotGoal,robotGoal]; % add parameters to the open node list
            
            % Add robot start location to the closed node list
            closedCount = closedCount + 1; % incremenet the closed node list size counter
            closedList(closedCount,1:2) = robotLocation; % add location to the closed node list
            
            while (((robotLocation(1) ~= targetLocation(1)) || (robotLocation(2) ~= targetLocation(2))) && mapIsPath == 1)
                
                %---------- Get Potential Nodes ----------
                expandCount = 0;
                expandList = [];

                for x = [1,0,-1] % iterate through all x neightbors
                    for y = [1,0,-1] % iterate through all y neighbors
                        if criteria(x,y) % check if the node fits the desired node location criteria
                            testPosition = [robotLocation(1)+x,robotLocation(2)+y]; % store the node location
                            if (((testPosition(1) > 0) && (testPosition(1) <= MAX_X)) && ((testPosition(2) > 0) && (testPosition(2) <= MAX_Y))) % check if the node is in the bounds of the map
                                isOpen = 1; % set this location as a possible new node
                                for closedNode = 1:closedCount % seach through all of the closed nodes
                                    if (testPosition(1) == closedList(closedNode,1) && (testPosition(2) == closedList(closedNode,2))) % if the possible new node has already been transversed
                                        isOpen = 0; % this location has already been transversed and do not add to the expanded node list
                                    end
                                end
                                if (isOpen == 1) % if the node fits the desired location criteria, within the bounds of the map, and not already trasnversed
                                    expandCount = expandCount + 1; % increment the expanded node list count
                                    expandList(expandCount,1) = testPosition(1); % store the x location of the new node
                                    expandList(expandCount,2) = testPosition(2); % store the y location of the new node
                                    expandList(expandCount,3) = robotCost + cost(robotLocation,testPosition); % add the cost for the new node from the original robot position
                                    expandList(expandCount,4) = cost(targetLocation,testPosition); % store the goal distance for the new node
                                    expandList(expandCount,5) = expandList(expandCount,3) + expandList(expandCount,4); % store the fitness of the new node
                                end
                            end
                        end
                    end
                end
                
                %---------- Check Potential Nodes ----------
                for expandedNode = 1:expandCount
                    nodeExists = 0; % flag variable used to indicate if the new node already exists in the open list 
                    for openNode = 1:openCount
                        if ((expandList(expandedNode,1) == openList(openNode,2)) && (expandList(expandedNode,2) == openList(openNode,3)))
                            openList(openNode,8) = min(openList(openNode,8),expandList(expandedNode,5)); % compare the fitness of the node already in the open list with the new node
                            if (openList(openNode,8) == expandList(expandedNode,5)) % if the new node has a smaller fitness
                                openList(openNode,4) = robotLocation(1); % change the x location of the parent node to the parent of the new node
                                openList(openNode,5) = robotLocation(2); % change the y location of the parent node to the parent of the new node
                                openList(openNode,6) = expandList(expandedNode,3); % change the cost to the cost of the new node
                                openList(openNode,7) = expandList(expandedNode,4); % change the goal distance to the distance of the new node
                            end
                            nodeExists = 1; % node already exists as an open node
                        end
                    end
                    if (nodeExists == 0) % if the node is not already in the open list
                        openCount = openCount + 1;
                        openList(openCount,:) = [1,expandList(expandedNode,1:2),robotLocation,expandList(expandedNode,3:5)]; % add parameters to the open node list
                    end
                end
                
                %---------- Get Next Node ----------
                optimalCount = 0;
                optimalList = [];

                % Find potential optimal nodes
                foundTarget = 0; % flag to indicate if an optimal node is at the target location
                for openNode = 1:openCount % for all nodes in the open list
                    if (openList(openNode,1) == 1) % if the current node is not a parent of another node
                        optimalCount = optimalCount + 1; % increment count for nodes in the optimal node list
                        optimalList(optimalCount,:) = [openList(openNode,8),openNode]; % add node to optimal node list
                        if ((openList(openNode,2) == targetLocation(1)) && (openList(openNode,3) == targetLocation(2))) % if the new optimal node is at the target location
                            foundTarget = 1; % indicate we found an optimal node at the target location
                            break; % break from for loop since optimal node has been found
                        end
                    end
                end

                % Stre location of the optimal node
                if (foundTarget == 1) % if the optimal node is at the target location
                    optimalNode = openNode; % store the index of the optimal node referencing the open node list
                elseif (size(optimalList ~= 0)) % if there are optimal nodes in the optimal node list
                    [~,optimalNodeIndex] = min(optimalList(:,1)); % find the node with the minimum fitness
                    optimalNode = optimalList(optimalNodeIndex,2); % store the index of the optimal node referencing the open node list
                else % if no more paths are available
                    optimalNode = -1; % indicate that no paths are available
                end

                % Update robot position for next iterations
                if (optimalNode ~= -1) % if an optimal node exits
                    robotLocation = [openList(optimalNode,2),openList(optimalNode,3)]; % update the robot location
                    robotCost = openList(optimalNode,6); % update the cost of the robot location
                    closedCount = closedCount + 1; % incremenet the closed node list size counter
                    closedList(closedCount,1:2) = robotLocation; % add location to the closed node list
                    openList(optimalNode,1) = 0; % set the robot location as a parent node
                else % if there are no nodes left
                    mapIsPath = 0; % indicate that there are not paths left
                end
            end
            
            %---------- Get Optimal Path ----------
            [robotCount,robotPath] = addToRobotPath(robotCount,robotPath,closedList(end,:)); % store the last node in the closed node location into the robot path
            if ((robotPath(1,1) == targetLocation(1)) && (robotPath(1,2) == targetLocation(2))) % if the last node in the closed node list is the target location
                
                targetNodeLocation = find((openList(:,2) == robotPath(1,1)) & (openList(:,3) == robotPath(1,2))); % find the index of the target node in the open node list
                parent = [openList(targetNodeLocation,4),openList(targetNodeLocation,5)]; % get the parent node of the target node in the open node list
                
                while ((parent(1) ~= startLocation(1)) || (parent(2) ~= startLocation(2))) % while the parent node is not the start location
                    [robotCount,robotPath] = addToRobotPath(robotCount,robotPath,parent); % add parent node to the robot path
                    nodeLocation = find((openList(:,2) == parent(1)) & (openList(:,3) == parent(2))); % find index of the parent in the open node list
                    parent = [openList(nodeLocation,4),openList(nodeLocation,5)]; % update the parents of the previous parent node
                end
                [robotCount,robotPath] = addToRobotPath(robotCount,robotPath,startLocation); % add the start location to the robot path
                astar.state.ready = 0; % indicate that the optimal path has been found
                
            else % if no path exists
                astar.state.ready = -2; % indicate that there is no solution
            end
            
            function [count,path] = addToRobotPath(count,path,location)     
            % EXAMPLE FUNCTION CALL: Can't run function externally
            % PROGRAMMER: Frederick Wachter
            % DATE CREATED: 2016-06-01
            % PURPOSE: Add element to the soltution path generated from AStar algorithm
            % TYPE: Private Method
                
                count = count + 1; % increment the robot path size counter
                path(count,1:2) = location; % add location to the robot path
                
            end
            
            astar.displayAlgorithmState();
            
            %---------- Place Variables Back Into Structure ----------
            astar.robot.location = robotLocation;
            astar.robot.cost     = robotCost;
            astar.robot.path     = robotPath;
            astar.robot.goal     = robotGoal;
            astar.robot.count    = robotCount;
            astar.map.isPath     = mapIsPath;
            
        end
        
    end
    
    
end


