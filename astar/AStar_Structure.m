%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter
% DATE CREATED: 2016-04-27
% PURPOSE: AStar algorithm class
% REFERENCE: http://ch.mathworks.com/matlabcentral/fileexchange/26248-a---a-star--search-for-path-planning-tutorial
% CONTACT INFO: wachterfreddy@gmail.com

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: www.github.com/FWchter/Micromouse/Wiki


%% Things to Add
% ==============================================
% Compare with script version to determine speed
% ==============================================

% ===================================
% Allow user to define the map legend
% ===================================


classdef AStar_Structure < handle
    
    properties(SetAccess = protected)
        map
        display
        robot
        open
        closed
        expand
        optimal
        state
    end
   
    
%% CONSTRUCTOR METHOD
    methods
        
        function astar = AStar_Structure(varargin)
            
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
            
            % Open Node List Properties
            astar.open.list  = [];
            astar.open.count = 0;
            
            % Closed Node List Properties
            astar.closed.list  = [];
            astar.closed.count = 0;
            
            % Expand Node List Properties
            astar.expand.list  = [];
            astar.expand.count = 0;
            
            % Optimal Node List Properties
            astar.optimal.list  = [];
            astar.optimal.count = 0;
            
            % Alogrithm State Properties
            astar.state.ready = -1;
            astar.state.time  = [];
            astar.state.count = 0;
            
        end
        
    end
    
    
%% PUBLIC METHODS

    methods
        
        function solution = runMap(astar,map,criteria)
            
            % Get initial map properties
            astar.map.coordinates  = map;
            astar.map.MAX_X        = size(map,1);
            astar.map.MAX_Y        = size(map,2);
            astar.map.findCriteria = criteria;
            
            % Reset Properties
            astar.robot.path   = [];
            astar.open.list    = [];
            astar.closed.list  = [];
            astar.robot.count  = 0;
            astar.open.count   = 0;
            astar.closed.count = 0;
            astar.map.isPath   = 1;
            
            % Run master function
            solution = astar.master();
            
        end
        
    end


%% PRIVATE METHODS

    methods(Access = private)
        
        % Pre-Processor Functions
        function solution = master(astar)
            
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
            
            % Get start and target locations
            [xStartLocation,yStartLocation] = find(astar.map.coordinates == astar.map.legend.start);
            [xTargetLocation,yTargetLocation] = find(astar.map.coordinates == astar.map.legend.target);
            
            % Store start, target, and initial robot location into astar object
            astar.map.startLocation = [xStartLocation,yStartLocation];
            astar.map.targetLocation = [xTargetLocation,yTargetLocation];
            astar.robot.location = astar.map.startLocation;

        end
        
        function initializeVariables(astar)
            
            % Set all of the objects as closed nodes
            for x = 1:astar.map.MAX_X
                for y = 1:astar.map.MAX_Y
                    if (astar.map.coordinates(x,y) == astar.map.legend.obstacle)
                        astar.closed.count = astar.closed.count + 1;
                        astar.closed.list(astar.closed.count,1:2) = [x,y];
                    end
                end
            end
            
            % Set robot cost and goal values
            astar.robot.cost = 0;
            astar.robot.goal = astar.cost(astar.robot.location,astar.map.targetLocation);
            
            % Add robot start location to open node list and set as parent node
            astar.addOpenNode(astar.robot.location,astar.robot.location,[astar.robot.cost,astar.robot.goal,astar.robot.goal]); % ??? will this always only be robot variables
            astar.open.list(astar.open.count,1) = 0;
            
            % Add robot start location to the closed node list
            astar.addClosedNode(astar.robot.location)
            
            % Get user defined criteria
            switch astar.map.findCriteria
                case 1 % check only orthoginal nodes
                    astar.map.criteria = @(x,y)(((x == 0) && (y ~= 0)) || ((y == 0) && (x ~= 0)));
                case 2 % check orthogonal and diagonal nodes
                    astar.map.criteria = @(x,y)((x ~= y) || (x ~= 0));
            end
            
        end

        % Algorithm Functions
        function startAlgorithm(astar)
            
            astar.displayAlgorithmState();
            
            while (((astar.robot.location(1) ~= astar.map.targetLocation(1)) || (astar.robot.location(2) ~= astar.map.targetLocation(2))) && astar.map.isPath == 1)
                astar.getPotentialNodes();
                astar.checkPotentialNodes();
                astar.getNextNode();
            end
            astar.getOptimalPath();
            
            astar.displayAlgorithmState();
            
        end
        
        function displayAlgorithmState(astar)
            
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
        
        function getPotentialNodes(astar)
            
            % Reset expand list variables
            astar.expand.count = 0;
            astar.expand.list = [];
            
            for x = [1,0,-1] % iterate through all x neightbors
                for y = [1,0,-1] % iterate through all y neighbors
                    if astar.map.criteria(x,y) % check if the node fits the desired node location criteria
                        testPosition = [astar.robot.location(1)+x,astar.robot.location(2)+y]; % store the node location
                        if (((testPosition(1) > 0) && (testPosition(1) <= astar.map.MAX_X)) && ((testPosition(2) > 0) && (testPosition(2) <= astar.map.MAX_Y))) % check if the node is in the bounds of the map
                            isOpen = 1; % set this location as a possible new node
                            for closedNode = 1:astar.closed.count % seach through all of the closed nodes
                                if (testPosition(1) == astar.closed.list(closedNode,1) && (testPosition(2) == astar.closed.list(closedNode,2))) % if the possible new node has already been transversed
                                    isOpen = 0; % this location has already been transversed and do not add to the expanded node list
                                end
                            end
                            if (isOpen == 1) % if the node fits the desired location criteria, within the bounds of the map, and not already trasnversed
                                astar.expand.count = astar.expand.count + 1; % increment the expanded node list count
                                astar.expand.list(astar.expand.count,1) = testPosition(1); % store the x location of the new node
                                astar.expand.list(astar.expand.count,2) = testPosition(2); % store the y location of the new node
                                astar.expand.list(astar.expand.count,3) = astar.robot.cost + astar.cost(astar.robot.location,testPosition); % add the cost for the new node from the original robot position
                                astar.expand.list(astar.expand.count,4) = astar.cost(astar.map.targetLocation,testPosition); % store the goal distance for the new node
                                astar.expand.list(astar.expand.count,5) = astar.expand.list(astar.expand.count,3) + astar.expand.list(astar.expand.count,4); % store the fitness of the new node
                            end
                        end
                    end
                end
            end
            
        end
        
        function checkPotentialNodes(astar)
           
            for expandedNode = 1:astar.expand.count
                nodeExists = 0; % flag variable used to indicate if the new node already exists in the open list 
                for openNode = 1:astar.open.count
                    if ((astar.expand.list(expandedNode,1) == astar.open.list(openNode,2)) && (astar.expand.list(expandedNode,2) == astar.open.list(openNode,3)))
                        astar.open.list(openNode,8) = min(astar.open.list(openNode,8),astar.expand.list(expandedNode,5)); % compare the fitness of the node already in the open list with the new node
                        if (astar.open.list(openNode,8) == astar.expand.list(expandedNode,5)) % if the new node has a smaller fitness
                            astar.open.list(openNode,4) = astar.robot.location(1); % change the x location of the parent node to the parent of the new node
                            astar.open.list(openNode,5) = astar.robot.location(2); % change the y location of the parent node to the parent of the new node
                            astar.open.list(openNode,6) = astar.expand.list(expandedNode,3); % change the cost to the cost of the new node
                            astar.open.list(openNode,7) = astar.expand.list(expandedNode,4); % change the goal distance to the distance of the new node
                        end
                        nodeExists = 1; % node already exists as an open node
                    end
                end
                if (nodeExists == 0) % if the node is not already in the open list
                    astar.addOpenNode(astar.expand.list(expandedNode,1:2),astar.robot.location,astar.expand.list(expandedNode,3:5));
                end
            end
            
        end
        
        function getNextNode(astar)
            
            % Reset the optimal node list
            astar.optimal.count = 0;
            astar.optimal.list = [];
            
            % Find potential optimal nodes
            foundTarget = 0; % flag to indicate if an optimal node is at the target location
            for openNode = 1:astar.open.count % for all nodes in the open list
                if (astar.open.list(openNode,1) == 1) % if the current node is not a parent of another node
                    astar.optimal.count = astar.optimal.count + 1; % increment count for nodes in the optimal node list
                    astar.optimal.list(astar.optimal.count,:) = [astar.open.list(openNode,8),openNode]; % add node to optimal node list
                    if ((astar.open.list(openNode,2) == astar.map.targetLocation(1)) && (astar.open.list(openNode,3) == astar.map.targetLocation(2))) % if the new optimal node is at the target location
                        foundTarget = 1; % indicate we found an optimal node at the target location
                        break; % break from for loop since optimal node has been found
                    end
                end
            end
            
            % Stre location of the optimal node
            if (foundTarget == 1) % if the optimal node is at the target location
                optimalNode = openNode; % store the index of the optimal node referencing the open node list
            elseif (size(astar.optimal.list ~= 0)) % if there are optimal nodes in the optimal node list
                [~,optimalNodeIndex] = min(astar.optimal.list(:,1)); % find the node with the minimum fitness
                optimalNode = astar.optimal.list(optimalNodeIndex,2); % store the index of the optimal node referencing the open node list
            else % if no more paths are available
                optimalNode = -1; % indicate that no paths are available
            end
            
            % Update robot position for next iterations
            if (optimalNode ~= -1) % if an optimal node exits
                astar.robot.location = [astar.open.list(optimalNode,2),astar.open.list(optimalNode,3)]; % update the robot location
                astar.robot.cost = astar.open.list(optimalNode,6); % update the cost of the robot location
                astar.addClosedNode(astar.robot.location); % add the robot location to the closed node list
                astar.open.list(optimalNode,1) = 0; % set the robot location as a parent node
            else % if there are no nodes left
                astar.map.isPath = 0; % indicate that there are not paths left
            end
            
        end
        
        function getOptimalPath(astar)
            
            [astar.robot.count,astar.robot.path] = addToRobotPath(astar.robot.count,astar.robot.path,astar.closed.list(end,:)); % store the last node in the closed node location into the robot path
            if ((astar.robot.path(1,1) == astar.map.targetLocation(1)) && (astar.robot.path(1,2) == astar.map.targetLocation(2))) % if the last node in the closed node list is the target location
                
                targetNodeLocation = find((astar.open.list(:,2) == astar.robot.path(1,1)) & (astar.open.list(:,3) == astar.robot.path(1,2))); % find the index of the target node in the open node list
                parent = [astar.open.list(targetNodeLocation,4),astar.open.list(targetNodeLocation,5)]; % get the parent node of the target node in the open node list
                
                while ((parent(1) ~= astar.map.startLocation(1)) || (parent(2) ~= astar.map.startLocation(2))) % while the parent node is not the start location
                    [astar.robot.count,astar.robot.path] = addToRobotPath(astar.robot.count,astar.robot.path,parent); % add parent node to the robot path
                    nodeLocation = find((astar.open.list(:,2) == parent(1)) & (astar.open.list(:,3) == parent(2))); % find index of the parent in the open node list
                    parent = [astar.open.list(nodeLocation,4),astar.open.list(nodeLocation,5)]; % update the parents of the previous parent node
                end
                [astar.robot.count,astar.robot.path] = addToRobotPath(astar.robot.count,astar.robot.path,astar.map.startLocation); % add the start location to the robot path
                astar.state.ready = 0; % indicate that the optimal path has been found
                
            else % if no path exists
                astar.state.ready = -2; % indicate that there is no solution
            end
            
            function [count,path] = addToRobotPath(count,path,location)
                
                count = count + 1; % increment the robot path size counter
                path(count,1:2) = location; % add location to the robot path
                
            end
            
        end
        
        function addOpenNode(astar,location,parent,fitnessParameters)
           
            astar.open.count = astar.open.count + 1; % increment the open node node list size counter
            astar.open.list(astar.open.count,:) = [1,location,parent,fitnessParameters]; % add parameters to the open node list
            
        end
        
        function addClosedNode(astar,location)
            
            astar.closed.count = astar.closed.count + 1; % incremenet the closed node list size counter
            astar.closed.list(astar.closed.count,1:2) = location; % add location to the closed node list
            
        end
        
        % Other Functions
        function displayMap(astar)
            
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
            
            astar.display.handles.solutionNodes = plot(astar.robot.path(:,1)+.5,astar.robot.path(:,2)+.5,'bo'); % display the solution as nodes
            astar.display.handles.solutionPath = plot(astar.robot.path(:,1)+.5,astar.robot.path(:,2)+.5,'b--'); % connect the solution nodes with a dotted line
            drawnow; % force MATLAB to draw the figure before continuing
            
        end
        
%         function displayTimeStatistics(astar) 
%            
%             Function has not been made yet
%             
%         end
        
    end
    
    
%% STATIC METHODS
    methods(Static = true, Access = private)

        function nodeCost = cost(location,target)
           
            nodeCost = sqrt((target(1)-location(1))^2 + (target(2)-location(2))^2);
            
        end
        
    end
    
    
end


