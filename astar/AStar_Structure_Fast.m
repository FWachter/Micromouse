%ASTAR_STRUCTURE_FAST   Class that allows for the A* algorithm to be run on maps
%   
%   ASTAR_STRUCTURE_FAST
%   initializes the A* class
%
%   ASTAR_STRUCTURE_FAST.runMap(map.data, map.criteria)
%   runs the A* algorithm on a provided map with provided map criteria.
%
%   ASTAR_STRUCTURE_FAST.displayMap
%   loads a figure showing the previously loaded map's walls.
%
%   ASTAR_STRUCTURE_FAST.planner.solution
%   returns the solution path using X and Y coordinates.
%
%   ASTAR_STRUCTURE_FAST.planner.cornerCutPath
%   returns the solution path using X and Y coordinates with all corners
%   cut from the original solution.
%
%   ASTAR_STRUCTURE_FAST.planner.smoothPath
%   returns the solution path using X and Y coordinates with polynomial
%   interpolation of the corner cut path.
%
%   ASTAR_STRUCTURE_FAST.displaySolution
%   displays the solution on the figure. This is shown by default when the
%   runMap() function is run.
%
%   ASTAR_STRUCTURE_FAST.displayCutCornerPath
%   displays the corner cut path on the figure.
%
%   ASTAR_STRUCTURE_FAST.displayOptimizedSolution
%   displays the polynomial interpolated solution on the figure.
%
%   ASTAR_STRUCTURE_FAST.displayDirectionField
%   displays the movements of the robot as a 3D bar graph where the 1
%   represents movement in the positive y direction, 2 for positive x, 3
%   for negative y, and 4 for negative x.
%
%   ASTAR_STRUCTURE_FAST.removeSolution
%   removes the displayed solution path from the figure if it is
%   displaying.
%
%   ASTAR_STRUCTURE_FAST.removeCutCornerPath
%   removes the displayed cut corner path from the figure if it is
%   displaying.
%
%   ASTAR_STRUCTURE_FAST.removeOptimizedSolution
%   removes the displayed plynomial interpolation path from the figure if
%   it is displaying.
%
%   ASTAR_STRUCTURE_FAST.changePathAgression
%   adjust the order of the polynomial interpolation solution. The default
%   is 3. A prompt will display asking for a new order for the polynomial.
%   Enter the desired positive interger for the order.
%
%   Rerun the map to get the new polynomial interpolation path with
%   the desired order of the polynomial.
%
%   See also EXAMPLESCRIPT, MAPGENERATOR, RUNMAPGENERATOR.
%
%   Please refer to the wiki for more information: 
%   https://github.com/FWachter/Micromouse/wiki/MATLAB

%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
% DATE CREATED: 2016-06-01
% PURPOSE: AStar algorithm class
% REFERENCE: http://ch.mathworks.com/matlabcentral/fileexchange/26248-a---a-star--search-for-path-planning-tutorial

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB


classdef AStar_Structure_Fast < handle
    
    properties(SetAccess = protected)
        map
        cost
        robot
        planner
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
            astar.map.legend.solution  = 3;
            
            astar.map.targetLocation   = [0,0];
            astar.map.startLocation    = [0,0];
            astar.map.findCriteria     = 0;
            astar.map.isPath           = 1;
            
            % Figure Properties
            astar.display.obstacleCount           = 0;
            astar.display.handles.obstacles       = [];
            astar.display.figureHandle            = -1;
            astar.display.handles.target          = -1;
            astar.display.handles.robot           = -1;
            astar.display.handles.solutionNodes   = -1;
            astar.display.handles.solutionPath    = -1;
            astar.display.handles.cornerCutPath   = -1;
            astar.display.handles.cornerCutNodes  = -1;
            astar.display.handles.optimalSolution = -1;
            astar.display.handles.directionField  = -1;
            
            % Robot Properties
            astar.robot.location = [0,0];
            astar.robot.path     = [];
            astar.robot.count    = 0;
            astar.robot.cost     = 0;
            astar.robot.goal     = 0;
            
            % Path Planner Properties
            astar.planner.solution       = [];
            astar.planner.cornerCutPath  = [];
            astar.planner.smoothPath     = [];
            astar.planner.directionField = [];
            
            astar.planner.time.cornerCutPath  = 0;
            astar.planner.time.smoothPath     = 0;
            astar.planner.time.directionField = 0;
            
            astar.planner.properties.polynomialOrder = 4;
            
            % Alogrithm State Properties
            astar.state.time            = [];
            astar.state.ready           = -1;
            astar.state.count           = 0;
            astar.state.figureDisplayed = 0;
            astar.state.plannerRun      = 0;
            
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
            
            % Get Initial Map Properties
            astar.map.coordinates  = map;
            astar.map.MAX_X        = size(map,1);
            astar.map.MAX_Y        = size(map,2);
            astar.map.findCriteria = criteria;
            
            % Set Interpolant Value Base on Move Criteria
            if (criteria == 2)
                astar.planner.properties.interpolants = 5;
            else
                astar.planner.properties.interpolants = 3;
            end
            
            % Reset Figure Properties
            astar.display.figureHandle            = -1;
            astar.display.handles.solutionNodes   = -1;
            astar.display.handles.solutionPath    = -1;
            astar.display.handles.cornerCutPath   = -1;
            astar.display.handles.cornerCutNodes  = -1;
            astar.display.handles.optimalSolution = -1;
            astar.display.handles.directionField  = -1;
            
            % Reset Properties
            astar.robot.path   = [];
            astar.robot.count  = 0;
            astar.map.isPath   = 1;
            
            % Reset State Properties
            astar.state.figureDisplayed = 0;
            astar.state.plannerRun      = 0;
            
            % Run master function
            solution = astar.master();
            
        end
        
        function changePathAgression(astar)
        % EXAMPLE FUNCTION CALL: astar.changePathAgression()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-02
        % PURPOSE: Display the map with obstacles, start location, and target location
        % TYPE: Public Method
        
            astar.planner.properties.polynomialOrder = input('New Path Aggression: ');
            if (astar.planner.properties.polynomialOrder > 14)
                error('[Error] Path agression value must be less than 15');
            else
                addCornerCurPath   = 0;
                addOptimalSolution = 0;
                addDirectionField  = 0;
                
                if (ishandle(astar.display.handles.cornerCutPath))
                    astar.removeCutCornerPath();
                    addCornerCurPath = 1;
                end
                if (ishandle(astar.display.handles.optimalSolution))
                    astar.removeOptimizedSolution();
                    addOptimalSolution = 1;
                end
                if (ishandle(astar.display.handles.directionField))
                    delete(astar.display.handles.directionField);
                    addDirectionField = 1;
                end
                
                astar.getPlannerProperties();
                
                if (addCornerCurPath == 1); astar.displayCutCornerPath(); end
                if (addOptimalSolution == 1); astar.displayOptimizedSolution(); end
                if (addDirectionField == 1); astar.displayDirectionField(); end
            end
        
        end
        
        function displayMap(astar)
        % EXAMPLE FUNCTION CALL: astar.displayMap()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Display the map with obstacles, start location, and target location
        % TYPE: Public Method
            
            % Initialize the figure
            astar.display.figureHandle = figure('Name','A* Algorithm','NumberTitle','off'); % initialize figure
            axis([1,astar.map.MAX_X+1,1,astar.map.MAX_Y+1]); % initialize axis spacing
            axis square; grid on; hold on; % set axis properties
            title('Path Palnner Solution');
            astar.state.figureDisplayed = 1;
            
            % Display obstacles, robot, and target location
            for x = 1:astar.map.MAX_X % for all map x locations
                for y = 1:astar.map.MAX_Y % for all map y locations
                    if (astar.map.coordinates(x,y) == astar.map.legend.obstacle) % if the current location is an obstacle
                        astar.display.obstacleCount = astar.display.obstacleCount + 1; % increment obstacle counter
                        astar.display.handles.obstacles(astar.display.obstacleCount) = plot(x+.5,y+.5,'ro'); % show the obstacle
                        displayBorders(astar.map.coordinates,x,y,astar.map.legend.obstacle); % 
                    elseif (astar.map.coordinates(x,y) == astar.map.legend.target) % if the current location is the target
                        astar.display.handles.target = plot(x+.5,y+.5,'gd'); % show the target
                    elseif (astar.map.coordinates(x,y) == astar.map.legend.start) % if the current locaiton is the robot
                        astar.display.handles.robot = plot(x+.5,y+.5,'bo'); % show the robot
                    end
                end
            end
            drawnow; % force MATLAB to draw the figure before continuing
            
            function displayBorders(map,x,y,obstacle)
            % EXAMPLE FUNCTION CALL: displayBorders(astar.map.coordinates,x,y,astar.map.legend.obstacle)
            % PROGRAMMER: Frederick Wachter
            % DATE CREATED: 2016-06-07
            % PURPOSE: Displays lines between neighboring obstacles
            % TYPE: Method within astar.DisplayMap()
                
                if (x ~= 1)
                    if (map(x-1,y) == obstacle)
                        line([x-0.5,x+0.5],[y+0.5,y+0.5],'Color','r');
                    end
                end
                if (y ~= 1)
                    if (map(x,y-1) == obstacle)
                        line([x+0.5,x+0.5],[y-0.5,y+0.5],'Color','r');
                    end
                end
                
            end
            
        end
        
        function displaySolution(astar)
        % EXAMPLE FUNCTION CALL: astar.displaySolution()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Displays the solution from AStar algorithm
        % TYPE: Public Method
            
            % Check if the main figure is displaying
            if (astar.state.figureDisplayed == 0) || ~(ishandle(astar.display.figureHandle))
                astar.display.figureHandle = -1;
                error('Need to run the following before displaying solution: astar.displayMap');
            end
        
            % Check is the solution is already displayed
            if (ishandle(astar.display.handles.solutionNodes))
                delete(astar.display.handles.solutionNodes);
            end
            if (ishandle(astar.display.handles.solutionPath))
                delete(astar.display.handles.solutionPath);
            end
            
            % Plot Nodes and Path to figure
            figure(astar.display.figureHandle);
            astar.display.handles.solutionNodes = plot(astar.robot.path(2:(end-1),1)+.5,astar.robot.path(2:(end-1),2)+.5,'bo'); % display the solution as nodes
            astar.display.handles.solutionPath = plot(astar.robot.path(:,1)+.5,astar.robot.path(:,2)+.5,'b--'); % connect the solution nodes with a dotted line
            drawnow; % force MATLAB to draw the figure before continuing
            
        end
        
        function removeSolution(astar)
        % EXAMPLE FUNCTION CALL: astar.removeSolution()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-02
        % PURPOSE: Removes the solution from the figure
        % TYPE: Public Method
            
            if (astar.state.figureDisplayed == 0)
                error('The solution is note current displayed.');
            end
        
            % Check is the solution is already displayed
            if (ishandle(astar.display.handles.solutionNodes))
                delete(astar.display.handles.solutionNodes);
            end
            if (ishandle(astar.display.handles.solutionPath))
                delete(astar.display.handles.solutionPath);
            end
            
        end
        
        function displayCutCornerPath(astar)
        % EXAMPLE FUNCTION CALL: astar.displayCutCornerPath()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-02
        % PURPOSE: Displays the cut corner path on the figure
        % TYPE: Public Method
        
            if (astar.map.findCriteria == 1)
                % Check if the main figure is displaying
                if (astar.state.figureDisplayed == 0) || ~(ishandle(astar.display.figureHandle))
                    astar.display.figureHandle = -1;
                    error('Need to run the following before displaying solution: astar.displayMap');
                end

                % Check if the planner properties have already been generated
                if (astar.state.plannerRun == 0)
                    astar.getPlannerProperties();
                end

                % Check is the optimized solution is already displayed
                if (ishandle(astar.display.handles.cornerCutPath))
                    delete(astar.display.handles.cornerCutPath);
                end
                if (ishandle(astar.display.handles.cornerCutNodes))
                    delete(astar.display.handles.cornerCutNodes);
                end

                % Plot optimal solution to the figure
                figure(astar.display.figureHandle);
                astar.display.handles.cornerCutNodes = plot(astar.planner.cornerCutPath(2:(end-1),1)+.5,astar.planner.cornerCutPath(2:(end-1),2)+.5,'mo'); % display the solution as nodes
                astar.display.handles.cornerCutPath = plot(astar.planner.cornerCutPath(:,1)+.5,astar.planner.cornerCutPath(:,2)+.5,'m--'); % display the solution as nodes
                drawnow; % force MATLAB to draw the figure before continuing
            else
                fprintf('[Warn] Convex decimator paths are not generated for paths with diagonal movements.\n');
            end
 
        end
        
        function removeCutCornerPath(astar)
        % EXAMPLE FUNCTION CALL: astar.removeCutCornerPath()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-02
        % PURPOSE: Removes the cut corner from the figure
        % TYPE: Public Method
        
            if (astar.map.findCriteria == 1)
                if (astar.state.figureDisplayed == 0)
                    error('The optimal solution is not current displayed.');
                end

                % Check is the solution is already displayed
                if (ishandle(astar.display.handles.cornerCutPath))
                    delete(astar.display.handles.cornerCutPath);
                end
                if (ishandle(astar.display.handles.cornerCutNodes))
                    delete(astar.display.handles.cornerCutNodes);
                end
            else
                fprintf('[Warn] Convex decimator paths are not generated for paths with diagonal movements.\n');
            end
            
        end
        
        function displayOptimizedSolution(astar)
        % EXAMPLE FUNCTION CALL: astar.displayOptimizedSolution()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-02
        % PURPOSE: Displays the optomized solution from AStar algorithm
        % TYPE: Public Method
        
            % Check if the main figure is displaying
            if (astar.state.figureDisplayed == 0) || ~(ishandle(astar.display.figureHandle))
                astar.display.figureHandle = -1;
                error('Need to run the following before displaying solution: astar.displayMap');
            end
            
            % Check if the planner properties have already been generated
            if (astar.state.plannerRun == 0)
                astar.getPlannerProperties();
            end
            
            % Check is the optimized solution is already displayed
            if (ishandle(astar.display.handles.optimalSolution))
                delete(astar.display.handles.optimalSolution);
            end
            
            % Plot optimal solution to the figure
            figure(astar.display.figureHandle);
            astar.display.handles.optimalSolution = plot(astar.planner.smoothPath(:,1)+.5,astar.planner.smoothPath(:,2)+.5,'k'); % display the solution as nodes
            drawnow; % force MATLAB to draw the figure before continuing
 
        end
        
        function removeOptimizedSolution(astar)
        % EXAMPLE FUNCTION CALL: astar.removeOptimizedSolution()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-02
        % PURPOSE: Removes the optimal solution from the figure
        % TYPE: Public Method
            
            if (astar.state.figureDisplayed == 0)
                error('The optimal solution is not current displayed.');
            end
        
            % Check is the solution is already displayed
            if (ishandle(astar.display.handles.optimalSolution))
                delete(astar.display.handles.optimalSolution);
            end
            
        end
        
        function displayDirectionField(astar)
        % EXAMPLE FUNCTION CALL: astar.displayDirectionField()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-02
        % PURPOSE: Displays the direction field bar graph
        % TYPE: Public Method
        
            % Check if the direction field figure is already displayed
            if (ishandle(astar.display.handles.directionField))
                error('Direction field is already showing');
            end
            
            % Check if the planner properties have already been generated
            if (astar.state.plannerRun == 0)
                astar.getPlannerProperties();
            end
            
            % Display the 3D bar plot of direction field
            astar.display.handles.directionField = figure('Name','Direction Field for Solution Path','NumberTitle','off'); % initialize figure
            barHandle = bar3(astar.planner.directionField);
            for bar = 1:length(barHandle)
                zdata = barHandle(bar).ZData;
                barHandle(bar).CData = zdata;
                barHandle(bar).FaceColor = 'interp';
            end
 
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
            
            if (astar.state.ready ~= -2)
                astar.displayMap();
                astar.displaySolution();

                astar.getPlannerProperties();

                solution = astar.robot.path;
            else
                solution = [];
            end
            
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
        
        function getPlannerProperties(astar)
        % EXAMPLE FUNCTION CALL: Can't run function externally
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-02
        % PURPOSE: Gets the cut corner path, polynomial fit path, and the direction field from the solution path
        % TYPE: Private Method  
        
            astar.state.plannerRun = 1; % set flag to indicate that the planner property generator function has been run
                    
%             % Notify user is the map has a obstacle criteria of diagonal
%             if (astar.map.findCriteria == 2)
%                 fprintf('[WARN] Please note that optimal paths for non-orthogonal barriers does not always work\n');
%             end
            
            % Add Solution Map to Planner
            astar.planner.solution = astar.map.coordinates;
            for i = 2:(size(astar.robot.path,1)-1)
                astar.planner.solution(astar.robot.path(i,1),astar.robot.path(i,2)) = astar.map.legend.solution;
            end
        
            % Convert Structured Components to Variables
            path            = astar.robot.path;
            MAX_X           = astar.map.MAX_X;
            MAX_Y           = astar.map.MAX_Y;
            interpolants    = astar.planner.properties.interpolants;
            polynomialOrder = astar.planner.properties.polynomialOrder;
            
            % Run Planners and Path Propery Generator Function
            if (astar.map.findCriteria == 1) % if diagonal movements are not allowed
                [astar.planner.cornerCutPath,astar.planner.time.cornerCutPath] = cutCornerPath(path);
                [astar.planner.smoothPath,astar.planner.time.smoothPath] = getPolynomialPath(astar.planner.cornerCutPath,interpolants,polynomialOrder);
                [astar.planner.directionField,astar.planner.time.directionField] = getDirectionField(path,MAX_X,MAX_Y);
            else % if diagonal movements are allowed
                [astar.planner.smoothPath,astar.planner.time.smoothPath] = getPolynomialPath(path,interpolants,polynomialOrder);
                [astar.planner.directionField,astar.planner.time.directionField] = getDirectionField(path,MAX_X,MAX_Y);
            end
            
            function [newPath,time] = cutCornerPath(path)
            % EXAMPLE FUNCTION CALL: Can't run function externally
            % PROGRAMMER: Frederick Wachter
            % DATE CREATED: 2016-06-02
            % PURPOSE: Generates a solution path with convex decimated corners
            % TYPE: Private Method     
                
                newPath = path;
                circleAdjustment = 1-cosd(45);
                diagonalAdjustment = 0.25;
                postAdjustment = diagonalAdjustment - circleAdjustment;

                tic;
                previousPosition = path(end,:);
                position = path(end-1,:);
                if (previousPosition(1) == position(1))
                    if (previousPosition(2) > position(2))
                        previousDirection = 4;
                    else
                        previousDirection = 2;
                    end
                else
                    if (previousPosition(1) > position(1))
                        previousDirection = 1;
                    else
                        previousDirection = 3;
                    end
                end
                previousPosition = position;
                
                consecutiveTurns = 0;
                for index = (size(path,1)-2):-1:1
                    position = path(index,:);
                    if (previousPosition(1) == position(1))
                        if (previousPosition(2) > position(2))
                            if (previousDirection ~= 4)
                                consecutiveTurns = consecutiveTurns + 1;
                                if (previousDirection == 1)
                                    if (consecutiveTurns < 2)
                                         newPath(index+1,1) = newPath(index+1,1) + circleAdjustment;
                                         newPath(index+1,2) = newPath(index+1,2) - circleAdjustment;
                                    else
                                        if (consecutiveTurns == 2)
                                            newPath(index+2,1) = newPath(index+2,1) - postAdjustment;
                                            newPath(index+2,2) = newPath(index+2,2) + postAdjustment;
                                        end
                                        newPath(index+1,1) = newPath(index+1,1) + diagonalAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) - diagonalAdjustment;
                                    end
                                else
                                    if (consecutiveTurns < 2)
                                        newPath(index+1,1) = newPath(index+1,1) - circleAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) - circleAdjustment;
                                    else
                                        if (consecutiveTurns == 2)
                                            newPath(index+2,1) = newPath(index+2,1) + postAdjustment;
                                            newPath(index+2,2) = newPath(index+2,2) + postAdjustment;
                                        end
                                        newPath(index+1,1) = newPath(index+1,1) - diagonalAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) - diagonalAdjustment;
                                    end
                                end
                                previousDirection = 4;
                            else
                                consecutiveTurns = 0;
                            end    
                        else
                            if (previousDirection ~= 2)
                                consecutiveTurns = consecutiveTurns + 1;
                                if (previousDirection == 1)
                                    if (consecutiveTurns < 2)
                                        newPath(index+1,1) = newPath(index+1,1) + circleAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) + circleAdjustment;
                                    else
                                        if (consecutiveTurns == 2)
                                            newPath(index+2,1) = newPath(index+2,1) - postAdjustment;
                                            newPath(index+2,2) = newPath(index+2,2) - postAdjustment;
                                        end
                                        newPath(index+1,1) = newPath(index+1,1) + diagonalAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) + diagonalAdjustment;
                                    end
                                else
                                    if (consecutiveTurns < 2)
                                        newPath(index+1,1) = newPath(index+1,1) - circleAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) + circleAdjustment;
                                    else
                                        if (consecutiveTurns == 2)
                                            newPath(index+2,1) = newPath(index+2,1) + postAdjustment;
                                            newPath(index+2,2) = newPath(index+2,2) - postAdjustment;
                                        end
                                        newPath(index+1,1) = newPath(index+1,1) - diagonalAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) + diagonalAdjustment;
                                    end
                                end
                                previousDirection = 2;
                            else
                                consecutiveTurns = 0;
                            end   
                        end
                    else
                        if (previousPosition(1) > position(1))
                            if (previousDirection ~= 1)
                                consecutiveTurns = consecutiveTurns + 1;
                                if (previousDirection == 2)
                                    if (consecutiveTurns < 2)
                                        newPath(index+1,1) = newPath(index+1,1) - circleAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) - circleAdjustment;
                                    else
                                        if (consecutiveTurns == 2)
                                            newPath(index+2,1) = newPath(index+2,1) + postAdjustment;
                                            newPath(index+2,2) = newPath(index+2,2) + postAdjustment;
                                        end
                                        newPath(index+1,1) = newPath(index+1,1) - diagonalAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) - diagonalAdjustment;
                                    end
                                else
                                    if (consecutiveTurns < 2)
                                        newPath(index+1,1) = newPath(index+1,1) - circleAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) + circleAdjustment;
                                    else
                                        if (consecutiveTurns == 2)
                                            newPath(index+2,1) = newPath(index+2,1) + postAdjustment;
                                            newPath(index+2,2) = newPath(index+2,2) - postAdjustment;
                                        end
                                        newPath(index+1,1) = newPath(index+1,1) - diagonalAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) + diagonalAdjustment;
                                    end
                                end
                                previousDirection = 1;
                            else
                                consecutiveTurns = 0;
                            end   
                        else
                            if (previousDirection ~= 3)
                                consecutiveTurns = consecutiveTurns + 1;
                                if (previousDirection == 2)
                                    if (consecutiveTurns < 2)
                                        newPath(index+1,1) = newPath(index+1,1) + circleAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) - circleAdjustment;
                                    else
                                        if (consecutiveTurns == 2)
                                            newPath(index+2,1) = newPath(index+2,1) - postAdjustment;
                                            newPath(index+2,2) = newPath(index+2,2) + postAdjustment;
                                        end
                                        newPath(index+1,1) = newPath(index+1,1) + diagonalAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) - diagonalAdjustment;
                                    end
                                else
                                    if (consecutiveTurns < 2)
                                        newPath(index+1,1) = newPath(index+1,1) + circleAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) + circleAdjustment;
                                    else
                                        if (consecutiveTurns == 2)
                                            newPath(index+2,1) = newPath(index+2,1) - postAdjustment;
                                            newPath(index+2,2) = newPath(index+2,2) - postAdjustment;
                                        end
                                        newPath(index+1,1) = newPath(index+1,1) + diagonalAdjustment;
                                        newPath(index+1,2) = newPath(index+1,2) + diagonalAdjustment;
                                    end
                                end
                                previousDirection = 3;
                            else
                                consecutiveTurns = 0;
                            end   
                        end
                    end

                    previousPosition = position;
                end
                time = toc;
                
            end
            
            function [smoothPath,time] = getPolynomialPath(path,interpolants,polynomialOrder)
            % EXAMPLE FUNCTION CALL: Can't run function externally
            % PROGRAMMER: Frederick Wachter
            % DATE CREATED: 2016-06-02
            % PURPOSE: Generates a polynomail fitted solution with desired agression
            % TYPE: Private Method     
                
                x = path(:,1);
                y = path(:,2);

                tic;
                interpolateX = linspace(x(end),x(end-1),interpolants);
                interpolateY = linspace(y(end),y(end-1),interpolants);
                
                for index = (length(x)-2):-1:1
                    interpolateX = [interpolateX(1:end-1),linspace(x(index+1),x(index),interpolants)];
                    interpolateY = [interpolateY(1:end-1),linspace(y(index+1),y(index),interpolants)];
                end
                
                windowWidth = 15;
                smoothX = sgolayfilt(interpolateX, polynomialOrder, windowWidth);
                smoothY = sgolayfilt(interpolateY, polynomialOrder, windowWidth);
                smoothPath = [smoothX',smoothY'];
                time = toc;
                
            end
            
            function [directionField,time] = getDirectionField(path,MAX_X,MAX_Y)
            % EXAMPLE FUNCTION CALL: Can't run function externally
            % PROGRAMMER: Frederick Wachter
            % DATE CREATED: 2016-06-02
            % PURPOSE: Generates the direction field from the soltion path
            % TYPE: Private Method 
               
                directionField = zeros(MAX_X,MAX_Y);
                
                tic;
                for index = (size(path,1)-1):-1:1
                    previousPosition = path(index+1,:);
                    position = path(index,:);

                    if (previousPosition(1) == position(1))
                        if (previousPosition(2) > position(2))
                            directionField(previousPosition(1),previousPosition(2)) = 4;
                        else
                            directionField(previousPosition(1),previousPosition(2)) = 2;
                        end
                    else
                        if (previousPosition(1) > position(1))
                            directionField(previousPosition(1),previousPosition(2)) = 1;
                        else
                            directionField(previousPosition(1),previousPosition(2)) = 3;
                        end
                    end
                end
                time = toc;
                
            end
            
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
                fprintf('\n'); % go to a new line
                error('Failure to initialize algorithm'); % notify user that the algorithm was not set up properly
            elseif (astar.state.ready == -2) % algorithm could not find a solution
                fprintf('No solution exists\n'); % go to a new line
                error('[Error] No solution exists for given map'); % notify user that no solution could be found for the given map
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
                        
            %---------- Convert Structured Components to Variables ----------
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
                                    
            % Set all of the objects as closed nodes
            for x = 1:MAX_X
                for y = 1:MAX_Y
                    if (astar.map.coordinates(x,y) == astar.map.legend.obstacle)
                        closedCount = closedCount + 1;
                        closedList(closedCount,1:2) = [x,y];
                    end
                end
            end    
            
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


