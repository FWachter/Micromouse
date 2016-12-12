%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
% DATE CREATED: 2016-12-10
% PURPOSE: Micromouse simulator class

% Please refer to the Wiki for instructions on how to use this class
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB


classdef simulator < handle
    
    properties
        
    end
    
    properties(SetAccess = protected)
        map
        robot
        sensor
        display
    end

% pull map, define environment
% move robot


%% CONSTRUCTOR METHOD
    methods
        
        function micromouse = simulator(varargin)
        % EXAMPLE FUNCTION CALL: micromouse = simulator(varargin)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-11
        % PURPOSE: Construct the structure
        % TYPE: Constructor Method
            
            % Map Properties
            micromouse.map.coordinates = [];
            micromouse.map.maxX        = 0;
            micromouse.map.maxY        = 0;
            
            micromouse.map.targetLocation   = [0,0];
            micromouse.map.startLocation    = [0,0];
            
            % Figure Properties
            micromouse.display.obstacleCount      = 0;
            micromouse.display.figureHandle       = -1;
            micromouse.display.handles.fullMap    = -1;
            micromouse.display.handles.robot      = -1;
            micromouse.display.displayedObstacles = [];
            micromouse.display.displayedLines     = [];
            
            % Robot Properties
            micromouse.robot.location  = [0,0];
            micromouse.robot.direction = 1; % N: 1, E: 2, S: 3, W: 4
            
            % Sensor Properties
            micromouse.sensor.lineOfSight = 2;
            
        end
        
    end
    
    
%% PUBLIC METHODS

    methods
        
        function getMap(micromouse, map)
        % EXAMPLE FUNCTION CALL: getMap(micromouse, map)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-11
        % PURPOSE: Take user varables and run master script for AStar
        % INPUTS:      map - 2D array containing obstacles, target, and start location
            
            % Get Initial Map Properties
            micromouse.map.coordinates = map.data;
            micromouse.map.legend      = map.legend;
            micromouse.map.maxX        = size(map.data, 1);
            micromouse.map.maxY        = size(map.data, 2);
            
            % Reset Figure Properties
            micromouse.display.figureHandle        = -1;
            micromouse.display.figureHandleFullMap = -1;
            
            % Display Figure
            micromouse.displayMap();
            micromouse.displayFullMap();
            
            % Get Initial Parameters
            micromouse.getRobotOrientation();
            micromouse.updateLineOfSight();
            
            micromouse.exampleMovement();

        end
        
        function moveRobot(micromouse, x, y)
            
            figure(micromouse.display.figureHandle);
            set(micromouse.display.handles.robot, 'XData', x, 'YData', y);
            micromouse.robot.location = [x, y];
            drawnow;
            
            micromouse.collisionDetection();
            
        end
        
    end
    

%% PRIVATE METHODS

    methods (Access = private)
        
        % Robot Function
        
        function getRobotOrientation(micromouse)
        % EXAMPLE FUNCTION CALL: micromouse.getRobotOrientation()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-10-11
        % PURPOSE: Get initial orientation of the robot
        
            if (micromouse.map.coordinates(micromouse.robot.location(1), micromouse.robot.location(2)+1) == micromouse.map.legend.freeSpace)
                micromouse.robot.direction = 1;
            elseif (micromouse.map.coordinates(micromouse.robot.location(1)+1, micromouse.robot.location(2)+1) == micromouse.map.legend.freeSpace)
                micromouse.robot.direction = 2;
            elseif (micromouse.map.coordinates(micromouse.robot.location(1), micromouse.robot.location(2)-1) == micromouse.map.legend.freeSpace)
                micromouse.robot.direction = 3;
            else
                micromouse.robot.direction = 4;
            end
            
            obstacles = getAssumedBoundaries(micromouse.map, micromouse.robot.location(1), micromouse.robot.location(2), micromouse.robot.direction);
            micromouse.addObstacles(obstacles);
            
            figure(micromouse.display.figureHandle);
            displayedLines = displayLines(micromouse.map, obstacles);
            micromouse.display.displayedLines = [micromouse.display.displayedLines; displayedLines];
            
            function obstacles = getAssumedBoundaries(map, robotX, robotY, robotDirection)
            % EXAMPLE FUNCTION CALL: micromouse.getRobotOrientation()
            % PROGRAMMER: Frederick Wachter
            % DATE CREATED: 2016-10-11
            % PURPOSE: Get initial borders of robot at robot placement
               
                possibleObstacleX = [];
                possibleObstacleY = [];
                
                if (robotDirection == 1)
                    if ((robotX > 1) && (robotX < map.maxX))
                        if (robotY > 1)
                            possibleObstacleX = [robotX-1, robotX-1, robotX, robotX+1, robotX+1];
                            possibleObstacleY = [robotY, robotY-1, robotY-1, robotY-1, robotY];
                        else
                            possibleObstacleX = [robotX-1, robotX+1];
                            possibleObstacleY = [robotY, robotY];
                        end
                    elseif (robotX > 1)
                        if (robotY > 1)
                            possibleObstacleX = [robotX-1, robotX-1, robotX];
                            possibleObstacleY = [robotY, robotY-1, robotY-1];
                        else
                            possibleObstacleX = [robotX-1];
                            possibleObstacleY = [robotY];
                        end
                    elseif (robotX < map.maxX)
                        if (robotY > 1)
                            possibleObstacleX = [robotX, robotX+1, robotX+1];
                            possibleObstacleY = [robotY-1, robotY-1, robotY];
                        else
                            possibleObstacleX = [robotX+1];
                            possibleObstacleY = [robotY];
                        end
                    end
                elseif (robotDirection == 2)
                    if ((robotY > 1) && (robotY < map.maxY))
                        if (robotX > 1)
                            possibleObstacleX = [robotX, robotX-1, robotX-1, robotX-1, robotX];
                            possibleObstacleY = [robotY+1, robotY+1, robotY, robotY-1, robotY-1];
                        else
                            possibleObstacleX = [robotX, robotX];
                            possibleObstacleY = [robotY+1, robotY-1];
                        end
                    elseif (robotY > 1)
                        if (robotX > 1)
                            possibleObstacleX = [robotX-1, robotX-1, robotX];
                            possibleObstacleY = [robotY, robotY-1, robotY-1];
                        else
                            possibleObstacleX = [robotX];
                            possibleObstacleY = [robotY-1];
                        end
                    elseif (robotY < map.maxY)
                        if (robotX > 1)
                            possibleObstacleX = [robotX, robotX-1, robotX-1];
                            possibleObstacleY = [robotY+1, robotY+1, robotY];
                        else
                            possibleObstacleX = [robotX];
                            possibleObstacleY = [robotY+1];
                        end
                    end
                elseif (robotDirection == 3)
                    if ((robotX > 1) && (robotX < map.maxX))
                        if (robotY < map.maxY)
                            possibleObstacleX = [robotX-1, robotX-1, robotX, robotX+1, robotX+1];
                            possibleObstacleY = [robotY, robotY+1, robotY+1, robotY+1, robotY];
                        else
                            possibleObstacleX = [robotX-1, robotX+1];
                            possibleObstacleY = [robotY, robotY];
                        end
                    elseif (robotX > 1)
                        if (robotY < map.maxY)
                            possibleObstacleX = [robotX-1, robotX-1, robotX];
                            possibleObstacleY = [robotY, robotY+1, robotY+1];
                        else
                            possibleObstacleX = [robotX-1];
                            possibleObstacleY = [robotY];
                        end
                    elseif (robotX < map.maxX)
                        if (robotY < map.maxY)
                            possibleObstacleX = [robotX, robotX+1, robotX+1];
                            possibleObstacleY = [robotY+1, robotY+1, robotY];
                        else
                            possibleObstacleX = [robotX+1];
                            possibleObstacleY = [robotY];
                        end
                    end
                else
                    if ((robotY > 1) && (robotY < map.maxY))
                        if (robotX < map.maxX)
                            possibleObstacleX = [robotX, robotX+1, robotX+1, robotX+1, robotX];
                            possibleObstacleY = [robotY+1, robotY+1, robotY, robotY-1, robotY-1];
                        else
                            possibleObstacleX = [robotX, robotX];
                            possibleObstacleY = [robotY+1, robotY-1];
                        end
                    elseif (robotY > 1)
                        if (robotX < map.maxX)
                            possibleObstacleX = [robotX+1, robotX+1, robotX];
                            possibleObstacleY = [robotY, robotY-1, robotY-1];
                        else
                            possibleObstacleX = [robotX];
                            possibleObstacleY = [robotY-1];
                        end
                    elseif (robotY < map.maxY)
                        if (robotX < map.maxX)
                            possibleObstacleX = [robotX, robotX+1, robotX+1];
                            possibleObstacleY = [robotY+1, robotY+1, robotY];
                        else
                            possibleObstacleX = [robotX];
                            possibleObstacleY = [robotY+1];
                        end
                    end
                end
                
                obstacles = [possibleObstacleX', possibleObstacleY'];
                
            end
            
            function displayedLines = displayLines(map, obstacles)

                displayedLines = [];
                for obstacle = 1:size(obstacles, 1)
                    if (map.coordinates(obstacles(obstacle, 1), obstacles(obstacle, 2)) == map.legend.obstacle)
                        for comparisonObstacle = 1:size(obstacles, 1)
                            if (map.coordinates(obstacles(comparisonObstacle, 1), obstacles(comparisonObstacle, 2)) == map.legend.obstacle)
                                if (obstacle ~= comparisonObstacle)
                                    if (sqrt((obstacles(obstacle, 1) - obstacles(comparisonObstacle, 1))^2 + (obstacles(obstacle, 2) - obstacles(comparisonObstacle, 2))^2) == 1)
                                        line([obstacles(obstacle, 1), obstacles(comparisonObstacle, 1)], [obstacles(obstacle, 2), obstacles(comparisonObstacle, 2)], 'Color', 'r');
                                        displayedLines = [displayedLines; (obstacles(obstacle, 1)+obstacles(comparisonObstacle, 1))/2, (obstacles(obstacle, 2)-obstacles(comparisonObstacle, 2))/2];
                                        break;
                                    end
                                end
                            end
                        end
                    end
                end
                drawnow;

            end
            
        end
        
        function obstacles = getLineOfSight(micromouse)
        % EXAMPLE FUNCTION CALL: updateLineOfSight()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-10-11
        % PURPOSE: Gets the list of obstacles the robot can see from its position
            
            % Get Line of Sight Obstacles
            possibleObstacleX = [];
            possibleObstacleY = [];
            robotX = micromouse.robot.location(1);
            robotY = micromouse.robot.location(2);
            
            if (micromouse.robot.direction == 1)
                if ((robotX > 1) && (robotX < micromouse.map.maxX))
                    if ((robotY < micromouse.map.maxY - 1) && (micromouse.map.coordinates(robotX, robotY+1) ~= micromouse.map.legend.obstacle))
                        possibleObstacleX = [robotX-1, robotX-1, robotX, robotX, robotX+1, robotX+1];
                        possibleObstacleY = [robotY, robotY+1, robotY+1, robotY+2, robotY, robotY+1];
                    elseif (robotY < micromouse.map.maxY)
                        possibleObstacleX = [robotX-1, robotX-1, robotX, robotX+1, robotX+1];
                        possibleObstacleY = [robotY, robotY+1, robotY+1, robotY, robotY+1];
                    else
                        possibleObstacleX = [robotX-1, robotX+1];
                        possibleObstacleY = [robotY, robotY];
                    end
                elseif (robotX == 1)
                    if ((robotY < micromouse.map.maxY - 1) && (micromouse.map.coordinates(robotX, robotY+1) ~= micromouse.map.legend.obstacle))
                        possibleObstacleX = [robotX, robotX, robotX+1, robotX+1];
                        possibleObstacleY = [robotY+1, robotY+2, robotY, robotY+1];
                    elseif (robotY < micromouse.map.maxY)
                        possibleObstacleX = [robotX, robotX+1, robotX+1];
                        possibleObstacleY = [robotY+1, robotY, robotY+1];
                    else
                        possibleObstacleX = [robotX+1];
                        possibleObstacleY = [robotY];
                    end
                elseif (robotX == micromouse.map.maxX)
                    if ((robotY < micromouse.map.maxY - 1) && (micromouse.map.coordinates(robotX, robotY+1) ~= micromouse.map.legend.obstacle))
                        possibleObstacleX = [robotX-1, robotX-1, robotX, robotX];
                        possibleObstacleY = [robotY, robotY+1, robotY+1, robotY+2];
                    elseif (robotY < micromouse.map.maxY)
                        possibleObstacleX = [robotX-1, robotX-1, robotX];
                        possibleObstacleY = [robotY, robotY+1, robotY+1];
                    else
                        possibleObstacleX = [robotX-1];
                        possibleObstacleY = [robotY];
                    end
                end
            elseif (micromouse.robot.direction == 2)
                if ((robotY > 1) && (robotY < micromouse.map.maxY))
                    if ((robotX < micromouse.map.maxX - 1) && (micromouse.map.coordinates(robotX+1, robotY) ~= micromouse.map.legend.obstacle))
                        possibleObstacleX = [robotX, robotX+1, robotX+1, robotX+2, robotX, robotX+1];
                        possibleObstacleY = [robotY+1, robotY+1, robotY, robotY, robotY-1, robotY-1];
                    elseif (robotX < micromouse.map.maxX)
                        possibleObstacleX = [robotX, robotX+1, robotX+1, robotX, robotX+1];
                        possibleObstacleY = [robotY+1, robotY+1, robotY, robotY-1, robotY-1];
                    else
                        possibleObstacleX = [robotX, robotX];
                        possibleObstacleY = [robotY+1, robotY-1];
                    end
                elseif (robotY == 1)
                    if ((robotX < micromouse.map.maxX - 1) && (micromouse.map.coordinates(robotX+1, robotY) ~= micromouse.map.legend.obstacle))
                        possibleObstacleX = [robotX, robotX+1, robotX+1, robotX+2];
                        possibleObstacleY = [robotY+1, robotY+1, robotY, robotY];
                    elseif (robotX < micromouse.map.maxX)
                        possibleObstacleX = [robotX, robotX+1, robotX+1];
                        possibleObstacleY = [robotY+1, robotY+1, robotY];
                    else
                        possibleObstacleX = [robotX];
                        possibleObstacleY = [robotY+1];
                    end
                elseif (robotY == micromouse.map.maxY)
                    if ((robotX < micromouse.map.maxX - 1) && (micromouse.map.coordinates(robotX+1, robotY) ~= micromouse.map.legend.obstacle))
                        possibleObstacleX = [robotX+1, robotX+2, robotX, robotX+1];
                        possibleObstacleY = [robotY, robotY, robotY-1, robotY-1];
                    elseif (robotX < micromouse.map.maxX)
                        possibleObstacleX = [robotX+1, robotX, robotX+1];
                        possibleObstacleY = [robotY, robotY-1, robotY-1];
                    else
                        possibleObstacleX = [robotX];
                        possibleObstacleY = [robotY-1];
                    end
                end
            elseif (micromouse.robot.direction == 3)
                if ((robotX > 1) && (robotX < micromouse.map.maxX))
                    if ((robotY > 2) && (micromouse.map.coordinates(robotX, robotY-1) ~= micromouse.map.legend.obstacle))
                        possibleObstacleX = [robotX-1, robotX-1, robotX, robotX, robotX+1, robotX+1];
                        possibleObstacleY = [robotY, robotY-1, robotY-1, robotY-2, robotY, robotY-1];
                    elseif (robotY > 1)
                        possibleObstacleX = [robotX-1, robotX-1, robotX, robotX+1, robotX+1];
                        possibleObstacleY = [robotY, robotY-1, robotY-1, robotY, robotY-1];
                    else
                        possibleObstacleX = [robotX-1, robotX+1];
                        possibleObstacleY = [robotY, robotY];
                    end
                elseif (robotX == 1)
                    if ((robotY > 2) && (micromouse.map.coordinates(robotX, robotY-1) ~= micromouse.map.legend.obstacle))
                        possibleObstacleX = [robotX, robotX, robotX+1, robotX+1];
                        possibleObstacleY = [robotY-1, robotY-2, robotY, robotY-1];
                    elseif (robotY > 1)
                        possibleObstacleX = [robotX, robotX+1, robotX+1];
                        possibleObstacleY = [robotY-1, robotY, robotY-1];
                    else
                        possibleObstacleX = [robotX+1];
                        possibleObstacleY = [robotY];
                    end
                elseif (robotX == micromouse.map.maxX)
                    if ((robotY > 2) && (micromouse.map.coordinates(robotX, robotY-1) ~= micromouse.map.legend.obstacle))
                        possibleObstacleX = [robotX-1, robotX-1, robotX, robotX];
                        possibleObstacleY = [robotY, robotY-1, robotY-1, robotY-2];
                    elseif (robotY > 1)
                        possibleObstacleX = [robotX-1, robotX-1, robotX];
                        possibleObstacleY = [robotY, robotY-1, robotY-1];
                    else
                        possibleObstacleX = [robotX-1];
                        possibleObstacleY = [robotY];
                    end
                end
            else
                if ((robotY > 1) && (robotY < micromouse.map.maxY))
                    if ((robotX > 2) && (micromouse.map.coordinates(robotX-1, robotY) ~= micromouse.map.legend.obstacle))
                        possibleObstacleX = [robotX, robotX-1, robotX-1, robotX-2, robotX, robotX-1];
                        possibleObstacleY = [robotY+1, robotY+1, robotY, robotY, robotY-1, robotY-1];
                    elseif (robotX < micromouse.map.maxX)
                        possibleObstacleX = [robotX, robotX-1, robotX-1, robotX, robotX-1];
                        possibleObstacleY = [robotY+1, robotY+1, robotY, robotY-1, robotY-1];
                    else
                        possibleObstacleX = [robotX, robotX];
                        possibleObstacleY = [robotY+1, robotY-1];
                    end
                elseif (robotY == 1)
                    if ((robotX > 2) && (micromouse.map.coordinates(robotX-1, robotY) ~= micromouse.map.legend.obstacle))
                        possibleObstacleX = [robotX, robotX-1, robotX-1, robotX-2];
                        possibleObstacleY = [robotY+1, robotY+1, robotY, robotY];
                    elseif (robotX < micromouse.map.maxX)
                        possibleObstacleX = [robotX, robotX-1, robotX-1];
                        possibleObstacleY = [robotY+1, robotY+1, robotY];
                    else
                        possibleObstacleX = [robotX];
                        possibleObstacleY = [robotY+1];
                    end
                elseif (robotY == micromouse.map.maxY)
                    if ((robotX > 2) && (micromouse.map.coordinates(robotX-1, robotY) ~= micromouse.map.legend.obstacle))
                        possibleObstacleX = [robotX-1, robotX-2, robotX, robotX-1];
                        possibleObstacleY = [robotY, robotY, robotY-1, robotY-1];
                    elseif (robotX < micromouse.map.maxX)
                        possibleObstacleX = [robotX-1, robotX, robotX-1];
                        possibleObstacleY = [robotY, robotY-1, robotY-1];
                    else
                        possibleObstacleX = [robotX];
                        possibleObstacleY = [robotY-1];
                    end
                end
            end
            
            obstacles = [possibleObstacleX', possibleObstacleY'];
            
        end
        
        function updateLineOfSight(micromouse)
        % EXAMPLE FUNCTION CALL: updateLineOfSight()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-10-11
        % PURPOSE: Updates line of sight of robot
            
            obstacles = micromouse.getLineOfSight();
            micromouse.addObstacles(obstacles);
            
        end
        
        function addObstacles(micromouse, obstacles)
        % EXAMPLE FUNCTION CALL: updateLineOfSight()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-10-11
        % PURPOSE: Check if provided obstacles already exist in the map and display them if not
            
            figure(micromouse.display.figureHandle);
            
            obstacleAlreadyDisplayed = 0;
            for obstacle = 1:size(obstacles, 1)
                if (micromouse.map.coordinates(obstacles(obstacle, 1), obstacles(obstacle, 2)) == micromouse.map.legend.obstacle)
                    for previousObstacle = 1:size(micromouse.display.displayedObstacles, 1)
                        if (obstacles(obstacle, 1) == micromouse.display.displayedObstacles(previousObstacle, 1))
                            if (obstacles(obstacle, 2) == micromouse.display.displayedObstacles(previousObstacle, 2))
                                obstacleAlreadyDisplayed = 1;
                                break;
                            end
                        end
                    end
                    if (~obstacleAlreadyDisplayed)
                        micromouse.display.displayedObstacles = [micromouse.display.displayedObstacles; obstacles(obstacle, :)];
                        displayLine = displayObstacle(micromouse.map.coordinates, obstacles(obstacle, 1), obstacles(obstacle, 2), ...
                                                      micromouse.robot.location, micromouse.robot.direction, micromouse.map.legend, micromouse.display.displayedLines, 1);
                    else
                        obstacleAlreadyDisplayed = 0;
                        displayLine = displayObstacle(micromouse.map.coordinates, obstacles(obstacle, 1), obstacles(obstacle, 2), ...
                                                      micromouse.robot.location, micromouse.robot.direction, micromouse.map.legend, micromouse.display.displayedLines, 0);
                    end
                    if (displayLine(1) ~= 0)
                        micromouse.display.displayedLines = [micromouse.display.displayedLines; displayLine];
                    end
                end
            end
            
            drawnow;
            
            function [displayLine] = displayObstacle(map, x, y, robotLocation, robotDirection, legend, displayedLines, displayObstacleDot)
            % EXAMPLE FUNCTION CALL: displayObstacle(micromouse.map.coordinates, x, y, micromouse.robot.location, micromouse.robot.direction, micromouse.map.legends, 1)
            % PROGRAMMER: Frederick Wachter
            % DATE CREATED: 2016-10-11
            % PURPOSE: Displays lines between neighboring obstacles using line of sight of the robot
            
                displayLine = zeros(1,2);
                if (robotDirection == 1)
                    if ((y == (robotLocation(2)+1)) && (x == robotLocation(1)))
                        if ((x > 1) && (map(x-1, y) == legend.obstacle))
                            if (~lineAlreadyDisplayed(x-0.5, y, displayedLines))
                                line([x-1, x], [y, y], 'Color', 'r');
                                displayLine = [x-0.5, y];
                            end
                        end
                        if ((x < size(map, 1)) && (map(x+1, y) == legend.obstacle))
                            if (~lineAlreadyDisplayed(x+0.5, y, displayedLines))
                                line([x, x+1], [y, y], 'Color', 'r');
                                displayLine = [x+0.5, y];
                            end
                        end
                    elseif (y ~= (robotLocation(2)+2))
                        if (y == robotLocation(2))
                            if ((y < size(map, 2)) && (map(x, y+1) == legend.obstacle))
                                if (~lineAlreadyDisplayed(x, y+0.5, displayedLines))
                                    line([x, x], [y, y+1], 'Color', 'r');
                                    displayLine = [x, y+0.5];
                                end
                            end
                        elseif (y == robotLocation(2)+1)
                            lineDisplayed = 0;
                            if (x < robotLocation(1))
                                if (map(x+1, y) == legend.obstacle)
                                    if (~lineAlreadyDisplayed(x+0.5, y, displayedLines))
                                        line([x, x+1], [y, y], 'Color', 'r');
                                        displayLine = [x+0.5, y];
                                        lineDisplayed = 1;
                                    end
                                end
                            else
                                if (map(x-1, y) == legend.obstacle)
                                    if (~lineAlreadyDisplayed(x-0.5, y, displayedLines))
                                        line([x-1, x], [y, y], 'Color', 'r');
                                        displayLine = [x-0.5, y];
                                        lineDisplayed = 1;
                                    end
                                end
                            end
                            if (map(x, y-1) == legend.obstacle)
                                if (~lineAlreadyDisplayed(x, y-0.5, displayedLines))
                                    line([x, x], [y-1, y], 'Color', 'r');
                                    if (lineDisplayed)
                                        displayLine = [displayLine; x, y-0.5];
                                    else
                                        displayLine = [x, y-0.5];
                                    end
                                end
                            end
                        end
                    end
                elseif (robotDirection == 2)
                    if ((x == (robotLocation(1)+1)) && (y == robotLocation(2)))
                        if ((y > 1) && (map(x, y-1) == legend.obstacle))
                            if (~lineAlreadyDisplayed(x, y-0.5, displayedLines))
                                line([x, x], [y-1, y], 'Color', 'r');
                                displayLine = [x, y-0.5];
                            end
                        end
                        if ((y < size(map, 2)) && (map(x, y+1) == legend.obstacle))
                            if (~lineAlreadyDisplayed(x, y+0.5, displayedLines))
                                line([x, x], [y, y+1], 'Color', 'r');
                                displayLine = [x, y+0.5];
                            end
                        end
                    elseif (x ~= (robotLocation(1)+2))
                        if (x == robotLocation(1))
                            if ((x < size(map, 1)) && (map(x+1, y) == legend.obstacle))
                                if (~lineAlreadyDisplayed(x+0.5, y, displayedLines))
                                    line([x, x+1], [y, y], 'Color', 'r');
                                    displayLine = [x+0.5, y];
                                end
                            end
                        elseif (x == robotLocation(1)+1)
                            lineDisplayed = 0;
                            if (y > robotLocation(2))
                                if (map(x, y-1) == legend.obstacle)
                                    if (~lineAlreadyDisplayed(x, y-0.5, displayedLines))
                                        line([x, x], [y-1, y], 'Color', 'r');
                                        displayLine = [x, y-0.5];
                                        lineDisplayed = 1;
                                    end
                                end
                            else
                                if (map(x, y+1) == legend.obstacle)
                                    if (~lineAlreadyDisplayed(x, y+0.5, displayedLines))
                                        line([x, x], [y, y+1], 'Color', 'r');
                                        displayLine = [x, y+0.5];
                                        lineDisplayed = 1;
                                    end
                                end
                            end
                            if (map(x-1, y) == legend.obstacle)
                                if (~lineAlreadyDisplayed(x-0.5, y, displayedLines))
                                    line([x-1, x], [y, y], 'Color', 'r');
                                    if (lineDisplayed)
                                        displayLine = [displayLine; x-0.5, y];
                                    else
                                        displayLine = [x-0.5, y];
                                    end
                                end
                            end
                        end
                    end
                elseif (robotDirection == 3)
                    if ((y == (robotLocation(2)-1)) && (x == robotLocation(1)))
                        if ((x > 1) && (map(x-1, y) == legend.obstacle))
                            if (~lineAlreadyDisplayed(x-0.5, y, displayedLines))
                                line([x-1, x], [y, y], 'Color', 'r');
                                displayLine = [x-0.5, y];
                            end
                        end
                        if ((x < size(map, 1)) && (map(x+1, y) == legend.obstacle))
                            if (~lineAlreadyDisplayed(x+0.5, y, displayedLines))
                                line([x, x+1], [y, y], 'Color', 'r');
                                displayLine = [x+0.5, y];
                            end
                        end
                    elseif  (y ~= (robotLocation(2)-2))
                        if (y == robotLocation(2))
                            if ((y > 1) && (map(x, y-1) == legend.obstacle))
                                if (~lineAlreadyDisplayed(x, y-0.5, displayedLines))
                                    line([x, x], [y-1, y], 'Color', 'r');
                                    displayLine = [x, y-0.5];
                                end
                            end
                        elseif (y == robotLocation(2)-1)
                            lineDisplayed = 0;
                            if (x > robotLocation(1))
                                if (map(x-1, y) == legend.obstacle)
                                    if (~lineAlreadyDisplayed(x-0.5, y, displayedLines))
                                        line([x-1, x], [y, y], 'Color', 'r');
                                        displayLine = [x-0.5, y];
                                        lineDisplayed = 1;
                                    end
                                end
                            else
                                if (map(x+1, y) == legend.obstacle)
                                    if (~lineAlreadyDisplayed(x+0.5, y, displayedLines))
                                        line([x, x+1], [y, y], 'Color', 'r');
                                        displayLine = [x+0.5, y];
                                        lineDisplayed = 1;
                                    end
                                end
                            end
                            if (map(x, y+1) == legend.obstacle)
                                if (~lineAlreadyDisplayed(x, y+0.5, displayedLines))
                                    line([x, x], [y, y+1], 'Color', 'r');
                                    if (lineDisplayed)
                                        displayLine = [displayLine; x, y+0.5];
                                    else
                                        displayLine = [x, y+0.5];
                                    end
                                end
                            end
                        end
                    end
                else 
                    if ((x == (robotLocation(1)-1)) && (y == robotLocation(2)))
                        if ((y > 1) && (map(x, y-1) == legend.obstacle))
                            if (~lineAlreadyDisplayed(x, y-0.5, displayedLines))
                                line([x, x], [y-1, y], 'Color', 'r');
                                displayLine = [x, y-0.5];
                            end
                        end
                        if ((y < size(map, 2)) && (map(x, y+1) == legend.obstacle))
                            if (~lineAlreadyDisplayed(x, y+0.5, displayedLines))
                                line([x, x], [y, y+1], 'Color', 'r');
                                displayLine = [x, y+0.5];
                            end
                        end
                    elseif (x ~= (robotLocation(1)-2))
                        if (x == robotLocation(1))
                            if ((x > 1) && (map(x-1, y) == legend.obstacle))
                                if (~lineAlreadyDisplayed(x-0.5, y, displayedLines))
                                    line([x-1, x], [y, y], 'Color', 'r');
                                    displayLine = [x-0.5, y];
                                end
                            end
                        elseif (x == robotLocation(1)-1)
                            lineDisplayed = 0;
                            if (y > robotLocation(1))
                                if (map(x, y-1) == legend.obstacle)
                                    if (~lineAlreadyDisplayed(x, y-0.5, displayedLines))
                                        line([x, x], [y-1, y], 'Color', 'r');
                                        displayLine = [x, y-0.5];
                                        lineDisplayed = 1;
                                    end
                                end
                            else
                                if (map(x, y+1) == legend.obstacle)
                                    if (~lineAlreadyDisplayed(x, y+0.5, displayedLines))
                                        line([x, x], [y, y+1], 'Color', 'r');
                                        displayLine = [x, y+0.5];
                                        lineDisplayed = 1;
                                    end
                                end
                            end
                            if (map(x+1, y) == legend.obstacle)
                                if (~lineAlreadyDisplayed(x+0.5, y, displayedLines))
                                    line([x, x+1], [y, y], 'Color', 'r');
                                    if (lineDisplayed)
                                        displayLine = [displayLine; x+0.5, y];
                                    else
                                        displayLine = [x+0.5, y];
                                    end
                                end
                            end
                        end
                    end
                end
                
                if (displayObstacleDot)
                    plot(x, y, 'or');
                end
                
                function alreadyDisplayed = lineAlreadyDisplayed(x, y, displayedLines)
                % EXAMPLE FUNCTION CALL: alreadyDisplayed = lineAlreadyDisplayed(x, y, displayedLines)
                % PROGRAMMER: Frederick Wachter
                % DATE CREATED: 2016-10-11
                % PURPOSE: Determine if a desired line to be drawn has already been drawn
            
                    alreadyDisplayed = 0;
                    for displayedLine = 1:size(displayedLines)
                        if (x == displayedLines(displayedLine, 1))
                            if (y == displayedLines(displayedLine, 2))
                                alreadyDisplayed = 1;
                                break;
                            end
                        end
                    end
                    
                end
                
            end
            
        end

        function collisionDetection(micromouse)
            
            
            
        end
        
        
        % Display Functions
        
        function displayMap(micromouse)
        % EXAMPLE FUNCTION CALL: micromouse.displayMap()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-10-11
        % PURPOSE: Display the map with only objects that the micromouse can see
            
            % Initialize the figure
            if (~ishandle(micromouse.display.figureHandle))
                micromouse.display.figureHandle = figure('Name', 'A* Algorithm', 'NumberTitle', 'off'); % initialize figure
                axis([0.5, micromouse.map.maxX+0.5, 0.5, micromouse.map.maxY+0.5]); % initialize axis spacing
                axis square; grid on; hold on; % set axis properties
            end
            
            % Display obstacles, robot, and target location
            for x = 1:micromouse.map.maxX % for all map x locations
                for y = 1:micromouse.map.maxY % for all map y locations
                    if (micromouse.map.coordinates(x, y) == micromouse.map.legend.obstacle) % if the current location is an obstacle
                        micromouse.display.obstacleCount = micromouse.display.obstacleCount + 1; % increment obstacle counter
                    elseif (micromouse.map.coordinates(x, y) == micromouse.map.legend.start) % if the current locaiton is the robot
                        micromouse.robot.location = [x, y];
                        micromouse.display.handles.robot = plot(x, y, 'bo'); % show the robot
                    end
                end
            end
            drawnow; % force MATLAB to draw the figure before continuing
            
        end
        
        function displayFullMap(micromouse)
        % EXAMPLE FUNCTION CALL: micromouse.displayFullMap()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Display the map with obstacles, start location, and target location
            
            % Initialize the figure
            if (~ishandle(micromouse.display.handles.fullMap))
                micromouse.display.handles.fullMap = figure('Name', 'A* Algorithm Full Map', 'NumberTitle', 'off'); % initialize figure
                axis([0.5, micromouse.map.maxX+0.5, 0.5, micromouse.map.maxY+0.5]); % initialize axis spacing
                axis square; grid on; hold on; % set axis properties
            end
            
            % Display obstacles, robot, and target location
            for x = 1:micromouse.map.maxX % for all map x locations
                for y = 1:micromouse.map.maxY % for all map y locations
                    if (micromouse.map.coordinates(x, y) == micromouse.map.legend.obstacle) % if the current location is an obstacle
                        plot(x, y, 'ro'); % show the obstacle
                        displayBorders(micromouse.map.coordinates, x, y, micromouse.map.legend.obstacle); % display connected borders
                    elseif (micromouse.map.coordinates(x, y) == micromouse.map.legend.target) % if the current location is the target
                        plot(x, y, 'gd'); % show the target
                    elseif (micromouse.map.coordinates(x, y) == micromouse.map.legend.start) % if the current locaiton is the robot
                        plot(x, y, 'bo'); % show the robot
                    end
                end
            end
            drawnow; % force MATLAB to draw the figure before continuing
            
            function displayBorders(map, x, y, obstacle)
            % EXAMPLE FUNCTION CALL: displayBorders(micromouse.map.coordinates, x, y, micromouse.map.legend.obstacle)
            % PROGRAMMER: Frederick Wachter
            % DATE CREATED: 2016-06-07
            % PURPOSE: Displays lines between neighboring obstacles
                
                if (x ~= 1)
                    if (map(x-1, y) == obstacle)
                        line([x-1, x], [y, y], 'Color', 'r');
                    end
                end
                if (y ~= 1)
                    if (map(x, y-1) == obstacle)
                        line([x, x], [y-1, y], 'Color', 'r');
                    end
                end
                
            end
            
        end
        
        
        % Example Functions
        
        function exampleMovement(micromouse)
            
            delayTime = 0.05;
            
            micromouse.moveRobot(micromouse.robot.location(1), micromouse.robot.location(2)-1);
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1), micromouse.robot.location(2)-1);
            micromouse.updateLineOfSight();
            pause(delayTime);
            
            micromouse.robot.direction = 2;
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            
            micromouse.robot.direction = 1;
            micromouse.moveRobot(micromouse.robot.location(1), micromouse.robot.location(2)+1);
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1), micromouse.robot.location(2)+1);
            micromouse.updateLineOfSight();
            pause(delayTime);
            
            micromouse.robot.direction = 2;
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            
            micromouse.robot.direction = 3;
            micromouse.moveRobot(micromouse.robot.location(1), micromouse.robot.location(2)-1);
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1), micromouse.robot.location(2)-1);
            micromouse.updateLineOfSight();
            pause(delayTime);
            
            micromouse.robot.direction = 2;
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            
            micromouse.robot.direction = 1;
            micromouse.moveRobot(micromouse.robot.location(1), micromouse.robot.location(2)+1);
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1), micromouse.robot.location(2)+1);
            micromouse.updateLineOfSight();
            pause(delayTime);
            
            micromouse.robot.direction = 2;
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            
            micromouse.robot.direction = 3;
            micromouse.moveRobot(micromouse.robot.location(1), micromouse.robot.location(2)-1);
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1), micromouse.robot.location(2)-1);
            micromouse.updateLineOfSight();
            pause(delayTime);
            
            micromouse.robot.direction = 2;
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            micromouse.moveRobot(micromouse.robot.location(1)+1, micromouse.robot.location(2));
            micromouse.updateLineOfSight();
            pause(delayTime);
            
        end
        
    end
    
end


