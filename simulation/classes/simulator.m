%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
% DATE CREATED: 2016-12-10
% PURPOSE: Micromouse simulator class

% Please refer to the Wiki for instructions on how to use this class
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB


classdef simulator < handle
    
    properties(SetAccess = protected)
        robot
    end
    
    properties(SetAccess = private, GetAccess = private)
        map
        sensor
        display
    end

%% CONSTRUCTOR METHOD
    methods
        
        function sim = simulator(varargin)
        % EXAMPLE FUNCTION CALL: sim = simulator(varargin)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-11
        % PURPOSE: Construct the structure
        % TYPE: Constructor Method
            
            % Map Properties
            sim.map.coordinates = [];
            sim.map.maxX        = 0;
            sim.map.maxY        = 0;
            
            sim.map.targetLocation   = [0,0];
            sim.map.startLocation    = [0,0];
            
            % Figure Properties
            sim.display.obstacleCount         = 0;
            sim.display.figureHandle          = -1;
            sim.display.handles.robot         = -1;
            sim.display.displayedObstacles    = [];
            sim.display.displayedLines        = [];
            sim.display.delaySpeed            = 0.001;
            sim.display.on                    = 0;
            sim.display.initialRobotLocation  = zeros(1, 2);
            sim.display.initialRobotDirection = 0;
            sim.display.mapSet                = 0;
            
            % Robot Properties
            sim.robot.location       = [0,0];
            sim.robot.direction      = 1; % N: 1, E: 2, S: 3, W: 4
            sim.robot.openDirections = ones(1,4);
            sim.robot.map            = [];
            sim.robot.movements      = 0;
            
            % Sensor Properties
            sim.sensor.lineOfSight = 2;
            
        end
        
    end
    
%% PUBLIC METHODS

    methods
        
        function getMap(sim, map, displaySim)
        % EXAMPLE FUNCTION CALL: sim.getMap(map)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-11
        % PURPOSE: Load map into simulation
            
            % Get Initial Map Properties
            sim.map.coordinates = map.data;
            sim.map.legend      = map.legend;
            sim.map.maxX        = size(map.data, 1);
            sim.map.maxY        = size(map.data, 2);
            
            sim.robot.map    = ones(size(map.data))*map.legend.freeSpace;
            sim.robot.legend = map.legend;
            
            sim.display.on = displaySim;
            
            if (sim.display.on)
                % Reset Figure Properties
                sim.display.figureHandle     = -1;

                % Display Figure
                sim.initializeFigure();
            end
            sim.displayFullMap();
            sim.displayMap();
            
            sim.display.mapSet = 1;
            
            % Get Initial Parameters
            sim.getRobotOrientation();
            sim.updateLineOfSight();
            sim.checkRobotBoundaries();
            pause(0.5);
            drawnow;

        end
        
        function moveRobot(sim, direction)
        % EXAMPLE FUNCTION CALL: sim.moveRobot(direction)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-11
        % PURPOSE: Move robot one unit in a specified direction in simulation
        
            sim.robot.movements = sim.robot.movements + 1;
            
            if (sim.display.on)
                figure(sim.display.figureHandle); subplot(121); % remove if faster speed needed
            end
            
            tic;
            % Check If Movement Allowed
            if (sim.robot.openDirections(direction) == 0)
                error('Cannot move in desired direction. Not performing movement');
            end
            
            % Update Robot Direction
            sim.robot.direction = direction;
            
            % Update Robot Location
            sim.updateRobotLocation();
            
            % Update Robot Sensors
            sim.updateLineOfSight();
            sim.checkRobotBoundaries();
            drawnow;
            
            if (sim.display.on)
                time = max(sim.display.delaySpeed - toc, 0);
                pause(time);
            end
            
        end
        
        function alive = isFigureAlive(sim)
        % EXAMPLE FUNCTION CALL: sim.isFigureAlive()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-11
        % PURPOSE: Check if simulation figure is still alive
            
            alive = 0;
            pause(0.0001);
            if (ishandle(sim.display.figureHandle))
                alive = 1;
            end
            
        end
        
        function displayGoal(sim, location)
        % EXAMPLE FUNCTION CALL: sim.displayGoal(location)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-15
        % PURPOSE: Display goal location on the map
            
            if (sim.display.on)
                figure(sim.display.figureHandle); subplot(121);
                plot(location(1), location(2), 'ok');
            end
            
            sim.robot.map(location(1), location(2)) = sim.robot.legend.target;
            
        end
        
        function exportDirectionVariables(sim, fileName)
        % EXAMPLE FUNCTION CALL: sim.exportDirectionVariables(fileName)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-30
        % PURPOSE: Export log files of all open squares and their available directions 

            if (sim.display.mapSet)
                % Change Directory to Log File Directory
                filePath = mfilename('fullpath');
                removalIndex = find(filePath == '/', 2, 'last');
                cd(filePath(1:removalIndex(1)-1));
                cd logFiles;

                % Open File
                fileName = strcat(fileName, '.txt');
                fileID = fopen(fileName, 'w');

                % Write File Header
                fprintf(fileID, 'Exported open square available locations from map\n\n');
                fprintf(fileID, '_____ Legend _____\n');
                fprintf(fileID, '[initial robot direction] [initial right wall open]\n');
                fprintf(fileID, '[location] [available directions]\n\n-----\n\n');

                % Get Rqeuired Data
                initialDirection = sim.display.initialRobotDirection;
                if (initialDirection == 1)
                    if (sim.display.initialRobotLocation(1) == sim.map.maxX)
                        initialRightWallOpen = 0;
                    else
                        initialRightWallOpen = (sim.map.coordinates(sim.display.initialRobotLocation(1)+1, sim.display.initialRobotLocation(2)) == sim.map.legend.freeSpace);
                    end
                elseif (initialDirection == 2)
                    if (sim.display.initialRobotLocation(2) == 1)
                        initialRightWallOpen = 0;
                    else
                        initialRightWallOpen = (sim.map.coordinates(sim.display.initialRobotLocation(1), sim.display.initialRobotLocation(2)-1) == sim.map.legend.freeSpace);
                    end
                elseif (initialDirection == 3)
                    if (sim.display.initialRobotLocation(1) == 1)
                        initialRightWallOpen = 0;
                    else
                        initialRightWallOpen = (sim.map.coordinates(sim.display.initialRobotLocation(1)-1, sim.display.initialRobotLocation(2)) == sim.map.legend.freeSpace);
                    end
                else
                    if (sim.display.initialRobotLocation(2) == sim.map.maxY)
                        initialRightWallOpen = 0;
                    else
                        initialRightWallOpen = (sim.map.coordinates(sim.display.initialRobotLocation(1), sim.display.initialRobotLocation(2)+1) == sim.map.legend.freeSpace);
                    end
                end

                % Write Data to File
                fprintf(fileID, '[%d] [%d]\n', initialDirection-1, initialRightWallOpen); % in C++, directions start with 0 not 1 as in MATLAB
                for x = 1:sim.map.maxX
                    for y = 1:sim.map.maxY
                        if (sim.map.coordinates(x, y) == sim.robot.legend.freeSpace)
                            directions = zeros(1, 4);
                            if ((y < sim.map.maxY) && (sim.map.coordinates(x, y+1) == sim.robot.legend.freeSpace)); directions(1) = 1; end
                            if ((x < sim.map.maxX) && (sim.map.coordinates(x+1, y) == sim.robot.legend.freeSpace)); directions(2) = 1; end
                            if ((y > 1) && (sim.map.coordinates(x, y-1) == sim.robot.legend.freeSpace)); directions(3) = 1; end
                            if ((x > 1) && (sim.map.coordinates(x-1, y) == sim.robot.legend.freeSpace)); directions(4) = 1; end

                            fprintf(fileID, '[%d %d] [%d %d %d %d]\n', x, y, directions(1), directions(2), directions(3), directions(4));
                        end
                    end
                end
                fclose(fileID);

                fprintf('Export successful\n');
            else
                fprintf('Error: Map was not set before running function\nPlease set the map using the function getMap() before running this function\n');
            end
        end
        
    end

%% PRIVATE METHODS

    methods (Access = private)
        
        % Robot Function
        
        function updateRobotLocation(sim)
        % EXAMPLE FUNCTION CALL: sim.updateRobotLocation()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-10-12
        % PURPOSE: Update position of sim based on direction
            
            % Update Location
            if (sim.robot.direction == 1)
                sim.robot.location(2) = sim.robot.location(2) + 1;
            elseif (sim.robot.direction == 2)
                sim.robot.location(1) = sim.robot.location(1) + 1;
            elseif (sim.robot.direction == 3)
                sim.robot.location(2) = sim.robot.location(2) - 1;
            else
                sim.robot.location(1) = sim.robot.location(1) - 1;
            end
            
            % Update Display Location
            if (sim.display.on == 1)
                set(sim.display.handles.robot, 'XData', sim.robot.location(1), 'YData', sim.robot.location(2));
            end
            
        end
        
        function checkRobotBoundaries(sim)
        % EXAMPLE FUNCTION CALL: sim.checkMapBoundaries()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-10-12
        % PURPOSE: Check if robot is at map boundaries
        
            sim.robot.openDirections = ones(1,4);
        
            if ((sim.robot.location(2) == sim.map.maxY) || ... 
               ((sim.robot.location(2) ~= sim.map.maxY) && ... 
                (sim.map.coordinates(sim.robot.location(1), sim.robot.location(2)+1) == sim.map.legend.obstacle)))
                sim.robot.openDirections(1) = 0;
            end
            if ((sim.robot.location(1) == sim.map.maxX) || ... 
               ((sim.robot.location(1) ~= sim.map.maxX) && ... 
                (sim.map.coordinates(sim.robot.location(1)+1, sim.robot.location(2)) == sim.map.legend.obstacle)))
                sim.robot.openDirections(2) = 0;
            end
            if ((sim.robot.location(2) == 1) || ... 
               ((sim.robot.location(2) ~= 1) && ... 
                (sim.map.coordinates(sim.robot.location(1), sim.robot.location(2)-1) == sim.map.legend.obstacle)))
                sim.robot.openDirections(3) = 0;
            end
            if ((sim.robot.location(1) == 1) || ... 
               ((sim.robot.location(1) ~= 1) && ... 
                (sim.map.coordinates(sim.robot.location(1)-1, sim.robot.location(2)) == sim.map.legend.obstacle)))
                sim.robot.openDirections(4) = 0;
            end
        
        end
        
        function getRobotOrientation(sim)
        % EXAMPLE FUNCTION CALL: sim.getRobotOrientation()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-10-11
        % PURPOSE: Get initial orientation of the robot
        
            % Initialize Robot Direction
            directionSet = 0;
            if ((sim.robot.location(2) ~= sim.map.maxY) && (sim.map.coordinates(sim.robot.location(1), sim.robot.location(2)+1) == sim.map.legend.freeSpace))
                sim.robot.direction = 1;
                directionSet = 1;
            elseif ((sim.robot.location(1) ~= sim.map.maxX) && (sim.map.coordinates(sim.robot.location(1)+1, sim.robot.location(2)) == sim.map.legend.freeSpace))
                sim.robot.direction = 2;
                directionSet = 1;
            elseif ((sim.robot.location(2) ~= 1) && (sim.map.coordinates(sim.robot.location(1), sim.robot.location(2)-1) == sim.map.legend.freeSpace))
                sim.robot.direction = 3;
                directionSet = 1;
            elseif (sim.robot.location(1) ~= 1)
                sim.robot.direction = 4;
                directionSet = 1;
            end
            
            if ~(directionSet)
                error('Fatal error in getRobotOrientation function');
            else
                sim.display.initialRobotDirection = sim.robot.direction;
            end
            
            % Get Potential Obstacle Locations Based on Start Location and Direction
            obstacles = getAssumedBoundaries(sim.map, sim.robot.location(1), sim.robot.location(2), sim.robot.direction);

            % Display Obstacles if Potential Obstacles are Validated to be Obstacles
            for obstacle = 1:size(obstacles, 1)
                if (sim.map.coordinates(obstacles(obstacle, 1), obstacles(obstacle, 2)) == sim.map.legend.obstacle)
                    if (sim.display.on)
                        plot(obstacles(obstacle, 1), obstacles(obstacle, 2), 'or');
                    end
                    sim.display.displayedObstacles = [sim.display.displayedObstacles; obstacles(obstacle, :)];
                    sim.robot.map(obstacles(obstacle, 1), obstacles(obstacle, 2)) = sim.robot.legend.obstacle;
                    
                    if (sim.display.on)
                        for comparisonObstacle = 1:size(obstacles, 1)
                            if (sim.map.coordinates(obstacles(comparisonObstacle, 1), obstacles(comparisonObstacle, 2)) == sim.map.legend.obstacle)
                                if (obstacle ~= comparisonObstacle)
                                    if (sqrt((obstacles(obstacle, 1) - obstacles(comparisonObstacle, 1))^2 + (obstacles(obstacle, 2) - obstacles(comparisonObstacle, 2))^2) == 1)
                                        line([obstacles(obstacle, 1), obstacles(comparisonObstacle, 1)], [obstacles(obstacle, 2), obstacles(comparisonObstacle, 2)], 'Color', 'r');
                                        sim.display.displayedLines = [sim.display.displayedLines; (obstacles(obstacle, 1)+obstacles(comparisonObstacle, 1))/2, (obstacles(obstacle, 2)-obstacles(comparisonObstacle, 2))/2];
                                        break;
                                    end
                                end
                            end
                        end
                    end
                end
            end
            drawnow;
            
            function obstacles = getAssumedBoundaries(map, robotX, robotY, robotDirection)
            % EXAMPLE FUNCTION CALL: sim.getRobotOrientation()
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
           
        end
        
        function obstacles = getLineOfSight(sim)
        % EXAMPLE FUNCTION CALL: updateLineOfSight()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-10-11
        % PURPOSE: Gets the list of obstacles the robot can see from its position
            
            % Get Line of Sight Obstacles
            possibleObstacleX = [];
            possibleObstacleY = [];
            robotX = sim.robot.location(1);
            robotY = sim.robot.location(2);
            
            if (sim.robot.direction == 1)
                if ((robotX > 1) && (robotX < sim.map.maxX))
                    if ((robotY < sim.map.maxY - 1) && (sim.map.coordinates(robotX, robotY+1) ~= sim.map.legend.obstacle))
                        possibleObstacleX = [robotX-1, robotX, robotX, robotX+1];
                        possibleObstacleY = [robotY+1, robotY+1, robotY+2, robotY+1];
                    elseif (robotY < sim.map.maxY)
                        possibleObstacleX = [robotX-1, robotX, robotX+1];
                        possibleObstacleY = [robotY+1, robotY+1, robotY+1];
                    end
                elseif (robotX == 1)
                    if ((robotY < sim.map.maxY - 1) && (sim.map.coordinates(robotX, robotY+1) ~= sim.map.legend.obstacle))
                        possibleObstacleX = [robotX, robotX, robotX+1];
                        possibleObstacleY = [robotY+1, robotY+2,robotY+1];
                    elseif (robotY < sim.map.maxY)
                        possibleObstacleX = [robotX, robotX+1];
                        possibleObstacleY = [robotY+1, robotY+1];
                    end
                elseif (robotX == sim.map.maxX)
                    if ((robotY < sim.map.maxY - 1) && (sim.map.coordinates(robotX, robotY+1) ~= sim.map.legend.obstacle))
                        possibleObstacleX = [robotX-1, robotX, robotX];
                        possibleObstacleY = [robotY+1, robotY+1, robotY+2];
                    elseif (robotY < sim.map.maxY)
                        possibleObstacleX = [robotX-1, robotX];
                        possibleObstacleY = [robotY+1, robotY+1];
                    end
                end
            elseif (sim.robot.direction == 2)
                if ((robotY > 1) && (robotY < sim.map.maxY))
                    if ((robotX < sim.map.maxX - 1) && (sim.map.coordinates(robotX+1, robotY) ~= sim.map.legend.obstacle))
                        possibleObstacleX = [robotX+1, robotX+1, robotX+2, robotX+1];
                        possibleObstacleY = [robotY+1, robotY, robotY, robotY-1];
                    elseif (robotX < sim.map.maxX)
                        possibleObstacleX = [robotX+1, robotX+1, robotX+1];
                        possibleObstacleY = [robotY+1, robotY, robotY-1];
                    end
                elseif (robotY == 1)
                    if ((robotX < sim.map.maxX - 1) && (sim.map.coordinates(robotX+1, robotY) ~= sim.map.legend.obstacle))
                        possibleObstacleX = [robotX+1, robotX+1, robotX+2];
                        possibleObstacleY = [robotY+1, robotY, robotY];
                    elseif (robotX < sim.map.maxX)
                        possibleObstacleX = [robotX+1, robotX+1];
                        possibleObstacleY = [robotY+1, robotY];
                    end
                elseif (robotY == sim.map.maxY)
                    if ((robotX < sim.map.maxX - 1) && (sim.map.coordinates(robotX+1, robotY) ~= sim.map.legend.obstacle))
                        possibleObstacleX = [robotX+1, robotX+2, robotX+1];
                        possibleObstacleY = [robotY, robotY, robotY-1];
                    elseif (robotX < sim.map.maxX)
                        possibleObstacleX = [robotX+1, robotX+1];
                        possibleObstacleY = [robotY, robotY-1];
                    end
                end
            elseif (sim.robot.direction == 3)
                if ((robotX > 1) && (robotX < sim.map.maxX))
                    if ((robotY > 2) && (sim.map.coordinates(robotX, robotY-1) ~= sim.map.legend.obstacle))
                        possibleObstacleX = [robotX-1, robotX, robotX, robotX+1];
                        possibleObstacleY = [robotY-1, robotY-1, robotY-2, robotY-1];
                    elseif (robotY > 1)
                        possibleObstacleX = [robotX-1, robotX, robotX+1];
                        possibleObstacleY = [robotY-1, robotY-1, robotY-1];
                    end
                elseif (robotX == 1)
                    if ((robotY > 2) && (sim.map.coordinates(robotX, robotY-1) ~= sim.map.legend.obstacle))
                        possibleObstacleX = [robotX, robotX, robotX+1];
                        possibleObstacleY = [robotY-1, robotY-2, robotY-1];
                    elseif (robotY > 1)
                        possibleObstacleX = [robotX, robotX+1];
                        possibleObstacleY = [robotY-1, robotY-1];
                    end
                elseif (robotX == sim.map.maxX)
                    if ((robotY > 2) && (sim.map.coordinates(robotX, robotY-1) ~= sim.map.legend.obstacle))
                        possibleObstacleX = [robotX-1, robotX, robotX];
                        possibleObstacleY = [robotY-1, robotY-1, robotY-2];
                    elseif (robotY > 1)
                        possibleObstacleX = [robotX-1, robotX];
                        possibleObstacleY = [robotY-1, robotY-1];
                    end
                end
            else
                if ((robotY > 1) && (robotY < sim.map.maxY))
                    if ((robotX > 2) && (sim.map.coordinates(robotX-1, robotY) ~= sim.map.legend.obstacle))
                        possibleObstacleX = [robotX-1, robotX-1, robotX-2, robotX-1];
                        possibleObstacleY = [robotY+1, robotY, robotY, robotY-1];
                    elseif ((robotX < sim.map.maxX) && (robotX > 1))
                        possibleObstacleX = [robotX-1, robotX-1, robotX-1];
                        possibleObstacleY = [robotY+1, robotY, robotY-1];
                    end
                elseif (robotY == 1)
                    if ((robotX > 2) && (sim.map.coordinates(robotX-1, robotY) ~= sim.map.legend.obstacle))
                        possibleObstacleX = [robotX-1, robotX-1, robotX-2];
                        possibleObstacleY = [robotY+1, robotY, robotY];
                    elseif ((robotX < sim.map.maxX) && (robotX > 1))
                        possibleObstacleX = [robotX-1, robotX-1];
                        possibleObstacleY = [robotY+1, robotY];
                    end
                elseif (robotY == sim.map.maxY)
                    if ((robotX > 2) && (sim.map.coordinates(robotX-1, robotY) ~= sim.map.legend.obstacle))
                        possibleObstacleX = [robotX-1, robotX-2, robotX-1];
                        possibleObstacleY = [robotY, robotY, robotY-1];
                    elseif ((robotX < sim.map.maxX) && (robotX > 1))
                        possibleObstacleX = [robotX-1, robotX-1];
                        possibleObstacleY = [robotY, robotY-1];
                    end
                end
            end
            
            obstacles = [possibleObstacleX', possibleObstacleY'];
            
        end
        
        function updateLineOfSight(sim)
        % EXAMPLE FUNCTION CALL: updateLineOfSight()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-10-11
        % PURPOSE: Updates line of sight of robot
            
            obstacles = sim.getLineOfSight();
            if (~isempty(obstacles))
                sim.addObstacles(obstacles);
            end
            
        end
        
        function addObstacles(sim, obstacles)
        % EXAMPLE FUNCTION CALL: updateLineOfSight()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-10-11
        % PURPOSE: Check if provided obstacles already exist in the map and display them if not
            
            obstacleAlreadyDisplayed = 0;
            for obstacle = 1:size(obstacles, 1)
                if (sim.map.coordinates(obstacles(obstacle, 1), obstacles(obstacle, 2)) == sim.map.legend.obstacle)
                    for previousObstacle = 1:size(sim.display.displayedObstacles, 1)
                        if (obstacles(obstacle, 1) == sim.display.displayedObstacles(previousObstacle, 1))
                            if (obstacles(obstacle, 2) == sim.display.displayedObstacles(previousObstacle, 2))
                                obstacleAlreadyDisplayed = 1;
                                break;
                            end
                        end
                    end
                    
                    if (~obstacleAlreadyDisplayed)
                        sim.display.displayedObstacles = [sim.display.displayedObstacles; obstacles(obstacle, :)];
                        if (sim.display.on)
                            displayLine = displayObstacle(sim.map.coordinates, obstacles(obstacle, 1), obstacles(obstacle, 2), ...
                                                          sim.robot.location, sim.robot.direction, sim.map.legend, sim.display.displayedLines, true);
                        end
                        sim.robot.map(obstacles(obstacle, 1), obstacles(obstacle, 2)) = sim.robot.legend.obstacle;
                    else
                        obstacleAlreadyDisplayed = 0;
                        if (sim.display.on)
                            displayLine = displayObstacle(sim.map.coordinates, obstacles(obstacle, 1), obstacles(obstacle, 2), ...
                                                          sim.robot.location, sim.robot.direction, sim.map.legend, sim.display.displayedLines, false);
                        end
                    end
                    if ((sim.display.on) && (displayLine(1) ~= 0))
                        sim.display.displayedLines = [sim.display.displayedLines; displayLine];
                    end
                end
            end
            drawnow;
            
            function [displayLine] = displayObstacle(map, x, y, robotLocation, robotDirection, legend, displayedLines, displayObstacleDot)
            % EXAMPLE FUNCTION CALL: displayObstacle(sim.map.coordinates, x, y, sim.robot.location, sim.robot.direction, sim.map.legends, 1)
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
        
        
        % Display Functions
        
        function initializeFigure(sim)
        % EXAMPLE FUNCTION CALL: initializeFigure()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-10-12
        % PURPOSE: Initialize figure for displaying maps
            
            sim.display.figureHandle = figure('Name', 'A* Algorithm', 'NumberTitle', 'off'); % initialize figure
            figurePosition = get(sim.display.figureHandle, 'Position');
            set(sim.display.figureHandle, 'Position', [figurePosition(1:2), figurePosition(3)*1.5, figurePosition(4)]);
            
            subplot(121);
            axis([0.5, sim.map.maxX+0.5, 0.5, sim.map.maxY+0.5]); % initialize axis spacing
            axis square; grid on; hold on; % set axis properties
            subplot(122);
            axis([0.5, sim.map.maxX+0.5, 0.5, sim.map.maxY+0.5]); % initialize axis spacing
            axis square; grid on; hold on; % set axis properties
            
        end
        
        function displayMap(sim)
        % EXAMPLE FUNCTION CALL: sim.displayMap()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-10-11
        % PURPOSE: Display the map with only objects that the sim can see

            if (sim.display.on)
                figure(sim.display.figureHandle); subplot(121);
                title('Map from Robot');
            end
            
            % Display obstacles, robot, and target location
            for x = 1:sim.map.maxX % for all map x locations
                for y = 1:sim.map.maxY % for all map y locations
                    if (sim.map.coordinates(x, y) == sim.map.legend.start) % if the current locaiton is the robot
                        sim.robot.location = [x, y];
                        if (sim.display.on)
                            sim.display.handles.robot = plot(x, y, 'bo'); % show the robot
                        end
                        sim.robot.map(x, y) = sim.robot.legend.start;
                        sim.robot.location  = [x, y];
                        sim.display.initialRobotLocation = sim.robot.location;
                    end
                end
            end
            drawnow; % force MATLAB to draw the figure before continuing
            
        end
        
        function displayFullMap(sim)
        % EXAMPLE FUNCTION CALL: sim.displayFullMap()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Display the map with obstacles, start location, and target location
            
            if (sim.display.on)
                figure(sim.display.figureHandle); subplot(122);
                title('Full Map');
            end
            
            % Display obstacles, robot, and target location
            for x = 1:sim.map.maxX % for all map x locations
                for y = 1:sim.map.maxY % for all map y locations
                    if (sim.map.coordinates(x, y) == sim.map.legend.obstacle) % if the current location is an obstacle
                        sim.display.obstacleCount = sim.display.obstacleCount + 1; % increment obstacle counter
                        if (sim.display.on)
                            plot(x, y, 'ro'); % show the obstacle
                            displayBorders(sim.map.coordinates, x, y, sim.map.legend.obstacle); % display connected borders
                        end
                    elseif ((sim.display.on) && (sim.map.coordinates(x, y) == sim.map.legend.target)) % if the current location is the target
                        plot(x, y, 'gd'); % show the target
                    elseif ((sim.display.on) && (sim.map.coordinates(x, y) == sim.map.legend.start)) % if the current locaiton is the robot
                        plot(x, y, 'bo'); % show the robot
                    end
                end
            end
            drawnow; % force MATLAB to draw the figure before continuing
            
            function displayBorders(map, x, y, obstacle)
            % EXAMPLE FUNCTION CALL: displayBorders(sim.map.coordinates, x, y, sim.map.legend.obstacle)
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
        
    end
    
end


