%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
% DATE CREATED: 2016-12-14
% PURPOSE: Micromouse functions class

% Please refer to the Wiki for instructions on how to use this class
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB


classdef micromouse < handle
    
    properties
        location
        direction
        openDirections
        map
    end
    
    properties(GetAccess = private)
        goal
    end

%% CONSTRUCTOR METHOD
    methods
        
        function robot = micromouse(varargin)
        % EXAMPLE FUNCTION CALL: nodes = node(varargin)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Construct the structure
            
            robot.location       = zeros(1, 2);
            robot.direction      = 0;
            robot.openDirections = zeros(1, 4);
            robot.map            = [];
            
            robot.goal.locations = zeros(5, 2);
        
        end
        
    end
    
%% PUBLIC METHODS
    
    methods
        
        function location = addCurrentLocation(robot, location)
        % EXAMPLE FUNCTION CALL: robot.addCurrentLocation(location)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Adds provided location to array used to check for if the robot is in the goal position
            
            robot.goal.locations(2:5, :) = robot.goal.locations(1:4, :);
            robot.goal.locations(1, :)   = robot.location;
            
        end
        
        function goalLocationFound = isAtGoalLocation(robot)
        % EXAMPLE FUNCTION CALL: robot.isAtGoalLocation()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Checks if the robot is at the goal position
            
            goalLocationFound = 0;
            if (sum(robot.goal.locations(1, :) == robot.goal.locations(5, :)) == 2) % if the robot found the goal
                directions = zeros(1, 4);
                for index = 1:4 % check the previous four directions traveled
                    directions(index) = robot.getTravelDirection(robot.goal.locations(index, :), robot.goal.locations(index+1, :));
                end
                
                for index = 1:3
                    if (robot.getOppositeDirection(directions(index)) == directions(index+1)) % if the set of locations was caused by a dead end not goal location
                        directions = zeros(1, 4);
                    end
                end
                
                if (length(unique(directions)) == 4) % if the robot went in a circle in the goal location
                    goalLocationFound = 1; 
                    fprintf('Solution found! Backtracking to remaining unsearched area\n');
                end
            end
            
        end
 
    end

    methods (Static = true)
        
        function oppositeDirection = getOppositeDirection(direction)
        % EXAMPLE FUNCTION CALL: robot.getOppositeDirection(direction)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Get the opposite direction of a give direction
           
            oppositeDirection = mod(direction+2, 4);
            if (oppositeDirection == 0)
                oppositeDirection = 4;
            end
            
        end
        
        function direction = getTravelDirection(origin, goal)
        % EXAMPLE FUNCTION CALL: robot.getTravelDirection(origin, goal)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Gets the neccessary direction of travel for going from an origin to a goal location
        % ASSUMPTION(S): A line can be drawn from the origin to the goal along one of the cartesian coordinates (same x or same y)
            
            if (goal(1) == origin(1))
                if (goal(2) > origin(2))
                    direction = 1;
                else
                    direction = 3;
                end
            else
                if (goal(1) > origin(1))
                    direction = 2;
                else
                    direction = 4;
                end
            end
            
        end
        
        function direction = chooseRandomDirection(openDirections)
        % EXAMPLE FUNCTION CALL: robot.chooseRandomDirection(openDirections)
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-12-14
        % PURPOSE: Choose a random direction from given open directions
            
            directions = find(openDirections == 1);
            randIndex = randi([1, length(directions)], 1);
            direction = directions(randIndex);
            
        end
        
    end 

%% PRIVATE METHODS

    methods (Access = private)
        
    end
    
end


