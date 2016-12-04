%MAPGENERATOR   Class that allows for the user to create maps for A* algorithm
%
%   MAPGENERATOR
%   initializes the map generator class
%
%   MAPGENERATOR.generateMap
%   runs the relevent functions to get desired format from user and then
%   brings up a figure that the user can then use to generate a map.
%
%   See also RUNMAPGENERATOR, EXAMPLESCRIPT, ASTAR_STRUCTURE_FAST.
%
%   Please refer to the wiki for more information: 
%   https://github.com/FWachter/Micromouse/wiki/MATLAB

%% ---- PROGRAM INFORMAITON ----
% PROGRAMMER: Frederick Wachter
% DATE CREATED: 2016-06-01
% PURPOSE: Micromouse map generator
% REFERENCE: http://ch.mathworks.com/matlabcentral/fileexchange/26248-a---a-star--search-for-path-planning-tutorial
% CONTACT INFO: wachterfreddy@gmail.com

% Please refer to the Wiki for instructions on how to use this script
% GITHUB WIKI: https://github.com/FWachter/Micromouse/wiki/MATLAB


classdef mapGenerator < handle
    
    properties(SetAccess = protected)
        limits
        legend
        data
        criteria
        inputs
        state
        window
    end
    

%% CONSTRUCTOR METHOD
    methods
        
        function map = mapGenerator(varargin)
        % EXAMPLE FUNCTION CALL: map = mapGenerator([20,20]);
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Constructs the structure
            
            % Initialize Map Limits
            map.limits.MAX_X = 0;
            map.limits.MAX_Y = 0;
        
            % Default Map Legend
            map.legend.obstacle  = -1;
            map.legend.target    = 0;
            map.legend.start     = 1;
            map.legend.freeSpace = 2;
            
            % Initialize Counters
            map.inputs.obstacles      = 0;
            map.inputs.targetLocation = zeros(0,0);
            map.inputs.startLocation  = zeros(0,0);
            
            % Initialize Map Criteria
            map.criteria = 0;
            
            % Initialize Window Properties
            map.window.handle         = -1;
            map.window.resultFigure   = -1;
            map.window.obstacleHandle = -1;
            map.window.obstacleHandle = -1;
            map.window.obstacleHandle = -1;
            
            % Intiialize State Properties
            map.state.ready         = 0;
            map.state.getDimensions = 0;
            map.state.getCriteria   = 0;
            
        end
        
    end

    
%% PUBLIC METHODS
    methods
        
        function generateMap(map) 
        % EXAMPLE FUNCTION CALL: map.generateMap();
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Calls all relevent functions to create and save a map
        % TYPE: Public Method
        
            % Set Properties
            map.getMapDimensions();
            map.getMapCriteria();
            map.setMapLegend();
            
            % Get Map Elements
            map.getElements();
            
            % Export the Map and Show Generated Map
            map.exportData();
            map.showMap();
            
        end
        
        function getMapDimensions(map)
        % EXAMPLE FUNCTION CALL: map.getMapDimensions();
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Set the limits of the map
        % TYPE: Public Method
            
            while (map.state.getDimensions == 0)
                map.limits.MAX_X = floor(input('\nSize of Map (X): ')); % get max x value for map
                map.limits.MAX_Y = floor(input('Size of Map (Y): ')); % get max y value for map
                
                if ((map.limits.MAX_X > 0) && (map.limits.MAX_Y > 0))
                    map.state.getDimensions = 1; % set the state flag to indicate that map dimensions have been set
                else
                    fprintf('[Error] Map limits have not been set properly.\nRetry inputs and ensure that inputs are integers and greater than 0\n\n');
                end
            end
                
            map.data = map.legend.freeSpace * ones(map.limits.MAX_X,map.limits.MAX_Y); % initialize map
            
        end
        
        function getMapCriteria(map)
        % EXAMPLE FUNCTION CALL: map.getMapCriteria();
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-02
        % PURPOSE: Set the move criteria for the map
        % TYPE: Public Method
            
            fprintf('\nSet the map criteria (listed below)\n');
            fprintf('  1: Orthogonal movements\n');
            fprintf('  2: Orthogonal and diagonal movements\n');
            
            mapCriteriaSet = 0;
            while (mapCriteriaSet == 0)
                map.criteria = input('\nCriteria (1 or 2): ');
                if (~(isempty(map.criteria)) && ((map.criteria == 1) || (map.criteria == 2)))
                    mapCriteriaSet = 1;
                else
                    fprintf('Please choose either 1 or 2\n');
                end
            end
            
            map.state.getCriteria = 1; % set the state flag to indicate that the map cirteria has been set
            
        end
        
        function setMapLegend(map)
        % EXAMPLE FUNCTION CALL: map.setMapLegend();
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Reset the map legend away from default value
            
            setLegend = input('\nWould you like to customize the legend? (y/n) ','s');
            
            if ((setLegend == 'y') || (setLegend == 'Y'))
                map.legend.obstacle  = input('Obstacle Location Legend: '); % set the obstacle location legend value
                map.legend.target    = input('Target Location Legend: '); % set the target location legend value
                map.legend.start     = input('Start Location Legend: '); % set the start location legend value
                map.legend.freeSpace = input('Free Space Legend: '); % set the free space location legend value

                map.data = map.legend.freeSpace * ones(map.limits.MAX_X,map.limits.MAX_Y); % reset the map to desired legend
            end
            
        end
        
        function exportData(map)
        % EXAMPLE FUNCTION CALL: map.exportData();
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Export map data
        % TYPE: Public Method
            
            % Set the data ready to be exported
            export.map.data = map.data;
            export.map.legend = map.legend;
            export.map.criteria = map.criteria;
            
            % Save the map to user specified location
            fprintf('\nSaving map... ');
            [fileName,pathName] = uiputfile('map.mat','Save Map');
            save([pathName,fileName],'-struct','export','map');
            fprintf('Map Saved.\nThe map can be found at: %s\n',[pathName,fileName]);
            
            % Ask user if they would like the data to be brought into the workspace
            loadData = input('\nWould you like to load the map into the workspace? (y/n) ','s');
            if ((loadData == 'y') || (loadData == 'Y'))
                load([pathName,fileName]);
                assignin('base','map',map);
            end
            
            fprintf('\n'); % add a new line

        end
        
        function showMap(map)
        % EXAMPLE FUNCTION CALL: map.showMap()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-03
        % PURPOSE: Display the newly generated map
        % TYPE: Public Method  
            
            % Initialize the figure
            map.window.resultFigure = figure('Name','Generated Map','NumberTitle','off'); % initialize figure
            axis([1,map.limits.MAX_X+1,1,map.limits.MAX_Y+1]); % initialize axis spacing
            axis square; grid on; hold on; % set axis properties
            
            obstacleCount = numel(find(map.data == map.legend.obstacle));
            
            % Display obstacles, robot, and target location
            for x = 1:map.limits.MAX_X % for all map x locations
                for y = 1:map.limits.MAX_Y % for all map y locations
                    if (map.data(x,y) == map.legend.obstacle) % if the current location is an obstacle
                        map.inputs.obstacles = map.inputs.obstacles + 1; % increment obstacle counter
                        map.window.obstacleHandle(obstacleCount) = plot(x+.5,y+.5,'ro'); % show the obstacle
                    elseif (map.data(x,y) == map.legend.target) % if the current location is the target
                        map.window.targetHandle = plot(x+.5,y+.5,'gd'); % show the target
                    elseif (map.data(x,y) == map.legend.start) % if the current locaiton is the robot
                        map.window.startHandle = plot(x+.5,y+.5,'bo'); % show the robot
                    end
                end
            end
            drawnow; % force MATLAB to draw the figure before continuing
            
        end
        
        function getElements(map)
        % EXAMPLE FUNCTION CALL: map.getElements()
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-03
        % PURPOSE: Master function for getting map elements from user
        % TYPE: Public Method  
            
            % Setup Figure
            map.setupFigure();
            
            % Add Map Elements
            map.selectTarget();
            map.selectObstacles();
            map.selectStart();
            
            % Close the Figure
            map.closeFigure();
            
        end
        
    end
    
    
%% PRIVATE METHODS
    methods (Access = private)
        
        function setupFigure(map)
        % EXAMPLE FUNCTION CALL: Can't run function externally
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Initialized the figure
        % TYPE: Private Method
            
            map.window.handle = figure('Name','Map Generator','NumberTitle','off');
            axis([1,map.limits.MAX_X+1,1,map.limits.MAX_Y+1]);
            grid on; hold on;
            
            map.window.leftMouseButton = 1;
            
        end
        
        function closeFigure(map)
        % EXAMPLE FUNCTION CALL: Can't run function externally
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-03
        % PURPOSE: Initialized the figure
        % TYPE: Private Method
        
            if (ishandle(map.window.handle))
                delete(map.window.handle)
            end
        
        end
            
        function selectTarget(map)
        % EXAMPLE FUNCTION CALL: Can't run function externally
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Get desired target location from the user
        % TYPE: Private Method
            
            % Display Instructions to the User
            msgHandle = msgbox('Please select the target location using the left mouse button');
            uiwait(msgHandle,5); % wait for user to click 'OK' or wait 5 seconds
            if (ishandle(msgHandle) == 1) % if the msgbox still exists, close it
                delete(msgHandle);
            end
            
            % Get the Desired Target Location on the Map
            title('Please select the target location using the left mouse button');
            buttonPressed = 0;
            while (buttonPressed ~= map.window.leftMouseButton) % while the left mouse button hasn't been pressed
                [xLocation,yLocation,buttonPressed] = ginput(1); % get user click position
            end
            
            map.inputs.targetLocation = floor([xLocation,yLocation]); % store the location of the target
            
            % Display Target Location on the Map
            map.data(map.inputs.targetLocation(1),map.inputs.targetLocation(2)) = map.legend.target;
            plot(map.inputs.targetLocation(1) + 0.5, map.inputs.targetLocation(2) + 0.5,'gd');
            text(map.inputs.targetLocation(1) + 1,map.inputs.targetLocation(2) + 0.5,'Target');
            
        end
        
        function selectObstacles(map)
        % EXAMPLE FUNCTION CALL: Can't run function externally
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Get desired obstacle locations from the user
        % TYPE: Private Method
            
            % Display Instructions to the User
            msgHandle = msgbox('Please select obstacle locations using the left mouse button, to select the last obstacle use the right mouse button');
            uiwait(msgHandle,5); % wait for user to click 'OK' or wait 5 seconds
            if (ishandle(msgHandle) == 1) % if the msgbox still exists, close it
                delete(msgHandle);
            end
            
            % Get the Desired Obstacle Locations from the Map
            title('Please select obstacle locations using the left mouse button, to select the last obstacle use the right mouse button','Color','blue');
            buttonPressed = 1;
            while (buttonPressed == map.window.leftMouseButton) % while the left button is being pressed per mouse click
                [xLocation,yLocation,buttonPressed] = ginput(1); % get user click position
                map.data(floor(xLocation),floor(yLocation)) = map.legend.obstacle; % store the obstacle location into the map
                plot(floor(xLocation) + 0.5,floor(yLocation) + 0.5,'ro'); % plot the location on the map
            end
            
        end
        
        function selectStart(map)
        % EXAMPLE FUNCTION CALL: Can't run function externally
        % PROGRAMMER: Frederick Wachter
        % DATE CREATED: 2016-06-01
        % PURPOSE: Get desired start location from the user
        % TYPE: Private Method
            
            % Display Instructions to the User
            msgHandle = msgbox('Please select the start location using the left mouse button');
            uiwait(msgHandle,5); % wait for user to click 'OK' or wait 5 seconds
            if (ishandle(msgHandle) == 1) % if the msgbox still exists, close it
                delete(msgHandle);
            end
            
            % Get the Desired Start Location on the Map
            title('Please select the start location using the left mouse button');
            buttonPressed = 0;
            while (buttonPressed ~= map.window.leftMouseButton) % while the left mouse button hasn't been pressed
                [xLocation,yLocation,buttonPressed] = ginput(1); % get user click position
            end
            
            map.inputs.startLocation = floor([xLocation,yLocation]); % store the location of the start
            
            % Display Start Location on the Map
            map.data(map.inputs.startLocation(1),map.inputs.startLocation(2)) = map.legend.start;
            plot(map.inputs.startLocation(1) + 0.5,map.inputs.startLocation(2) + 0.5,'bo');
            text(map.inputs.startLocation(1) + 1,map.inputs.startLocation(2) + 0.5,'Start');
            
        end
        
    end


end


